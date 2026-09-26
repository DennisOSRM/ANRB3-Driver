//! Mode-S burst decoding: search the framing grid, validate with CRC, correct
//! errors. Samples in, [`Frame`]s out; nothing here knows how the device
//! delivers the samples or what an ADS-B message means.
//!
//! * `frame` - the recovered frame and its fields
//! * `gate` - the plausibility check a CRC match must also pass
//! * `whitelist` - trusted addresses, for the formats that cannot self-validate
//! * `stats` - counters, options and profiling phases
//! * this file - the decoder itself: the grid search and correction passes

mod frame;
mod gate;
mod stats;
mod whitelist;

pub use frame::Frame;
pub use stats::{Options, Phases, Stats};
use whitelist::Whitelist;

use crate::crc::{ApMap, Crc};
use crate::profile::tick;
use crate::stages::{packbits, Stages, MAX_BITS};

/// One demodulator configuration in the search grid.
#[derive(Clone, Copy)]
struct Cfg { skipbits: usize, gt: i32, bias: bool }

/// The Mode-S decoder: one burst of samples in, at most one [`Frame`] out.
/// Holds the working buffers and the address whitelist, so reuse one decoder
/// for a whole stream.
pub struct Decoder {
    crc: Crc,
    ap: ApMap,
    stages: Stages,
    grid: Vec<Cfg>,
    wl: Whitelist,
    dec: Vec<u8>,
    /// Demodulator output cached per grid configuration for the current burst.
    /// The correction passes walk the same grid, and what the
    /// demodulator produces depends only on the configuration and the burst -
    /// not on how hard the pass is willing to work - so it is computed once
    /// and reused. Filled lazily: an early success must not pay for
    /// configurations nobody looks at.
    cache_bits: Vec<u8>,
    cache_margin: Vec<i8>,
    cache_nb: Vec<i32>,
    cache_stride: usize,
    /// Viable framings per configuration: offsets whose first five bits are a
    /// downlink format the decoder handles and whose frame fits in the decoded
    /// bits. Computed once in `stage_cached` and shared by every pass.
    frame_st: Vec<u8>,
    frame_nbytes: Vec<u8>,
    frame_pure: Vec<bool>,
    frame_n: Vec<u8>,
    pub opts: Options,
    pub stats: Stats,
    pub phases: Phases,
}

impl Decoder {
    pub fn new() -> Self {
        // Ordered so that productive configurations are tried first (the
        // search returns on the first hit) while keeping equal skipbits
        // adjacent, which is what the memo in Stages needs to pay off.
        let sb_order = [1usize, 0, 2, 3, 4, 5, 6, 7, 8];
        let gt_order = [1i32, 2];
        let mut grid = Vec::with_capacity(sb_order.len() * gt_order.len() * 2);
        for &skipbits in &sb_order {
            for &gt in &gt_order {
                for bias in [true, false] {
                    grid.push(Cfg { skipbits, gt, bias });
                }
            }
        }
        let crc = Crc::new();
        let ap = ApMap::new(&crc);
        let ncfg = grid.len();
        Decoder {
            crc,
            ap,
            stages: Stages::new(),
            grid,
            wl: Whitelist::new(),
            dec: vec![0u8; MAX_BITS],
            cache_bits: Vec::new(),
            cache_margin: Vec::new(),
            cache_nb: Vec::new(),
            cache_stride: 0,
            frame_st: vec![0; ncfg * Self::FRAMES_MAX],
            frame_nbytes: vec![0; ncfg * Self::FRAMES_MAX],
            frame_pure: vec![false; ncfg * Self::FRAMES_MAX],
            frame_n: vec![0; ncfg],
            opts: Options::default(),
            stats: Stats::default(),
            phases: Phases::default(),
        }
    }

    /// Addresses the whitelist holds.
    pub fn whitelist_len(&self) -> usize { self.wl.count() }
    /// Addresses dropped from the whitelist to make room for newer ones. A
    /// receiver that has been up long enough to fill it keeps learning, and
    /// this says how hard it is having to work at that.
    pub fn whitelist_evicted(&self) -> u64 { self.wl.evicted }

    /// Most framing offsets a configuration can yield: start positions 0
    /// through 20.
    const FRAMES_MAX: usize = 21;

    /// Demodulate configuration `c`, or return the cached result.
    fn stage_cached(&mut self, c: usize, w: &[u8]) -> usize {
        if self.cache_nb[c] >= 0 {
            return self.cache_nb[c] as usize;
        }
        let cfg = self.grid[c];
        let t0 = tick();
        let nb = self.stages.run(w, cfg.skipbits, cfg.gt, cfg.bias, &mut self.dec);
        let t1 = tick();
        self.stats.stage_runs += 1;
        let off = c * self.cache_stride;
        let n = nb.min(self.cache_stride);
        self.cache_bits[off..off + n].copy_from_slice(&self.dec[..n]);
        self.cache_margin[off..off + n].copy_from_slice(&self.stages.margin[..n]);
        self.cache_nb[c] = n as i32;
        let t2 = tick();
        self.phases.demod += t1.wrapping_sub(t0);
        self.phases.cache_copy += t2.wrapping_sub(t1);

        // Classify the framings once, while the bits are hot in cache.
        let base = c * Self::FRAMES_MAX;
        let mut k = 0usize;
        if n >= 56 {
            let maxst = (n - 56).min(20);
            // Read the five format bits directly per offset. A rolling window
            // needs one load instead of five but serialises them, and measured
            // slightly slower: the independent loads issue in parallel where
            // the carried value does not.
            for st in 0..=maxst {
                let db = &self.cache_bits[off + st..off + n];
                let df = (db[0] << 4) | (db[1] << 3) | (db[2] << 2) | (db[3] << 1) | db[4];
                let nbits = if df < 16 { 56usize } else { 112 };
                if st + nbits > n { continue; }
                let pure = matches!(df, 11 | 17 | 18);
                if !pure && !matches!(df, 0 | 4 | 5 | 16 | 20 | 21) { continue; }
                self.frame_st[base + k] = st as u8;
                self.frame_nbytes[base + k] = (nbits / 8) as u8;
                self.frame_pure[base + k] = pure;
                k += 1;
            }
        }
        self.frame_n[c] = k as u8;
        n
    }

    fn pass(&mut self, w: &[u8], maxbits: u8, overlaid_mode: bool, soft: bool, ms: u32) -> Option<Frame> {
        // 0 = clean, 1 = 1-bit, 2 = soft, 3 = overlaid, 4 = blind 2-bit
        let pass_id = if overlaid_mode { 3 } else if soft { 2 } else if maxbits >= 2 { 4 } else { maxbits as usize };
        for c in 0..self.grid.len() {
            self.stats.cfgs_by_pass[pass_id] += 1;
            let nb = self.stage_cached(c, w);
            let off = c * self.cache_stride;
            if nb < 56 {
                continue;
            }
            let base = c * Self::FRAMES_MAX;
            let tf0 = tick();
            for k in 0..self.frame_n[c] as usize {
                self.stats.offsets += 1;
                self.stats.offsets_by_pass[pass_id] += 1;
                let pure = self.frame_pure[base + k];
                // Entries are either pure or address-overlaid, never both, so
                // one comparison selects what this pass wants.
                if pure == overlaid_mode { continue; }
                self.stats.offsets_kept += 1;
                let st = self.frame_st[base + k] as usize;
                let nbytes = self.frame_nbytes[base + k] as usize;
                let nbits = nbytes * 8;
                let db = &self.cache_bits[off + st..off + nb];

                let mut cand = [0u8; 14];
                packbits(db, nbits, &mut cand);
                if cand[..nbytes].iter().all(|&b| b == 0) {
                    continue;
                }

                self.stats.crc_calls += 1;
                if pure {
                    let mut tmp = cand;
                    let ts0 = tick();
                    let r = if soft {
                        let margin = &self.cache_margin[off + st..off + nb];
                        let got = self.crc.fix_soft(&mut tmp[..nbytes], margin, maxbits);
                        if got.searched {
                            self.stats.soft_tried += 1;
                            if matches!(got.bits, Some(1..=2)) {
                                self.stats.soft_hit += 1;
                            }
                        }
                        got.bits
                    } else {
                        self.crc.fix(&mut tmp[..nbytes], maxbits >= 2)
                    };
                    self.phases.soft += tick().wrapping_sub(ts0);
                    if let Some(r) = r {
                        if r <= maxbits && gate::plausible(&tmp[..nbytes]) {
                            self.wl.add(frame::icao(&tmp), ms);
                            self.phases.frame += tick().wrapping_sub(tf0);
                            return Some(Frame { bytes: tmp, len: nbytes as u8, corrected: r, ms });
                        }
                    }
                } else {
                    // Address-overlaid: the syndrome is the CRC of the
                    // address, not the address, so it has to be mapped back
                    // before the whitelist can be consulted. No error
                    // correction - the match must be exact.
                    let syn = self.crc.crc24(&cand[..nbytes]);
                    let icao = self.ap.address(syn);
                    self.stats.overlaid_tried += 1;
                    if self.wl.has(icao) {
                        self.stats.overlaid_hit += 1;
                        self.phases.frame += tick().wrapping_sub(tf0);
                        return Some(Frame { bytes: cand, len: nbytes as u8, corrected: 0, ms });
                    }
                }
            }
            self.phases.frame += tick().wrapping_sub(tf0);
        }
        None
    }

    /// Decode one burst. Passes run cheapest and most trustworthy first so a
    /// genuine frame is never displaced by a corrected noise match.
    pub fn decode_burst(&mut self, w: &[u8], ms: u32) -> Option<Frame> {
        let tb = tick();
        self.stages.new_burst();
        self.stats.bursts += 1;
        // The demodulator emits at most one bit per input byte plus the zero
        // prefix, so size the cache to the burst.
        let stride = (w.len() + 8).min(MAX_BITS);
        let need = stride * self.grid.len();
        if self.cache_bits.len() < need {
            self.cache_bits.resize(need, 0);
            self.cache_margin.resize(need, 0);
        }
        self.cache_stride = stride;
        self.cache_nb.clear();
        self.cache_nb.resize(self.grid.len(), -1);
        let mut found = self.pass(w, 0, false, false, ms);
        if found.is_none() {
            found = self.pass(w, 1, false, false, ms);
        }
        if found.is_none() && self.opts.soft {
            found = self.pass(w, 2, false, true, ms);
        }
        if found.is_none() && self.opts.overlaid {
            found = self.pass(w, 0, true, false, ms);
        }
        if found.is_none() && self.opts.blind2 {
            found = self.pass(w, 2, false, false, ms);
        }
        self.phases.total += tick().wrapping_sub(tb);
        if let Some(f) = found {
            self.stats.frames += 1;
            match f.corrected {
                0 => self.stats.clean += 1,
                1 => self.stats.fixed1 += 1,
                _ => self.stats.fixed2 += 1,
            }
        }
        found
    }

    /// Check the fast demodulator against the reference for one configuration.
    pub fn verify_stages(&mut self, w: &[u8], skipbits: usize, gt: i32, bias: bool) -> Result<(), &'static str> {
        let mut a = vec![0u8; MAX_BITS];
        let mut b = vec![0u8; MAX_BITS];
        let ra = self.stages.run_ref(w, skipbits, gt, bias, &mut a);
        self.stages.new_burst();
        let rb = self.stages.run(w, skipbits, gt, bias, &mut b);
        if ra != rb {
            return Err("length mismatch");
        }
        if a[..ra] != b[..rb] {
            return Err("content mismatch");
        }
        Ok(())
    }

    /// Cycle counts inside the demodulator.
    #[cfg(feature = "profile")]
    pub fn stage_phases(&self) -> crate::stages::StagePhases { self.stages.phases }

    /// Cycle counts inside the soft search.
    #[cfg(feature = "profile")]
    pub fn soft_phases(&self) -> crate::crc::SoftPhases { self.crc.soft_phases.get() }
}

impl Default for Decoder {
    fn default() -> Self { Self::new() }
}

