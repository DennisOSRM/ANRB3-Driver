//! The three-stage demodulator.
//!
//! The device sends 8 samples per Mode-S bit as packed bits, one per sample.
//!
//!   stage 1  skip leading zeros, keep every bit from the first 1 onward
//!   stage 2  sum groups of 4 samples, threshold with optional hysteresis
//!   stage 3  PPM: keep the even-indexed half-bit decisions
//!
//! Stage 2's group sums are the nibble popcounts of the packed stream once it
//! has been aligned so the first 1 bit lands on a byte boundary, so the stream
//! is never expanded to one byte per sample.

use crate::simd::{self, Backend};

use crate::profile::fine_tick as tick;

/// Cycle counts inside the demodulator (`profile_fine`), plus memo hit/miss
/// counts (`profile`).
#[derive(Default, Clone, Copy, Debug)]
pub struct StagePhases {
    /// Finding the first set bit.
    pub find_s: u64,
    /// Shifting the samples so that bit starts a byte.
    pub shift: u64,
    /// Nibble popcounts: the group sums.
    pub popcount: u64,
    /// The soft metric per bit.
    pub margin: u64,
    /// Thresholding without hysteresis.
    pub thresh_plain: u64,
    /// Thresholding with hysteresis.
    pub thresh_hyst: u64,
    /// Runs that reused the group sums.
    pub memo_hits: u64,
    /// Runs that computed the group sums.
    pub memo_misses: u64,
}

const MAX_GROUPS: usize = 4096;
const SHBUF: usize = 2048;

/// Most bits [`Stages::run`] and [`Stages::run_ref`] return: one per pair of
/// groups.
pub const MAX_BITS: usize = MAX_GROUPS / 2;

/// Scratch buffers and the memo for one decoding context.
pub struct Stages {
    shbuf: [u8; SHBUF],
    accb: [u8; MAX_GROUPS],
    /// PPM soft metric per decoded bit: energy in the first half of the bit
    /// minus the second. Sign gives the bit, magnitude the confidence (0..4).
    pub margin: [i8; MAX_GROUPS],
    generation: u32,
    /// The (burst generation, S) the group sums were last built for, if any.
    memo: Option<(u32, Option<usize>)>,
    memo_ng: usize,
    backend: Backend,
    pub phases: StagePhases,
}

/// Leading zero samples placed in front of the sample stream.
///
/// `rb.dll` stage 1 prepends four when the burst is 100 bytes or shorter and
/// none otherwise - short bursts carry a 56-bit reply, long ones a 112-bit
/// squitter. Four samples are one stage-2 group, so prepending them shifts the
/// group grid by half a Mode-S bit, which swaps which groups stage 3 reads as
/// the first half of a bit and which as the second. This selects the half-bit
/// phase, and the two burst lengths need opposite phases.
///
/// Both branches are required: applying either rule to both lengths loses
/// about 99% of frames in the other class.
fn prefix_samples(wl: usize) -> usize {
    if wl <= 100 { 4 } else { 0 }
}

impl Stages {
    /// Empty buffers, and the fastest backend this CPU supports.
    pub fn new() -> Self {
        Stages {
            shbuf: [0; SHBUF],
            accb: [0; MAX_GROUPS],
            margin: [0; MAX_GROUPS],
            generation: 0,
            memo: None,
            memo_ng: 0,
            backend: Backend::detect(),
            phases: StagePhases::default(),
        }
    }

    /// Invalidate the memo. Call once per burst, before the grid search.
    pub fn new_burst(&mut self) {
        self.generation = self.generation.wrapping_add(1);
    }

    /// Run the three stages for one (skipbits, gt, bias) configuration.
    /// Returns the number of decoded bits written to `out`.
    pub fn run(&mut self, w: &[u8], skipbits: usize, gt: i32, bias: bool, out: &mut [u8]) -> usize {
        let wl = w.len();
        let total = wl * 8;
        let pfx = prefix_samples(wl);

        // S: index of the first set bit at or after skipbits. Everything from
        // there on is kept, so S alone determines the group sums.
        let t_s = tick();
        // None when every bit from skipbits on is zero.
        let s = (skipbits..total).find(|&i| (w[i >> 3] >> (7 - (i & 7))) & 1 != 0);

        self.phases.find_s += tick().wrapping_sub(t_s);
        let n = pfx + s.map_or(0, |s| total - s);
        if n == 0 {
            return 0;
        }
        let mut ng = ((n - 1) / 4 + 1).min(MAX_GROUPS);
        // Whole groups of zeros in front. The prefix is a multiple of 4 by
        // construction, so the real data always starts on a group boundary.
        let g0 = pfx >> 2;

        // The shift-and-popcount work depends only on S, which is unchanged
        // across most of the grid, so a single-entry memo removes nearly all
        // of the repeated effort.
        if self.memo == Some((self.generation, s)) {
            ng = self.memo_ng;
            #[cfg(feature = "profile")]
            { self.phases.memo_hits += 1; }
        } else {
            #[cfg(feature = "profile")]
            { self.phases.memo_misses += 1; }
            self.memo = Some((self.generation, s));
            self.memo_ng = ng;

            for k in 0..g0.min(ng) {
                self.accb[k] = 0;
            }
            if let Some(s) = s {
                let r = total - s; // bits available from S
                let nby = ((r + 7) >> 3).min(SHBUF - 1);
                let t_sh = tick();
                let byo = s >> 3;
                let sh = s & 7;
                if sh == 0 {
                    for i in 0..nby {
                        self.shbuf[i] = if byo + i < wl { w[byo + i] } else { 0 };
                    }
                } else {
                    for i in 0..nby {
                        let a = byo + i;
                        let hi = if a < wl { w[a] << sh } else { 0 };
                        let lo = if a + 1 < wl { w[a + 1] >> (8 - sh) } else { 0 };
                        self.shbuf[i] = hi | lo;
                    }
                }
                // Bits past the end of the burst must read as zero.
                if r & 7 != 0 && nby > 0 {
                    self.shbuf[nby - 1] &= 0xFFu8 << (8 - (r & 7));
                }
                self.phases.shift += tick().wrapping_sub(t_sh);
                let t_pc = tick();
                simd::nibble_groups(self.backend, &self.shbuf, &mut self.accb, g0, ng);
                self.phases.popcount += tick().wrapping_sub(t_pc);
            } else {
                for k in g0..ng {
                    self.accb[k] = 0;
                }
            }
        }

        let m = (ng / 2).min(out.len());

        let t_mg = tick();
        simd::margin(self.backend, &self.accb, &mut self.margin, m, ng);
        self.phases.margin += tick().wrapping_sub(t_mg);

        if !bias {
            // No hysteresis means no serial dependency and the odd half-bits
            // are discarded anyway, so only the even ones are computed.
            let t_th = tick();
            simd::threshold_even(self.backend, &self.accb, out, m, ng, gt);
            self.phases.thresh_plain += tick().wrapping_sub(t_th);
        } else {
            // See `simd::threshold_hyst`.
            let t_th = tick();
            simd::threshold_hyst(self.backend, &self.accb, out, m, ng, gt);
            self.phases.thresh_hyst += tick().wrapping_sub(t_th);
        }
        m
    }

    /// Reference version: expands to one byte per sample and runs each stage
    /// in turn. Kept so the fast path can be checked against it.
    pub fn run_ref(&self, w: &[u8], skipbits: usize, gt: i32, bias: bool, out: &mut [u8]) -> usize {
        let wl = w.len();
        let pfx = prefix_samples(wl);
        let mut bits: Vec<u8> = Vec::with_capacity(wl * 8 + pfx);
        bits.extend(std::iter::repeat_n(0u8, pfx));
        let mut started = false;
        let mut bitidx = 0usize;
        for &byte in w {
            for k in (0..8).rev() {
                let v = (byte >> k) & 1;
                bitidx += 1;
                if bitidx - 1 < skipbits {
                    continue;
                }
                if v != 0 {
                    bits.push(1);
                    started = true;
                } else if started {
                    bits.push(0);
                }
            }
        }
        let n = bits.len();
        if n == 0 {
            return 0;
        }
        let ng = ((n - 1) / 4 + 1).min(MAX_GROUPS);
        let mut acc = vec![0i32; ng];
        for (i, &b) in bits.iter().enumerate() {
            let gi = i >> 2;
            if gi < ng {
                acc[gi] += b as i32;
            }
        }
        let mut o = vec![0u8; ng];
        o[0] = u8::from(acc[0] > gt);
        for k in 1..ng {
            let b = if bias { if o[k - 1] == 0 { 1 } else { -1 } } else { 0 };
            o[k] = u8::from(acc[k] + b > gt);
        }
        let m = (ng / 2).min(out.len());
        for k in 0..m {
            out[k] = o[2 * k];
        }
        m
    }
}

impl Default for Stages {
    fn default() -> Self { Self::new() }
}

/// Pack one-bit-per-byte decisions into a frame.
pub(crate) fn packbits(bits: &[u8], nbits: usize, fr: &mut [u8]) {
    for b in fr[..nbits / 8].iter_mut() {
        *b = 0;
    }
    for i in 0..nbits {
        if bits[i] != 0 {
            fr[i >> 3] |= 0x80 >> (i & 7);
        }
    }
}
