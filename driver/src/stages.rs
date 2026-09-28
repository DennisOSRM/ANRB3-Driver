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
    if wl <= 100 {
        4
    } else {
        0
    }
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
            {
                self.phases.memo_hits += 1;
            }
        } else {
            #[cfg(feature = "profile")]
            {
                self.phases.memo_misses += 1;
            }
            self.memo = Some((self.generation, s));
            self.memo_ng = ng;

            for k in 0..g0.min(ng) {
                self.accb[k] = 0;
            }
            if let Some(s) = s {
                let r = total - s; // bits available from S
                                   // MAX_GROUPS groups need all SHBUF bytes, so a longer burst
                                   // fills the buffer and is cut at its end.
                let full = (r + 7) >> 3;
                let nby = full.min(SHBUF);
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
                // Bits past the end of the burst must read as zero. When the
                // burst was cut, the last byte holds real samples only.
                if r & 7 != 0 && nby == full {
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
            let b = if bias {
                if o[k - 1] == 0 {
                    1
                } else {
                    -1
                }
            } else {
                0
            };
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
    fn default() -> Self {
        Self::new()
    }
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

#[cfg(test)]
mod tests {
    use super::*;

    /// A burst longer than the buffers is cut after MAX_GROUPS groups, and
    /// the last group is read from the burst's own samples. Here the kept
    /// samples start at bit 3 and are all ones from bit 8 on, so the last
    /// two groups both hold 4 and the last margin is 0.
    #[test]
    fn an_oversized_burst_keeps_its_last_group() {
        let mut w = vec![0xFF; 2100];
        w[0] = 0x10;
        let mut st = Stages::new();
        let mut out = vec![0u8; MAX_BITS];
        assert_eq!(st.run(&w, 0, 1, false, &mut out), MAX_BITS);
        assert_eq!(st.margin[MAX_BITS - 1], 0);
        assert!(out.iter().skip(1).all(|&b| b == 1));
    }

    use crate::decode::synth::{bit, burst, hex, noise};

    const EVEN: &str = "8d4009da5833318e2bd82af8c6f5";

    fn frame_bits(frame: &[u8]) -> Vec<u8> {
        (0..frame.len() * 8)
            .map(|i| u8::from(bit(frame, i)))
            .collect()
    }

    fn run(w: &[u8], skipbits: usize) -> Vec<u8> {
        let mut st = Stages::new();
        let mut out = vec![0u8; MAX_BITS];
        let m = st.run(w, skipbits, 1, false, &mut out);
        out.truncate(m);
        out
    }

    /// A DF11 burst starts with a 0 bit, whose first four samples are zero.
    /// Stage 1 drops them and the prefix puts them back, so the output is the
    /// frame, starting with bit 0.
    #[test]
    fn short_bursts_get_a_four_sample_prefix() {
        let f = hex("5d4ca2d6e1f0a8");
        let bits = frame_bits(&f);
        let w = burst(&f);
        assert_eq!(w.len(), 56);
        assert_eq!(run(&w, 0), bits);
        // Still a short burst at 100 bytes.
        let mut w100 = w.clone();
        w100.resize(100, 0);
        assert_eq!(run(&w100, 0)[..56], bits[..]);
        // At 101 bytes there is no prefix, and every window lands on the
        // second half of a bit: the output is the frame inverted.
        let mut w101 = w;
        w101.resize(101, 0);
        let inverted: Vec<u8> = bits.iter().map(|b| 1 - b).collect();
        assert_eq!(run(&w101, 0)[..56], inverted[..]);
    }

    /// A DF17 burst starts with a 1 bit and needs no prefix. Cut to 100
    /// bytes it gets one, and every window moves back half a bit.
    #[test]
    fn long_bursts_get_no_prefix() {
        let f = hex(EVEN);
        let bits = frame_bits(&f);
        let w = burst(&f);
        assert_eq!(w.len(), 112);
        assert_eq!(run(&w, 0), bits);
        let cut = run(&w[..100], 0);
        assert_eq!(cut.len(), 100);
        assert_eq!(cut[0], 0, "the prefix group");
        let inverted: Vec<u8> = bits[..99].iter().map(|b| 1 - b).collect();
        assert_eq!(cut[1..], inverted[..]);
    }

    #[test]
    fn margin_is_the_half_bit_energy_difference() {
        let f = hex(EVEN);
        let mut st = Stages::default();
        let mut out = vec![0u8; MAX_BITS];
        assert_eq!(st.run(&burst(&f), 0, 1, false, &mut out), 112);
        for i in 0..112 {
            assert_eq!(st.margin[i], if bit(&f, i) { 4 } else { -4 }, "bit {i}");
        }
    }

    #[test]
    fn output_is_limited_by_the_buffer() {
        let w = burst(&hex(EVEN));
        let st = Stages::new();
        let mut st2 = Stages::new();
        let mut a = [0u8; 10];
        let mut b = [0u8; 10];
        assert_eq!(st.run_ref(&w, 0, 1, true, &mut a), 10);
        assert_eq!(st2.run(&w, 0, 1, true, &mut b), 10);
        assert_eq!(a, b);
        assert_eq!(a[..], frame_bits(&hex(EVEN))[..10]);
    }

    /// Runs with the same first set bit reuse the group sums until the next
    /// burst is announced; after that a new burst with the same first set bit
    /// is demodulated afresh.
    #[test]
    fn memo_is_reused_within_a_burst_only() {
        let a = burst(&hex(EVEN));
        let b = burst(&hex("8d4009da583324fef1cbc5c7449d"));
        let mut st = Stages::new();
        let reference = |w: &[u8], gt: i32, bias: bool| {
            let mut o = vec![0u8; MAX_BITS];
            let m = Stages::new().run_ref(w, 0, gt, bias, &mut o);
            o.truncate(m);
            o
        };
        let mut out = vec![0u8; MAX_BITS];
        st.new_burst();
        for (gt, bias) in [(1, false), (2, false), (1, true), (2, true)] {
            let m = st.run(&a, 0, gt, bias, &mut out);
            assert_eq!(out[..m], reference(&a, gt, bias)[..], "gt={gt} bias={bias}");
        }
        st.new_burst();
        let m = st.run(&b, 0, 1, false, &mut out);
        assert_eq!(out[..m], reference(&b, 1, false)[..]);
        assert_eq!(
            out[..m],
            frame_bits(&hex("8d4009da583324fef1cbc5c7449d"))[..]
        );
    }

    /// A burst with no set bit after `skipbits` yields nothing but the prefix.
    #[test]
    fn silence_yields_no_bits() {
        assert!(run(&[0u8; 112], 0).is_empty());
        assert!(run(&[], 0).is_empty());
        // Four prefix samples make one group, and one group is no whole bit.
        assert!(run(&[0u8; 56], 0).is_empty());
        // A set bit before skipbits is skipped.
        assert!(run(&[0x80, 0, 0, 0], 1).is_empty());
        let st = Stages::new();
        assert_eq!(st.run_ref(&[0u8; 112], 0, 1, false, &mut [0u8; 8]), 0);
        assert_eq!(st.run_ref(&[0x80, 0, 0, 0], 1, 1, false, &mut [0u8; 8]), 0);
    }

    /// With noise the output length follows the first set bit: one bit per
    /// eight kept samples, rounded up to whole groups.
    #[test]
    fn output_length_follows_the_first_set_bit() {
        let mut w = noise(3, 112);
        w[0] = 0x01; // first set bit at 7
        w[1] |= 0x80; // and at 8
        assert_eq!(run(&w, 0).len(), (112usize * 8 - 7).div_ceil(4) / 2);
        assert_eq!(run(&w, 8).len(), (112usize * 8 - 8).div_ceil(4) / 2);
    }

    #[test]
    fn packbits_packs_most_significant_bit_first() {
        let mut fr = [0xAAu8; 14];
        let bits = frame_bits(&hex(EVEN));
        packbits(&bits, 56, &mut fr);
        assert_eq!(fr[..7], hex(EVEN)[..7]);
        assert_eq!(fr[7], 0xAA, "bytes past nbits are left alone");
        packbits(&bits, 112, &mut fr);
        assert_eq!(fr[..], hex(EVEN)[..]);
    }
}
