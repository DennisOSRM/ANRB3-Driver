//! Mode-S CRC-24 and syndrome-based error correction.
//!
//! The CRC is linear, so the syndrome of a corrupted frame equals the CRC of
//! the error pattern alone. A table of single-bit syndromes therefore turns a
//! 1-bit correction into a lookup and a 2-bit correction into a pair search.

use crate::profile::fine_tick as tick;

/// Outcome of a soft-decision correction attempt. `searched` separates a
/// frame that was already clean from one the search worked on, so the caller
/// counts only real attempts.
pub struct SoftFix {
    pub bits: Option<u8>,
    pub searched: bool,
}

/// Sub-phase cycle counts inside the soft search.
#[derive(Default, Clone, Copy, Debug)]
pub struct SoftPhases {
    pub crc: u64,
    pub select: u64,
    pub single: u64,
    pub pair: u64,
    pub calls: u64,
}

/// Generator polynomial for the Mode-S 24-bit CRC.
const POLY: u32 = 0x00FF_F409;

/// The CRC table and the single-bit syndrome tables used for correction.
pub struct Crc {
    /// Sub-phase cycle counts inside the soft search; profiling builds only.
    #[cfg(feature = "profile")]
    pub soft_phases: std::cell::Cell<SoftPhases>,
    table: [u32; 256],
    /// Syndromes of a single bit error at each position, short frames.
    synd56: [u32; 56],
    /// Same for long frames.
    synd112: [u32; 112],
}

impl Crc {
    /// Build the byte table and the syndrome tables.
    pub fn new() -> Self {
        // Fold the eight per-bit shifts of a byte into one table entry.
        let mut table = [0u32; 256];
        for (i, e) in table.iter_mut().enumerate() {
            let mut rem = (i as u32) << 16;
            for _ in 0..8 {
                rem = if rem & 0x0080_0000 != 0 {
                    ((rem << 1) ^ POLY) & 0x00FF_FFFF
                } else {
                    (rem << 1) & 0x00FF_FFFF
                };
            }
            *e = rem;
        }
        let mut c = Crc {
            #[cfg(feature = "profile")]
            soft_phases: std::cell::Cell::new(SoftPhases::default()),
            table,
            synd56: [0; 56],
            synd112: [0; 112],
        };
        for i in 0..56 {
            let mut buf = [0u8; 7];
            buf[i >> 3] = 0x80 >> (i & 7);
            c.synd56[i] = c.crc24(&buf);
        }
        for i in 0..112 {
            let mut buf = [0u8; 14];
            buf[i >> 3] = 0x80 >> (i & 7);
            c.synd112[i] = c.crc24(&buf);
        }
        c
    }

    /// The 24-bit CRC of `m`. Zero for a frame whose parity field matches.
    pub fn crc24(&self, m: &[u8]) -> u32 {
        let mut rem = 0u32;
        for &b in m {
            rem = ((rem << 8) & 0x00FF_FFFF) ^ self.table[(((rem >> 16) as u8) ^ b) as usize];
        }
        rem
    }

    fn syndromes(&self, nbytes: usize) -> &[u32] {
        if nbytes == 14 {
            &self.synd112
        } else {
            &self.synd56
        }
    }

    /// Blind correction: try every single-bit flip, then every pair.
    /// Returns the number of bits corrected, or `None` if unfixable.
    pub fn fix(&self, fr: &mut [u8], allow2: bool) -> Option<u8> {
        let nbytes = fr.len();
        let nbits = nbytes * 8;
        let t = self.syndromes(nbytes);
        let s = self.crc24(fr);
        if s == 0 {
            return Some(0);
        }
        for i in 0..nbits {
            if t[i] == s {
                fr[i >> 3] ^= 0x80 >> (i & 7);
                return Some(1);
            }
        }
        if !allow2 {
            return None;
        }
        for i in 0..nbits {
            for j in i + 1..nbits {
                if t[i] ^ t[j] == s {
                    fr[i >> 3] ^= 0x80 >> (i & 7);
                    fr[j >> 3] ^= 0x80 >> (j & 7);
                    return Some(2);
                }
            }
        }
        None
    }

    /// Number of least-confident bit positions the soft search considers.
    const SOFT_K: usize = 14;

    /// Correction guided by per-bit confidence. Only the least-confident
    /// positions are candidates, so this searches ~91 pairs instead of the
    /// 6216 a blind search would try - both cheaper and far less likely to
    /// land on a coincidental CRC match.
    pub fn fix_soft(&self, fr: &mut [u8], margin: &[i8], maxbits: u8) -> SoftFix {
        // The sub-phase record is kept only in profiling builds. Otherwise it
        // is a local nothing reads, so the optimiser removes every update to
        // it - this runs 1.2 million times on a 15-minute corpus, and a Cell
        // get and set around each call is not free.
        #[cfg(feature = "profile")]
        {
            let mut ph = self.soft_phases.get();
            let r = self.soft_search(fr, margin, maxbits, &mut ph);
            self.soft_phases.set(ph);
            r
        }
        #[cfg(not(feature = "profile"))]
        {
            self.soft_search(fr, margin, maxbits, &mut SoftPhases::default())
        }
    }

    fn soft_search(
        &self,
        fr: &mut [u8],
        margin: &[i8],
        maxbits: u8,
        ph: &mut SoftPhases,
    ) -> SoftFix {
        let nbytes = fr.len();
        let nbits = nbytes * 8;
        let t = self.syndromes(nbytes);
        let t0 = tick();
        let s = self.crc24(fr);
        ph.crc += tick().wrapping_sub(t0);
        ph.calls += 1;
        if s == 0 {
            return SoftFix {
                bits: Some(0),
                searched: false,
            };
        }
        if maxbits < 1 {
            return SoftFix {
                bits: None,
                searched: false,
            };
        }
        let t_sel = tick();

        // |margin| is the difference between two four-sample half-bit sums, so
        // it can only be 0, 1, 2, 3 or 4. Five buckets therefore hold every
        // possible confidence level, and filling them yields both the
        // selection and its ascending order in one O(nbits) pass.
        // Never more than K are needed from any one bucket.
        let mut idx = [0usize; Self::SOFT_K];
        let mut bucket = [[0u8; Self::SOFT_K]; 5];
        let mut bn = [0usize; 5];
        // Scanned back to front deliberately. Both this and a forward scan
        // select the same multiset of confidences - the K least-confident bits
        // - but they keep different members when several bits tie, and keeping
        // the later ones recovers measurably more corrections. Demodulation
        // error grows toward the end of a burst as timing drifts away from the
        // preamble lock, so among equally unconfident bits the later ones are
        // the likelier to be wrong.
        for i in (0..nbits).rev() {
            let v = (margin.get(i).copied().unwrap_or(0).unsigned_abs() as usize).min(4);
            if bn[v] < Self::SOFT_K {
                bucket[v][bn[v]] = i as u8;
                bn[v] += 1;
            }
        }
        let mut nidx = 0usize;
        'fill: for v in 0..5 {
            for &i in &bucket[v][..bn[v]] {
                if nidx == Self::SOFT_K {
                    break 'fill;
                }
                idx[nidx] = i as usize;
                nidx += 1;
            }
        }
        ph.select += tick().wrapping_sub(t_sel);
        // Materialise the chosen syndromes. The search below is O(K^2) in the
        // pair case, and reading them through idx each time is a double
        // indirection plus a bounds check on every probe.
        let mut sy = [0u32; Self::SOFT_K];
        for k in 0..nidx {
            sy[k] = t[idx[k]];
        }
        let t3 = tick();
        for k in 0..nidx {
            if sy[k] == s {
                let i = idx[k];
                fr[i >> 3] ^= 0x80 >> (i & 7);
                ph.single += tick().wrapping_sub(t3);
                return SoftFix {
                    bits: Some(1),
                    searched: true,
                };
            }
        }
        ph.single += tick().wrapping_sub(t3);
        if maxbits < 2 {
            return SoftFix {
                bits: None,
                searched: true,
            };
        }
        let t4 = tick();
        for k in 0..nidx {
            let a = sy[k];
            for j in k + 1..nidx {
                if a ^ sy[j] == s {
                    let (a, b) = (idx[k], idx[j]);
                    fr[a >> 3] ^= 0x80 >> (a & 7);
                    fr[b >> 3] ^= 0x80 >> (b & 7);
                    ph.pair += tick().wrapping_sub(t4);
                    return SoftFix {
                        bits: Some(2),
                        searched: true,
                    };
                }
            }
        }
        ph.pair += tick().wrapping_sub(t4);
        SoftFix {
            bits: None,
            searched: true,
        }
    }

    /// The syndrome an address produces when XORed into the parity field.
    ///
    /// For DF 0/4/5/16/20/21 the AP field is the parity XOR the address, and
    /// for DF11 the PI field is the parity XOR the interrogator code. Because
    /// the parity occupies the last 24 bits of the division, the syndrome of
    /// such a frame is not the address but the CRC of the address taken as a
    /// three-byte message.
    pub fn ap_syndrome(&self, addr: u32) -> u32 {
        self.crc24(&[(addr >> 16) as u8, (addr >> 8) as u8, addr as u8])
    }
}

impl Default for Crc {
    fn default() -> Self {
        Self::new()
    }
}

/// Inverse of [`Crc::ap_syndrome`], built once by Gaussian elimination over
/// GF(2) on the 24 basis images.
pub struct ApMap {
    inv: [u32; 24],
}

impl ApMap {
    pub fn new(crc: &Crc) -> Self {
        let mut b = [0u32; 24];
        let mut i = [0u32; 24];
        for k in 0..24 {
            b[k] = crc.ap_syndrome(1 << k);
            i[k] = 1 << k;
        }
        let mut row = 0usize;
        for col in (0..24).rev() {
            let Some(piv) = (row..24).find(|&r| (b[r] >> col) & 1 == 1) else {
                continue;
            };
            b.swap(row, piv);
            i.swap(row, piv);
            for r in 0..24 {
                if r != row && (b[r] >> col) & 1 == 1 {
                    b[r] ^= b[row];
                    i[r] ^= i[row];
                }
            }
            row += 1;
        }
        let mut inv = [0u32; 24];
        for r in 0..24 {
            if let Some(col) = (0..24).find(|&c| (b[r] >> c) & 1 == 1) {
                inv[col] = i[r];
            }
        }
        ApMap { inv }
    }
    /// Recover the address from a frame's syndrome.
    pub fn address(&self, syndrome: u32) -> u32 {
        let mut a = 0u32;
        for k in 0..24 {
            if (syndrome >> k) & 1 == 1 {
                a ^= self.inv[k];
            }
        }
        a
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A real DF17 frame from the receiver; the CRC of a good frame is zero.
    fn good_frame() -> Vec<u8> {
        (0..14)
            .map(|i| {
                let s = "8d4009da5833318e2bd82af8c6f5";
                u8::from_str_radix(&s[i * 2..i * 2 + 2], 16).unwrap()
            })
            .collect()
    }

    #[test]
    fn clean_frame_has_zero_syndrome() {
        let c = Crc::new();
        assert_eq!(c.crc24(&good_frame()), 0);
    }

    #[test]
    fn single_bit_error_is_corrected() {
        let c = Crc::new();
        for bit in [0usize, 7, 33, 64, 111] {
            let mut f = good_frame();
            f[bit >> 3] ^= 0x80 >> (bit & 7);
            assert_eq!(c.fix(&mut f, false), Some(1), "bit {bit}");
            assert_eq!(f, good_frame(), "bit {bit} restored");
        }
    }

    #[test]
    fn two_bit_error_needs_blind_search() {
        let c = Crc::new();
        let mut f = good_frame();
        f[2] ^= 0x10;
        f[9] ^= 0x04;
        assert_eq!(
            c.fix(&mut f.clone(), false),
            None,
            "1-bit search must not claim it"
        );
        assert_eq!(c.fix(&mut f, true), Some(2));
        assert_eq!(f, good_frame());
    }

    #[test]
    fn soft_search_uses_only_low_confidence_bits() {
        let c = Crc::new();
        let mut f = good_frame();
        f[1] ^= 0x20;
        // Every bit confident except the corrupted one: it must be found.
        let mut margin = [4i8; 112];
        margin[8 + 2] = 0;
        let r = c.fix_soft(&mut f, &margin, 2);
        assert_eq!(r.bits, Some(1));
        assert_eq!(f, good_frame());

        // Same error, but that bit looks certain and 14 others do not, so
        // the search never considers it.
        let mut g = good_frame();
        g[1] ^= 0x20;
        let mut m2 = [4i8; 112];
        m2[60..80].fill(0);
        assert_eq!(c.fix_soft(&mut g, &m2, 2).bits, None);
    }

    /// A clean frame is left alone, and says it was not searched.
    #[test]
    fn a_clean_frame_needs_no_fix() {
        let c = Crc::default();
        let mut f = good_frame();
        assert_eq!(c.fix(&mut f, true), Some(0));
        let r = c.fix_soft(&mut f, &[0; 112], 2);
        assert_eq!((r.bits, r.searched), (Some(0), false));
        assert_eq!(f, good_frame());
    }

    /// Three wrong bits are more than either search repairs, and the frame
    /// is left as it was.
    #[test]
    fn three_bit_errors_are_not_fixed() {
        let c = Crc::new();
        let mut f = good_frame();
        f[0] ^= 0x01;
        f[5] ^= 0x10;
        f[12] ^= 0x80;
        let bad = f.clone();
        assert_eq!(c.fix(&mut f, true), None);
        assert_eq!(f, bad);
        let r = c.fix_soft(&mut f, &[0; 112], 2);
        assert_eq!((r.bits, r.searched), (None, true));
        assert_eq!(f, bad);
    }

    /// Two wrong bits among the least confident are repaired by the soft
    /// search, but only when it is allowed two; with none allowed it does not
    /// search at all.
    #[test]
    fn soft_search_repairs_a_pair() {
        let c = Crc::new();
        let mut f = good_frame();
        f[3] ^= 0x02;
        f[10] ^= 0x40;
        let mut margin = [4i8; 112];
        margin[3 * 8 + 6] = 1;
        margin[10 * 8 + 1] = -2;
        margin[50] = 0;

        let bad = f.clone();
        let r = c.fix_soft(&mut f, &margin, 0);
        assert_eq!((r.bits, r.searched), (None, false));
        let r = c.fix_soft(&mut f, &margin, 1);
        assert_eq!((r.bits, r.searched), (None, true));
        assert_eq!(f, bad, "a failed search changes nothing");
        let r = c.fix_soft(&mut f, &margin, 2);
        assert_eq!((r.bits, r.searched), (Some(2), true));
        assert_eq!(f, good_frame());
    }

    /// Short frames use their own syndrome table.
    #[test]
    fn a_short_frame_is_corrected() {
        let c = Crc::new();
        let mut good = vec![0x5d, 0x3c, 0x65, 0x51, 0, 0, 0];
        let p = c.crc24(&good[..4]);
        good[4..].copy_from_slice(&[(p >> 16) as u8, (p >> 8) as u8, p as u8]);
        assert_eq!(c.crc24(&good), 0);
        let mut f = good.clone();
        f[2] ^= 0x08;
        assert_eq!(c.fix(&mut f, false), Some(1));
        assert_eq!(f, good);
        f[1] ^= 0x01;
        f[6] ^= 0x20;
        let mut margin = [4i8; 56];
        margin[15] = 0;
        margin[50] = 0;
        assert_eq!(c.fix_soft(&mut f, &margin, 2).bits, Some(2));
        assert_eq!(f, good);
    }

    /// The address map undoes the address syndrome.
    #[test]
    fn the_address_comes_back_from_its_syndrome() {
        let c = Crc::new();
        let ap = ApMap::new(&c);
        for a in [0, 1, 0x4009DA, 0x3C6551, 0xFFFFFF, 0x800000] {
            assert_eq!(ap.address(c.ap_syndrome(a)), a, "{a:06X}");
        }
    }
}
