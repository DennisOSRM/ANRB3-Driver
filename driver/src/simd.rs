//! Vectorised inner loops for the demodulator, with a scalar fallback.
//!
//! Two loops dominate `Stages::run`:
//!
//! 1. **Group sums.** Stage 2 sums groups of four samples. Once the stream is
//!    byte-aligned those sums are the popcounts of its nibbles, so a 16-entry
//!    lookup table applied to all 32 nibbles of a register computes 32 groups
//!    at a time. `pshufb` on x86_64 and `tbl` on aarch64 do exactly that.
//!
//! 2. **Threshold and decimate.** Without hysteresis each output bit is an
//!    independent comparison and only even-indexed groups survive, so 16
//!    outputs come from one gather plus one compare.
//!
//! Every path here must produce bit-identical results to the scalar code; the
//! unit test below and the fingerprint test in tests/decoder_regression.rs
//! check that.

/// Popcount of each nibble value, the table both architectures look up.
const NIBPOP: [u8; 16] = [0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4];

/// Which implementation the inner loops will use, decided once at startup.
///
/// Public so a caller can report it. The dispatch functions that act on it are
/// crate-private on purpose: calling a vector kernel is only sound on a CPU
/// that has the instructions, and inside the crate a vector backend only ever
/// comes from [`Backend::detect`], which checks.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Backend {
    /// x86_64 with SSSE3 (`pshufb`).
    Ssse3,
    /// aarch64 with NEON (`tbl`, `uzp1`).
    Neon,
    /// Portable code, used everywhere else and on x86_64 without SSSE3.
    Scalar,
}

impl Backend {
    /// The fastest backend this CPU supports. Setting `ANRB_BACKEND=scalar`
    /// forces the portable code; any other value is reported on stderr and
    /// ignored.
    pub fn detect() -> Backend {
        Backend::choose(std::env::var("ANRB_BACKEND").ok().as_deref())
    }

    /// [`Backend::detect`], given what ANRB_BACKEND says, if anything.
    fn choose(setting: Option<&str>) -> Backend {
        // Allow the fallback to be forced, so the portable path stays
        // exercisable on machines that would otherwise never run it.
        match setting {
            Some("scalar") => return Backend::Scalar,
            Some(other) if !other.is_empty() => {
                eprintln!("ANRB_BACKEND={other} not recognised; using autodetect");
            }
            _ => {}
        }
        #[cfg(target_arch = "x86_64")]
        {
            if std::arch::is_x86_feature_detected!("ssse3") {
                return Backend::Ssse3;
            }
        }
        // NEON is part of the aarch64 baseline, so no runtime check is needed.
        if cfg!(target_arch = "aarch64") {
            Backend::Neon
        } else {
            Backend::Scalar
        }
    }

    /// A name for reports, such as "x86_64 SSSE3".
    pub fn name(self) -> &'static str {
        match self {
            Backend::Ssse3 => "x86_64 SSSE3",
            Backend::Neon => "aarch64 NEON",
            Backend::Scalar => "scalar",
        }
    }
}

/// Fill `accb[g0..ng]` with the nibble popcounts of `shbuf`.
///
/// Group `2i` is the high nibble of byte `i` and group `2i+1` the low nibble,
/// so the two lookups have to be interleaved back into group order.
pub(crate) fn nibble_groups(be: Backend, shbuf: &[u8], accb: &mut [u8], g0: usize, ng: usize) {
    let ngroups = ng.saturating_sub(g0);
    let done = match be {
        #[cfg(target_arch = "x86_64")]
        // SAFETY: Ssse3 is only ever chosen by Backend::detect, after
        // establishing that this CPU has SSSE3.
        Backend::Ssse3 => unsafe { x86::nibble_groups(shbuf, accb, g0, ngroups) },
        #[cfg(target_arch = "aarch64")]
        // SAFETY: Neon is only ever chosen by Backend::detect, after
        // establishing that this CPU has NEON.
        Backend::Neon => unsafe { arm::nibble_groups(shbuf, accb, g0, ngroups) },
        _ => 0,
    };
    // Tail, and the whole job when no vector path applies.
    for (k, a) in accb[..ng].iter_mut().enumerate().skip(g0 + done) {
        let bo = (k - g0) * 4;
        let b2 = bo >> 3;
        let byte = shbuf.get(b2).copied().unwrap_or(0);
        *a = if bo & 4 != 0 {
            NIBPOP[(byte & 0x0F) as usize]
        } else {
            NIBPOP[(byte >> 4) as usize]
        };
    }
}

/// `margin[k] = accb[2k] - accb[2k+1]` for `k` in `0..m`: the PPM soft metric,
/// the energy in the first half of a bit minus the second.
///
/// This is a deinterleave followed by a subtract. x86 gathers the even and odd
/// bytes with two shuffles each; aarch64 has `ld2`, which deinterleaves as it
/// loads. Group sums are 0..4 so the difference always fits a signed byte.
pub(crate) fn margin(be: Backend, accb: &[u8], margin: &mut [i8], m: usize, ng: usize) {
    let done = match be {
        #[cfg(target_arch = "x86_64")]
        // SAFETY: Ssse3 is only ever chosen by Backend::detect, after
        // establishing that this CPU has SSSE3.
        Backend::Ssse3 => unsafe { x86::margin(accb, margin, m, ng) },
        #[cfg(target_arch = "aarch64")]
        // SAFETY: Neon is only ever chosen by Backend::detect, after
        // establishing that this CPU has NEON.
        Backend::Neon => unsafe { arm::margin(accb, margin, m, ng) },
        _ => 0,
    };
    for k in done..m {
        let e = if 2 * k < ng { accb[2 * k] as i32 } else { 0 };
        let o = if 2 * k + 1 < ng {
            accb[2 * k + 1] as i32
        } else {
            0
        };
        margin[k] = (e - o) as i8;
    }
}

/// `out[k] = (accb[2k] > gt) as u8` for `k` in `0..m`, the no-hysteresis path.
pub(crate) fn threshold_even(
    be: Backend,
    accb: &[u8],
    out: &mut [u8],
    m: usize,
    ng: usize,
    gt: i32,
) {
    let done = match be {
        #[cfg(target_arch = "x86_64")]
        // SAFETY: Ssse3 is only ever chosen by Backend::detect, after
        // establishing that this CPU has SSSE3.
        Backend::Ssse3 => unsafe { x86::threshold_even(accb, out, m, ng, gt as i8) },
        #[cfg(target_arch = "aarch64")]
        // SAFETY: Neon is only ever chosen by Backend::detect, after
        // establishing that this CPU has NEON.
        Backend::Neon => unsafe { arm::threshold_even(accb, out, m, ng, gt as i8) },
        _ => 0,
    };
    for k in done..m {
        out[k] = u8::from(accb[2 * k] as i32 > gt);
    }
}

/// The hysteresis threshold, as a prefix scan rather than a recurrence.
///
/// `v[k] = (accb[k] + (v[k-1] ? -1 : +1)) > gt` is, writing `A[k] = accb[k] >=
/// gt` and `B[k] = accb[k] >= gt+2`, exactly `v[k] = v[k-1] ? B[k] : A[k]`.
/// Only even `k` survive, so consecutive steps compose into one function of a
/// single bit, described by what it returns for 0 and for 1:
///
/// ```text
/// p[b] = A[2b-1] ? B[2b] : A[2b]
/// q[b] = B[2b-1] ? B[2b] : A[2b]
/// ```
///
/// Those depend on no earlier output, so they are computed 16 at a time and
/// packed into bitmasks. Composition of two such functions is associative -
/// `(g after f)` has `p = p_f ? q_g : p_g` and `q = q_f ? q_g : p_g` - so a
/// Hillis-Steele scan over 64 steps at once replaces the dependency chain,
/// which measurement showed to be the real cost rather than the step count.
pub(crate) fn threshold_hyst(
    be: Backend,
    accb: &[u8],
    out: &mut [u8],
    m: usize,
    ng: usize,
    gt: i32,
) {
    if m == 0 {
        return;
    }
    out[0] = u8::from(accb[0] as i32 > gt);
    match be {
        #[cfg(target_arch = "x86_64")]
        // SAFETY: Ssse3 is only ever chosen by Backend::detect, after
        // establishing that this CPU has SSSE3.
        Backend::Ssse3 => unsafe { x86::hyst(accb, out, m, ng, gt) },
        #[cfg(target_arch = "aarch64")]
        // SAFETY: Neon is only ever chosen by Backend::detect, after
        // establishing that this CPU has NEON.
        Backend::Neon => unsafe { arm::hyst(accb, out, m, ng, gt) },
        _ => hyst_scalar(accb, out, m, gt),
    }
}

/// One step's function pair, computed without reference to any earlier output.
#[inline(always)]
fn pair_at(accb: &[u8], k: usize, gt: i32) -> (bool, bool) {
    let odd = accb[k - 1] as i32;
    let even = accb[k] as i32;
    let ae = even >= gt;
    let be = even >= gt + 2;
    (
        if odd >= gt { be } else { ae },
        if odd >= gt + 2 { be } else { ae },
    )
}

/// Resolve 64 composed steps at once and write them out. Shared by every
/// backend: the scan is plain 64-bit arithmetic, only the generation differs.
#[inline(always)]
fn hyst_scan(mut p: u64, mut q: u64, state: bool, n: usize, out: &mut [u8]) -> bool {
    let mut d = 1usize;
    while d < 64 {
        let fp = p << d;
        let fq = (q << d) | ((1u64 << d) - 1);
        let np = (fp & q) | (!fp & p);
        let nq = (fq & q) | (!fq & p);
        p = np;
        q = nq;
        d <<= 1;
    }
    let res = if state { q } else { p };
    for (j, o) in out[..n].iter_mut().enumerate() {
        *o = ((res >> j) & 1) as u8;
    }
    (res >> (n - 1)) & 1 != 0
}

fn hyst_scalar(accb: &[u8], out: &mut [u8], m: usize, gt: i32) {
    let mut state = out[0] != 0;
    let mut b = 1usize;
    while b < m {
        let n = (m - b).min(64);
        let (mut p, mut q) = (0u64, 0u64);
        for j in 0..n {
            let (pj, qj) = pair_at(accb, 2 * (b + j), gt);
            p |= (pj as u64) << j;
            q |= (qj as u64) << j;
        }
        state = hyst_scan(p, q, state, n, &mut out[b..]);
        b += n;
    }
}

#[cfg(target_arch = "x86_64")]
mod x86 {
    //! Safe functions: each load and store goes through a bounds-checked
    //! sub-slice of exactly the width it touches, so no argument can make them
    //! read or write outside a buffer. What stays `unsafe` is the unaligned
    //! access itself, which SSE permits on any address.
    use super::NIBPOP;
    use std::arch::x86_64::*;

    #[inline(always)]
    fn load(b: &[u8]) -> __m128i {
        let b: &[u8; 16] = b.try_into().expect("a 16-byte window");
        // SAFETY: b is exactly 16 readable bytes; loadu has no alignment rule.
        unsafe { _mm_loadu_si128(b.as_ptr().cast()) }
    }
    #[inline(always)]
    fn store<T>(b: &mut [T], v: __m128i) {
        assert_eq!(std::mem::size_of_val(b), 16, "a 16-byte window");
        // SAFETY: b is exactly 16 writable bytes; storeu has no alignment rule.
        unsafe { _mm_storeu_si128(b.as_mut_ptr().cast(), v) }
    }
    /// Shuffle control that gathers the even bytes of a register into its low
    /// half and zeroes the high half.
    #[target_feature(enable = "ssse3")]
    fn even_mask() -> __m128i {
        _mm_setr_epi8(
            0, 2, 4, 6, 8, 10, 12, 14, -128, -128, -128, -128, -128, -128, -128, -128,
        )
    }

    #[target_feature(enable = "ssse3")]
    pub fn nibble_groups(shbuf: &[u8], accb: &mut [u8], g0: usize, ngroups: usize) -> usize {
        let lut = _mm_setr_epi8(
            NIBPOP[0] as i8,
            NIBPOP[1] as i8,
            NIBPOP[2] as i8,
            NIBPOP[3] as i8,
            NIBPOP[4] as i8,
            NIBPOP[5] as i8,
            NIBPOP[6] as i8,
            NIBPOP[7] as i8,
            NIBPOP[8] as i8,
            NIBPOP[9] as i8,
            NIBPOP[10] as i8,
            NIBPOP[11] as i8,
            NIBPOP[12] as i8,
            NIBPOP[13] as i8,
            NIBPOP[14] as i8,
            NIBPOP[15] as i8,
        );
        let m0f = _mm_set1_epi8(0x0f);
        let mut gi = 0usize;
        let mut by = 0usize;
        while gi + 32 <= ngroups && by + 16 <= shbuf.len() && g0 + gi + 32 <= accb.len() {
            let v = load(&shbuf[by..by + 16]);
            let lo = _mm_and_si128(v, m0f);
            let hi = _mm_and_si128(_mm_srli_epi16(v, 4), m0f);
            let clo = _mm_shuffle_epi8(lut, lo);
            let chi = _mm_shuffle_epi8(lut, hi);
            // High nibble is the even group, low nibble the odd one.
            let at = g0 + gi;
            store(&mut accb[at..at + 16], _mm_unpacklo_epi8(chi, clo));
            store(&mut accb[at + 16..at + 32], _mm_unpackhi_epi8(chi, clo));
            gi += 32;
            by += 16;
        }
        gi
    }

    /// The hysteresis scan, driven from inside the feature boundary so the
    /// 16-at-a-time pair generation inlines. Called out of line it costs a
    /// function call per 16 outputs, a significant share of the runtime.
    /// Each backend has its own copy of this loop for that reason.
    #[target_feature(enable = "ssse3")]
    pub fn hyst(accb: &[u8], out: &mut [u8], m: usize, ng: usize, gt: i32) {
        use super::{hyst_scan, pair_at};
        let mut state = out[0] != 0;
        let mut b = 1usize;
        while b < m {
            let n = (m - b).min(64);
            let (mut p, mut q) = (0u64, 0u64);
            let mut i = 0usize;
            while i + 16 <= n && 2 * (b + i) + 32 <= ng.min(accb.len()) {
                let (pm, qm) = hyst_pairs(accb, 2 * (b + i), gt);
                p |= (pm as u64) << i;
                q |= (qm as u64) << i;
                i += 16;
            }
            for j in i..n {
                let (pj, qj) = pair_at(accb, 2 * (b + j), gt);
                p |= (pj as u64) << j;
                q |= (qj as u64) << j;
            }
            state = hyst_scan(p, q, state, n, &mut out[b..]);
            b += n;
        }
    }

    /// 16 function pairs from `accb[k]`, `accb[k-1]` ... `accb[k+30]`, as bitmasks.
    /// No inline(always) - it cannot be combined with target_feature - but
    /// LLVM inlines it into `hyst` freely, both having the same feature set.
    #[target_feature(enable = "ssse3")]
    fn hyst_pairs(accb: &[u8], k: usize, gt: i32) -> (u16, u16) {
        let ev = even_mask();
        let w = &accb[k - 1..k + 32]; // everything read below
        let gather = |base: &[u8]| -> __m128i {
            _mm_unpacklo_epi64(
                _mm_shuffle_epi8(load(&base[..16]), ev),
                _mm_shuffle_epi8(load(&base[16..32]), ev),
            )
        };
        let e = gather(&w[1..]); // accb[k], k+2, ...
        let o = gather(&w[..32]); // accb[k-1], k+1, ...
                                  // Values are 0..4 and gt is 1..2, so a signed byte compare is exact.
        let lo = _mm_set1_epi8((gt - 1) as i8);
        let hi = _mm_set1_epi8((gt + 1) as i8);
        let ae = _mm_cmpgt_epi8(e, lo);
        let be = _mm_cmpgt_epi8(e, hi);
        let ao = _mm_cmpgt_epi8(o, lo);
        let bo = _mm_cmpgt_epi8(o, hi);
        // blendv is SSE4.1, so select with and/andnot/or.
        let p = _mm_or_si128(_mm_andnot_si128(ao, ae), _mm_and_si128(ao, be));
        let q = _mm_or_si128(_mm_andnot_si128(bo, ae), _mm_and_si128(bo, be));
        (_mm_movemask_epi8(p) as u16, _mm_movemask_epi8(q) as u16)
    }

    #[target_feature(enable = "ssse3")]
    pub fn margin(accb: &[u8], marg: &mut [i8], m: usize, ng: usize) -> usize {
        let ev = even_mask();
        let od = _mm_setr_epi8(
            1, 3, 5, 7, 9, 11, 13, 15, -128, -128, -128, -128, -128, -128, -128, -128,
        );
        let mut k = 0usize;
        while k + 16 <= m.min(marg.len()) && 2 * k + 32 <= ng.min(accb.len()) {
            let a = load(&accb[2 * k..2 * k + 16]);
            let b = load(&accb[2 * k + 16..2 * k + 32]);
            let e = _mm_unpacklo_epi64(_mm_shuffle_epi8(a, ev), _mm_shuffle_epi8(b, ev));
            let o = _mm_unpacklo_epi64(_mm_shuffle_epi8(a, od), _mm_shuffle_epi8(b, od));
            store(&mut marg[k..k + 16], _mm_sub_epi8(e, o));
            k += 16;
        }
        k
    }

    #[target_feature(enable = "ssse3")]
    pub fn threshold_even(accb: &[u8], out: &mut [u8], m: usize, ng: usize, gt: i8) -> usize {
        // The high half of each gather is discarded by the 64-bit unpack.
        let ev = even_mask();
        let gv = _mm_set1_epi8(gt);
        let one = _mm_set1_epi8(1);
        let mut k = 0usize;
        while k + 16 <= m.min(out.len()) && 2 * k + 32 <= ng.min(accb.len()) {
            let a = load(&accb[2 * k..2 * k + 16]);
            let b = load(&accb[2 * k + 16..2 * k + 32]);
            let e = _mm_unpacklo_epi64(_mm_shuffle_epi8(a, ev), _mm_shuffle_epi8(b, ev));
            store(
                &mut out[k..k + 16],
                _mm_and_si128(_mm_cmpgt_epi8(e, gv), one),
            );
            k += 16;
        }
        k
    }
}

#[cfg(target_arch = "aarch64")]
mod arm {
    //! Safe functions, on the same terms as the x86 ones: every load and store
    //! goes through a bounds-checked window of exactly the width it touches.
    use super::NIBPOP;
    use std::arch::aarch64::*;

    #[inline(always)]
    fn ld1(b: &[u8]) -> uint8x16_t {
        let b: &[u8; 16] = b.try_into().expect("a 16-byte window");
        // SAFETY: b is exactly 16 readable bytes.
        unsafe { vld1q_u8(b.as_ptr()) }
    }
    #[inline(always)]
    fn ld2(b: &[u8]) -> uint8x16x2_t {
        let b: &[u8; 32] = b.try_into().expect("a 32-byte window");
        // SAFETY: b is exactly 32 readable bytes.
        unsafe { vld2q_u8(b.as_ptr()) }
    }
    #[inline(always)]
    fn st1(b: &mut [u8], v: uint8x16_t) {
        let b: &mut [u8; 16] = b.try_into().expect("a 16-byte window");
        // SAFETY: b is exactly 16 writable bytes.
        unsafe { vst1q_u8(b.as_mut_ptr(), v) }
    }

    #[target_feature(enable = "neon")]
    pub fn nibble_groups(shbuf: &[u8], accb: &mut [u8], g0: usize, ngroups: usize) -> usize {
        let lut = ld1(&NIBPOP);
        let m0f = vdupq_n_u8(0x0f);
        let mut gi = 0usize;
        let mut by = 0usize;
        while gi + 32 <= ngroups && by + 16 <= shbuf.len() && g0 + gi + 32 <= accb.len() {
            let v = ld1(&shbuf[by..by + 16]);
            let lo = vandq_u8(v, m0f);
            let hi = vshrq_n_u8(v, 4);
            // tbl is the direct counterpart of pshufb for a 16-byte table.
            let clo = vqtbl1q_u8(lut, lo);
            let chi = vqtbl1q_u8(lut, hi);
            // zip1/zip2 interleave exactly as punpcklbw/punpckhbw do.
            let at = g0 + gi;
            st1(&mut accb[at..at + 16], vzip1q_u8(chi, clo));
            st1(&mut accb[at + 16..at + 32], vzip2q_u8(chi, clo));
            gi += 32;
            by += 16;
        }
        gi
    }

    /// The hysteresis scan, driven inside the feature boundary so the pair
    /// generation inlines. See the x86 counterpart.
    #[target_feature(enable = "neon")]
    pub fn hyst(accb: &[u8], out: &mut [u8], m: usize, ng: usize, gt: i32) {
        use super::{hyst_scan, pair_at};
        let mut state = out[0] != 0;
        let mut b = 1usize;
        while b < m {
            let n = (m - b).min(64);
            let (mut p, mut q) = (0u64, 0u64);
            let mut i = 0usize;
            while i + 16 <= n && 2 * (b + i) + 32 <= ng.min(accb.len()) {
                let (pm, qm) = hyst_pairs(accb, 2 * (b + i), gt);
                p |= (pm as u64) << i;
                q |= (qm as u64) << i;
                i += 16;
            }
            for j in i..n {
                let (pj, qj) = pair_at(accb, 2 * (b + j), gt);
                p |= (pj as u64) << j;
                q |= (qj as u64) << j;
            }
            state = hyst_scan(p, q, state, n, &mut out[b..]);
            b += n;
        }
    }

    /// 16 function pairs, as bitmasks. NEON has no movemask, so the byte
    /// masks are weighted by bit position and summed per half.
    #[target_feature(enable = "neon")]
    fn hyst_pairs(accb: &[u8], k: usize, gt: i32) -> (u16, u16) {
        let w = &accb[k - 1..k + 32]; // everything read below
        let e = ld2(&w[1..33]).0; // accb[k], k+2, ...
        let o = ld2(&w[..32]).0; // accb[k-1], k+1, ...
        let lo = vdupq_n_s8((gt - 1) as i8);
        let hi = vdupq_n_s8((gt + 1) as i8);
        let es = vreinterpretq_s8_u8(e);
        let os = vreinterpretq_s8_u8(o);
        let ae = vcgtq_s8(es, lo);
        let be = vcgtq_s8(es, hi);
        let ao = vcgtq_s8(os, lo);
        let bo = vcgtq_s8(os, hi);
        let p = vbslq_u8(ao, be, ae);
        let q = vbslq_u8(bo, be, ae);
        const PW: [u8; 16] = [1, 2, 4, 8, 16, 32, 64, 128, 1, 2, 4, 8, 16, 32, 64, 128];
        let w = ld1(&PW);
        let pack = |v: uint8x16_t| -> u16 {
            let t = vandq_u8(v, w);
            (vaddv_u8(vget_low_u8(t)) as u16) | ((vaddv_u8(vget_high_u8(t)) as u16) << 8)
        };
        (pack(p), pack(q))
    }

    #[target_feature(enable = "neon")]
    pub fn margin(accb: &[u8], marg: &mut [i8], m: usize, ng: usize) -> usize {
        let mut k = 0usize;
        while k + 16 <= m.min(marg.len()) && 2 * k + 32 <= ng.min(accb.len()) {
            // ld2 deinterleaves on load: .0 gets the even bytes, .1 the odd.
            let p = ld2(&accb[2 * k..2 * k + 32]);
            let d = vsubq_s8(vreinterpretq_s8_u8(p.0), vreinterpretq_s8_u8(p.1));
            let dst: &mut [i8; 16] = (&mut marg[k..k + 16]).try_into().expect("16 bytes");
            // SAFETY: dst is exactly 16 writable bytes.
            unsafe { vst1q_s8(dst.as_mut_ptr(), d) }
            k += 16;
        }
        k
    }

    #[target_feature(enable = "neon")]
    pub fn threshold_even(accb: &[u8], out: &mut [u8], m: usize, ng: usize, gt: i8) -> usize {
        let gv = vdupq_n_s8(gt);
        let one = vdupq_n_u8(1);
        let mut k = 0usize;
        while k + 16 <= m.min(out.len()) && 2 * k + 32 <= ng.min(accb.len()) {
            let a = vreinterpretq_s8_u8(ld1(&accb[2 * k..2 * k + 16]));
            let b = vreinterpretq_s8_u8(ld1(&accb[2 * k + 16..2 * k + 32]));
            // uzp1 takes the even lanes of both registers in one instruction,
            // which is the whole gather the x86 path needs two shuffles for.
            let e = vuzp1q_s8(a, b);
            st1(&mut out[k..k + 16], vandq_u8(vcgtq_s8(e, gv), one));
            k += 16;
        }
        k
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Whatever backend this machine picked must agree with the scalar code.
    #[test]
    fn backend_matches_scalar() {
        let be = Backend::detect();
        let mut shbuf = [0u8; 2048];
        // Deterministic but structurally varied input.
        let mut x: u32 = 0x1234_5678;
        for b in shbuf.iter_mut() {
            x = x.wrapping_mul(1664525).wrapping_add(1013904223);
            *b = (x >> 16) as u8;
        }
        for &(g0, ng) in &[
            (0usize, 4096usize),
            (1, 4096),
            (1, 137),
            (0, 33),
            (1, 32),
            (0, 1),
        ] {
            let mut a = vec![0u8; 4096];
            let mut b = vec![0u8; 4096];
            nibble_groups(be, &shbuf, &mut a, g0, ng);
            nibble_groups(Backend::Scalar, &shbuf, &mut b, g0, ng);
            assert_eq!(a, b, "nibble_groups g0={g0} ng={ng} on {}", be.name());
        }
        // Group sums are 0..4, the range the real accumulator holds.
        let mut accb = vec![0u8; 4096];
        for (i, v) in accb.iter_mut().enumerate() {
            x = x.wrapping_mul(1664525).wrapping_add(1013904223);
            *v = ((x >> 16) % 5) as u8;
            if i % 97 == 0 {
                *v = 4;
            }
        }
        for &ng in &[4096usize, 200, 33, 32, 3] {
            let m = ng / 2;
            let mut a = vec![0i8; 4096];
            let mut b = vec![0i8; 4096];
            margin(be, &accb, &mut a, m, ng);
            margin(Backend::Scalar, &accb, &mut b, m, ng);
            assert_eq!(a, b, "margin ng={ng} on {}", be.name());
        }
        for &gt in &[1i32, 2] {
            for &ng in &[4096usize, 400, 200, 137, 33, 32, 4] {
                let m = ng / 2;
                let mut a = vec![0u8; 4096];
                let mut b = vec![0u8; 4096];
                threshold_hyst(be, &accb, &mut a, m, ng, gt);
                threshold_hyst(Backend::Scalar, &accb, &mut b, m, ng, gt);
                assert_eq!(
                    a[..m],
                    b[..m],
                    "threshold_hyst gt={gt} ng={ng} on {}",
                    be.name()
                );
            }
            for &ng in &[4096usize, 200, 33, 32, 3] {
                let m = ng / 2;
                let mut a = vec![0u8; 4096];
                let mut b = vec![0u8; 4096];
                threshold_even(be, &accb, &mut a, m, ng, gt);
                threshold_even(Backend::Scalar, &accb, &mut b, m, ng, gt);
                assert_eq!(a, b, "threshold_even gt={gt} ng={ng} on {}", be.name());
            }
        }
    }

    /// ANRB_BACKEND=scalar forces the portable code; an empty or unknown
    /// value is ignored.
    #[test]
    fn the_backend_can_be_forced_to_scalar() {
        let auto = Backend::choose(None);
        assert_eq!(Backend::choose(Some("scalar")), Backend::Scalar);
        assert_eq!(Backend::choose(Some("")), auto);
        assert_eq!(Backend::choose(Some("avx512")), auto, "an unknown value");
        #[cfg(target_arch = "x86_64")]
        assert_eq!(
            auto == Backend::Ssse3,
            std::arch::is_x86_feature_detected!("ssse3")
        );
        #[cfg(target_arch = "aarch64")]
        assert_eq!(auto, Backend::Neon);
    }

    #[test]
    fn every_backend_has_a_name() {
        assert_eq!(Backend::Ssse3.name(), "x86_64 SSSE3");
        assert_eq!(Backend::Neon.name(), "aarch64 NEON");
        assert_eq!(Backend::Scalar.name(), "scalar");
    }
}
