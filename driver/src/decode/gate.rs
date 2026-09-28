//! The plausibility gate: what a CRC match must also satisfy to be a frame.

use super::frame::{icao, is_valid_icao};

/// Reject frames that cannot be real. With several hundred candidate framings
/// per burst, the CRC alone is not a sufficient gate: a 2-bit search accepts
/// 6328 syndromes (112 single + 6216 pairs) and would false-accept noise most
/// of the time.
pub(super) fn plausible(fr: &[u8]) -> bool {
    if fr.iter().all(|&b| b == 0) {
        return false; // all-zero satisfies the CRC trivially
    }
    let df = fr[0] >> 3;
    if !matches!(df, 11 | 17 | 18) {
        return false;
    }
    if !is_valid_icao(icao(fr)) {
        return false;
    }
    if fr[1] >= 0xF0 {
        return false; // no ICAO block allocated there
    }
    if fr.len() == 14 {
        let tc = fr[4] >> 3;
        // Only the type codes the spec assigns; 23-27 and 30 are reserved
        // and admitting them only widens the window for noise.
        if !(tc <= 22 || tc == 28 || tc == 29 || tc == 31) {
            return false;
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::plausible;
    use crate::decode::synth::{df11, df17, hex, with_parity};

    #[test]
    fn real_formats_pass() {
        assert!(plausible(&hex("8d4009da5833318e2bd82af8c6f5")));
        assert!(plausible(&df11(0x4CA2D6)));
        assert!(plausible(&with_parity(&[
            0x90, 0x4C, 0xA2, 0xD6, 0x58, 0, 0, 0, 0, 0, 0
        ])));
    }

    #[test]
    fn all_zeros_fails() {
        assert!(!plausible(&[0; 7]));
        assert!(!plausible(&[0; 14]));
    }

    #[test]
    fn only_formats_11_17_and_18_pass() {
        for df in [0u8, 4, 5, 16, 19, 20, 21, 24, 31] {
            let mut f = df17(0x4CA2D6, [0x58, 0, 0, 0, 0, 0, 0]);
            f[0] = (df << 3) | 5;
            assert!(!plausible(&f), "DF{df}");
        }
    }

    #[test]
    fn addresses_no_aircraft_has_fail() {
        for a in [0, 0xFF_FFFF, 0xF0_0000, 0xFE_1234] {
            assert!(!plausible(&df11(a)), "{a:06X}");
        }
        assert!(plausible(&df11(0xEF_FFFF)));
    }

    /// Long frames need an assigned type code; short frames have none.
    #[test]
    fn reserved_type_codes_fail() {
        for tc in 0u8..32 {
            let f = df17(0x4CA2D6, [tc << 3, 0, 0, 0, 0, 0, 0]);
            let want = tc <= 22 || matches!(tc, 28 | 29 | 31);
            assert_eq!(plausible(&f), want, "type code {tc}");
        }
        // The gate does not check parity. Byte 4 of this short frame would
        // be type code 23 in a long one.
        let mut s = df11(0x4CA2D6);
        s[4] = 0xB8;
        assert!(plausible(&s));
    }
}
