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
