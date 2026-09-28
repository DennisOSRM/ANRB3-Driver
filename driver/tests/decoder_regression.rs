//! Regression checks of the decoder against captures/bursts.bin, and of the
//! fast demodulator against the reference one. Each test returns early, and
//! passes, when its capture file is absent. The checks in decoder.rs run on
//! generated bursts and need no capture.

use anrb::corpus::{self, Fingerprint};
use anrb::Decoder;

fn bursts_bin() -> Option<Vec<u8>> {
    std::fs::read("../captures/bursts.bin").ok()
}

/// The fingerprint of the frames decoded from captures/bursts.bin. A change
/// here means the decoder's output changed; update it only after validating
/// the new output.
const EXPECTED_FINGERPRINT: u64 = 0x296b_7602_37e2_81c6;
/// The number of frames decoded from captures/bursts.bin.
const EXPECTED_FRAMES: usize = 18;

#[test]
fn fingerprint_is_unchanged() {
    let Some(d) = bursts_bin() else {
        eprintln!("corpus absent, skipping");
        return;
    };
    let mut dec = Decoder::new();
    let mut fp = Fingerprint::new();
    for b in corpus::burst_file(&d) {
        if let Some(f) = dec.decode_burst(b, 0) {
            fp.add(&f);
        }
    }
    assert_eq!(fp.frames, EXPECTED_FRAMES, "frame count");
    assert_eq!(
        format!("{:016x}", fp.hash),
        format!("{EXPECTED_FINGERPRINT:016x}"),
        "fingerprint"
    );
}

#[test]
fn fast_demodulator_matches_reference() {
    let Some(d) = bursts_bin() else {
        eprintln!("corpus absent, skipping");
        return;
    };
    let mut dec = Decoder::new();
    let mut checked = 0usize;
    for b in corpus::burst_file(&d) {
        for skipbits in 0..9usize {
            for gt in 1..=2i32 {
                for bias in [false, true] {
                    dec.verify_stages(b, skipbits, gt, bias)
                        .unwrap_or_else(|e| panic!("skip={skipbits} gt={gt} bias={bias}: {e}"));
                    checked += 1;
                }
            }
        }
    }
    assert!(checked > 1000, "expected a real workload, got {checked}");
}
