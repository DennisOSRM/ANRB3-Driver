//! The decoder on synthetic bursts: frames with known content, encoded the
//! way the device sends them, with errors placed where a test needs them.

mod common;

use anrb::crc::Crc;
use anrb::{Decoder, Frame, Options, Stats};
use common::*;

/// Real DF17 frames with valid parity: an even and an odd airborne position
/// from 4009DA.
const EVEN: &str = "8d4009da5833318e2bd82af8c6f5";
const ODD: &str = "8d4009da583324fef1cbc5c7449d";

fn decode(dec: &mut Decoder, w: &[u8]) -> Option<Frame> {
    dec.decode_burst(w, 1234)
}

#[test]
fn helper_parity_matches_real_frames_and_the_library() {
    let crc = Crc::new();
    for s in [EVEN, ODD] {
        let f = hex(s);
        assert_eq!(with_parity(&f[..11]), f, "{s}");
    }
    for f in [
        df11(0x4009DA),
        df17(0x3C6551, [0x99, 0x44, 0x11, 0x00, 0x00, 0x00, 0x00]),
        with_parity(&[0x5D, 1, 2, 3]),
    ] {
        assert_eq!(crc.crc24(&f), 0);
    }
    // The overlaid formats leave the address as the syndrome's source.
    let f = df4(0x3C6551);
    assert_eq!(crc.crc24(&f), crc.ap_syndrome(0x3C6551));
}

#[test]
fn helper_encodes_eight_samples_per_bit() {
    // 0x80 is a 1 then seven 0s.
    assert_eq!(
        samples(&[0x80])[..16],
        [1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1]
    );
    assert_eq!(
        burst(&[0x80]),
        [0xF0, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F, 0x0F]
    );
    assert_eq!(burst(&hex(EVEN)).len(), 112);
    assert_eq!(burst(&df11(1)).len(), 56);
    assert_eq!(pack(&[1, 0, 1]), [0xA0]);
    let mut s = samples(&[0xFF]);
    erase(&mut s, 1);
    assert_eq!(pack(&s), [0xF0, 0x00, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0, 0xF0]);
}

#[test]
fn clean_long_frames_round_trip() {
    let mut dec = Decoder::new();
    for s in [EVEN, ODD] {
        let f = decode(&mut dec, &burst(&hex(s))).expect(s);
        assert_eq!(f.hex(), s);
        assert_eq!(f.corrected, 0);
        assert_eq!(f.ms, 1234);
        assert_eq!(f.df(), 17);
        assert_eq!(f.icao(), 0x4009DA);
        assert_eq!(f.type_code(), Some(11));
    }
    assert_eq!((dec.stats.frames, dec.stats.clean), (2, 2));
}

#[test]
fn clean_short_frame_round_trips() {
    let mut dec = Decoder::new();
    let want = df11(0x4CA2D6);
    let f = decode(&mut dec, &burst(&want)).expect("DF11");
    assert_eq!(f.as_bytes(), &want[..]);
    assert_eq!(
        (f.len(), f.df(), f.icao(), f.corrected),
        (7, 11, 0x4CA2D6, 0)
    );
    assert_eq!(f.type_code(), None);
}

/// The first `n` positions from `from` on where `frame` has a 1 bit.
fn ones(frame: &[u8], from: usize, n: usize) -> Vec<usize> {
    (from..frame.len() * 8)
        .filter(|&i| bit(frame, i))
        .take(n)
        .collect()
}

#[test]
fn one_bit_errors_are_corrected() {
    let mut dec = Decoder::new();
    let good = hex(EVEN);
    for i in [5usize, 30, 57, 88, 111] {
        let mut f = good.clone();
        flip(&mut f, i);
        let got = decode(&mut dec, &burst(&f)).unwrap_or_else(|| panic!("bit {i}"));
        assert_eq!(got.as_bytes(), &good[..], "bit {i}");
        assert_eq!(got.corrected, 1, "bit {i}");
    }
    let short = df11(0x4CA2D6);
    for i in [5usize, 31, 55] {
        let mut f = short.clone();
        flip(&mut f, i);
        let got = decode(&mut dec, &burst(&f)).unwrap_or_else(|| panic!("short bit {i}"));
        assert_eq!(
            (got.as_bytes(), got.corrected),
            (&short[..], 1),
            "short bit {i}"
        );
    }
    assert_eq!(
        (dec.stats.frames, dec.stats.fixed1, dec.stats.clean),
        (8, 8, 0)
    );
}

#[test]
fn two_hard_errors_need_the_blind_pass() {
    let good = hex(EVEN);
    let mut f = good.clone();
    flip(&mut f, 20);
    flip(&mut f, 60);
    let w = burst(&f);

    // Every bit is equally confident, so the soft search looks only at the
    // last 14 bits and does not find errors this early.
    let mut dec = Decoder::new();
    assert_eq!(decode(&mut dec, &w), None);
    assert!(dec.stats.soft_tried > 0);
    assert_eq!(dec.stats.soft_hit, 0);

    dec.opts = Options {
        soft: false,
        blind2: true,
        overlaid: false,
    };
    let got = decode(&mut dec, &w).expect("blind 2-bit");
    assert_eq!((got.as_bytes(), got.corrected), (&good[..], 2));
    assert_eq!(dec.stats.fixed2, 1);
    assert_eq!(
        dec.stats.cfgs_by_pass[2], 36,
        "only the first burst ran the soft pass"
    );
}

#[test]
fn soft_search_prefers_the_last_bits_when_confidence_ties() {
    let good = hex(EVEN);
    let mut f = good.clone();
    flip(&mut f, 100);
    flip(&mut f, 105);
    let mut dec = Decoder::new();
    let got = decode(&mut dec, &burst(&f)).expect("soft 2-bit");
    assert_eq!((got.as_bytes(), got.corrected), (&good[..], 2));
    assert_eq!((dec.stats.soft_hit, dec.stats.fixed2), (1, 1));
}

#[test]
fn soft_search_corrects_low_confidence_bits() {
    let good = hex(EVEN);
    let pick = ones(&good, 8, 6);
    let (a, b) = (pick[0], pick[5]);
    let mut s = samples(&good);
    erase(&mut s, a);
    erase(&mut s, b);
    let w = pack(&s);

    let mut dec = Decoder::new();
    let got = decode(&mut dec, &w).expect("soft");
    assert_eq!((got.as_bytes(), got.corrected), (&good[..], 2));
    assert_eq!(
        (dec.stats.soft_hit, dec.stats.fixed2, dec.stats.frames),
        (1, 1, 1)
    );
    assert!(dec.stats.soft_tried >= 1);

    let mut dec = Decoder::new();
    dec.opts.soft = false;
    assert_eq!(
        decode(&mut dec, &w),
        None,
        "no pass corrects it without soft decisions"
    );
}

#[test]
fn a_single_erased_bit_is_a_one_bit_fix() {
    let good = df11(0x4CA2D6);
    let mut s = samples(&good);
    erase(&mut s, ones(&good, 8, 1)[0]);
    let mut dec = Decoder::new();
    let got = decode(&mut dec, &pack(&s)).expect("1-bit");
    assert_eq!((got.as_bytes(), got.corrected), (&good[..], 1));
    assert_eq!(dec.stats.soft_tried, 0, "found before the soft pass");
}

#[test]
fn overlaid_frames_need_the_option_and_a_trusted_address() {
    let addr = 0x4CA2D6;
    let reply = df4(addr);
    let w = burst(&reply);
    let mut dec = Decoder::new();
    assert_eq!(decode(&mut dec, &w), None, "off by default");
    assert_eq!(dec.stats.cfgs_by_pass[3], 0);

    dec.opts.overlaid = true;
    assert_eq!(decode(&mut dec, &w), None, "address not heard yet");
    assert!(dec.stats.overlaid_tried > 0);

    assert!(decode(&mut dec, &burst(&df11(addr))).is_some());
    assert_eq!(dec.whitelist_len(), 1);
    assert_eq!(decode(&mut dec, &w), None, "heard once is not trusted");

    assert!(decode(&mut dec, &burst(&df11(addr))).is_some());
    let got = decode(&mut dec, &w).expect("trusted address");
    assert_eq!(
        (got.as_bytes(), got.corrected, got.df()),
        (&reply[..], 0, 4)
    );
    assert_eq!(dec.stats.overlaid_hit, 1);
    assert_eq!(dec.whitelist_len(), 1);
    assert_eq!(dec.whitelist_evicted(), 0);

    // A long overlaid format, DF20, from the same address.
    let long = with_ap(
        &[
            0xA0, 0x00, 0x17, 0x18, 0x20, 0x2C, 0xC3, 0x71, 0xC3, 0x2C, 0xE0,
        ],
        addr,
    );
    let got = decode(&mut dec, &burst(&long)).expect("DF20");
    assert_eq!(
        (got.as_bytes(), got.df(), got.type_code()),
        (&long[..], 20, None)
    );

    // Another address is not trusted.
    assert_eq!(decode(&mut dec, &burst(&df4(0x3C6551))), None);
    assert_eq!(dec.stats.overlaid_hit, 2);
}

#[test]
fn the_gate_refuses_frames_no_aircraft_sends() {
    let mut dec = Decoder::new();
    let me = |tc: u8| [tc << 3, 0, 0, 0, 0, 0, 0];
    for tc in [23u8, 24, 27, 30] {
        assert_eq!(
            decode(&mut dec, &burst(&df17(0x4009DA, me(tc)))),
            None,
            "type code {tc}"
        );
    }
    for tc in [0u8, 4, 11, 19, 22, 28, 29, 31] {
        let f = df17(0x4009DA, me(tc));
        let got = decode(&mut dec, &burst(&f)).unwrap_or_else(|| panic!("type code {tc}"));
        assert_eq!(got.type_code(), Some(tc));
    }
    for addr in [0, 0xFF_FFFF, 0xF0_1234] {
        assert_eq!(decode(&mut dec, &burst(&df11(addr))), None, "{addr:06X}");
        assert_eq!(
            decode(&mut dec, &burst(&df17(addr, me(11)))),
            None,
            "{addr:06X}"
        );
    }
    // DF18 passes; DF19 is a long format the decoder does not handle.
    let df18 = with_parity(&[0x90, 0x4C, 0xA2, 0xD6, 0x58, 0, 0, 0, 0, 0, 0]);
    assert_eq!(decode(&mut dec, &burst(&df18)).map(|f| f.df()), Some(18));
    let df19 = with_parity(&[0x98, 0x4C, 0xA2, 0xD6, 0x58, 0, 0, 0, 0, 0, 0]);
    assert_eq!(decode(&mut dec, &burst(&df19)), None);
}

#[test]
fn noise_is_rejected() {
    let mut dec = Decoder::new();
    dec.opts = Options {
        soft: true,
        blind2: true,
        overlaid: true,
    };
    for seed in 0..200u64 {
        for len in [56usize, 112] {
            assert_eq!(
                decode(&mut dec, &noise(seed, len)),
                None,
                "seed {seed} len {len}"
            );
        }
    }
    assert_eq!(decode(&mut dec, &[]), None);
    assert_eq!(decode(&mut dec, &[0xFF; 112]), None);
    assert_eq!(dec.stats.frames, 0);
    assert_eq!(dec.stats.yield_pct(), 0.0);
}

#[test]
fn silence_reaches_no_framing() {
    let mut dec = Decoder::new();
    assert_eq!(decode(&mut dec, &[0; 112]), None);
    let s = dec.stats;
    assert_eq!((s.bursts, s.offsets, s.crc_calls), (1, 0, 0));
    assert_eq!(s.stage_runs, 36);
}

#[test]
fn an_all_zero_candidate_is_not_checked() {
    // A frame of zero bits is DF0, an overlaid format, and every bit of it is 0.
    let mut dec = Decoder::new();
    dec.opts.overlaid = true;
    assert_eq!(decode(&mut dec, &burst(&[0; 7])), None);
    let s = dec.stats;
    assert!(s.offsets_kept > 0);
    assert_eq!((s.crc_calls, s.overlaid_tried), (0, 0));
}

#[test]
fn stats_count_passes_and_outcomes() {
    let mut dec = Decoder::new();
    assert_eq!(Stats::default().yield_pct(), 0.0);

    // A failed burst runs every enabled pass over the whole grid, and each
    // configuration is demodulated only once.
    assert_eq!(decode(&mut dec, &noise(7, 112)), None);
    let s = dec.stats;
    assert_eq!(s.bursts, 1);
    assert_eq!(s.cfgs_by_pass, [36, 36, 36, 0, 0]);
    assert_eq!(s.stage_runs, 36);
    assert_eq!(s.offsets_by_pass.iter().sum::<u64>(), s.offsets);
    assert!(s.crc_calls <= s.offsets_kept && s.offsets_kept <= s.offsets);

    dec.opts = Options {
        soft: true,
        blind2: true,
        overlaid: true,
    };
    assert_eq!(decode(&mut dec, &noise(8, 112)), None);
    let s = dec.stats;
    assert_eq!(s.cfgs_by_pass, [72, 72, 72, 36, 36]);
    assert_eq!(s.stage_runs, 72);

    // A clean frame stops in the first pass.
    let before = dec.stats;
    assert!(decode(&mut dec, &burst(&hex(ODD))).is_some());
    let s = dec.stats;
    assert_eq!(s.cfgs_by_pass[1..], before.cfgs_by_pass[1..]);
    assert!(s.stage_runs - before.stage_runs <= 36);
    assert_eq!(
        (s.bursts, s.frames, s.clean, s.fixed1, s.fixed2),
        (3, 1, 1, 0, 0)
    );
    assert!((s.yield_pct() - 100.0 / 3.0).abs() < 1e-9);
}

#[test]
fn frames_from_elsewhere() {
    let f = Frame::new(&hex(EVEN), 5).expect("14 bytes, DF17");
    assert_eq!(
        (f.len(), f.ms, f.corrected, f.hex().as_str()),
        (14, 5, 0, EVEN)
    );
    assert!(!f.is_empty());
    let s = Frame::new(&df11(0xABCDEF), 0).expect("7 bytes, DF11");
    assert_eq!((s.len(), s.icao(), s.type_code()), (7, 0xABCDEF, None));
    assert_eq!(Frame::new(&[], 0), None);
    assert_eq!(Frame::new(&hex(EVEN)[..7], 0), None, "DF17 needs 14 bytes");
    assert_eq!(Frame::new(&[0x5D; 14], 0), None, "DF11 needs 7 bytes");
    // Frames compare by content.
    let mut dec = Decoder::new();
    let got = dec.decode_burst(&burst(&hex(EVEN)), 5).unwrap();
    assert_eq!(got, f);
}

#[test]
fn decode_reuses_state_across_burst_lengths() {
    // Short and long bursts in turn: the per-burst cache is resized and reset
    // without results from one burst leaking into the next.
    let mut dec = Decoder::new();
    let long = hex(EVEN);
    let short = df11(0x4CA2D6);
    let big = [burst(&long), vec![0; 400]].concat();
    for w in [burst(&short), big.clone(), burst(&short), burst(&long)] {
        assert!(decode(&mut dec, &w).is_some());
    }
    assert_eq!(
        decode(&mut dec, &big).map(|f| f.hex()),
        Some(EVEN.to_string())
    );
    assert_eq!(decode(&mut dec, &noise(1, 56)), None);
}

/// The fast demodulator agrees with the reference one for every grid setting
/// and more, on bursts of every shape: frames, noise, silence, the prefix
/// boundary at 100 bytes, and sparse data.
#[test]
fn fast_demodulator_matches_reference_on_synthetic_bursts() {
    let mut ws: Vec<Vec<u8>> = vec![
        vec![],
        vec![0; 1],
        vec![0x01],
        vec![0x80],
        vec![0; 112],
        burst(&hex(EVEN)),
        burst(&hex(ODD)),
        burst(&df11(0x4CA2D6)),
        burst(&df4(1)),
    ];
    for len in [1usize, 7, 20, 56, 99, 100, 101, 112, 113, 300] {
        ws.push(noise(len as u64, len));
    }
    // Long runs of zeros before the first set bit.
    let mut sparse = vec![0u8; 101];
    sparse[60] = 0x10;
    ws.push(sparse);
    let mut dec = Decoder::new();
    for w in &ws {
        for skipbits in 0..=12usize {
            for gt in 0..=3i32 {
                for bias in [false, true] {
                    dec.verify_stages(w, skipbits, gt, bias)
                        .unwrap_or_else(|e| {
                            panic!("len={} skip={skipbits} gt={gt} bias={bias}: {e}", w.len())
                        });
                }
            }
        }
    }
}

/// Bursts longer than the demodulator's 4096 groups are cut at the same
/// place by both versions, with the last group still read from real samples.
#[test]
fn fast_demodulator_matches_reference_on_oversized_bursts() {
    let mut ws: Vec<Vec<u8>> = [2047usize, 2048, 2049, 3000]
        .iter()
        .map(|&n| noise(n as u64, n))
        .collect();
    // The first set bit at 3, so the kept samples do not end on a byte.
    let mut late = vec![0xFF; 2100];
    late[0] = 0x10;
    ws.push(late);
    let mut dec = Decoder::new();
    for w in &ws {
        for skipbits in [0usize, 3, 8] {
            for bias in [false, true] {
                dec.verify_stages(w, skipbits, 1, bias)
                    .unwrap_or_else(|e| panic!("len={} skip={skipbits} bias={bias}: {e}", w.len()));
            }
        }
    }
}
