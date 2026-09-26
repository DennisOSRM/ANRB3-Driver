//! Replay a recorded burst log through the decoder, with no device present.
//!
//! Decodes a recorded log offline and prints a fingerprint of the recovered
//! frames plus decoder statistics, for comparing decoder changes and for
//! profiling.

use anrb::corpus::{self, Fingerprint};
use anrb::{protocol, Backend, Decoder, Options, Tracker};
use std::env;

const USAGE: &str = "\
anrb-replay - decode a recorded log offline

usage: anrb-replay [FILE] [FLAGS]

  FILE                  the log to decode (default captures/tuning_15min_v2.raw)
  --no-soft             no soft-decision retry on a failed burst
  --2bit                also try blind two-bit corrections on DF11/DF17, not
                        guided by demodulator confidence
  --overlaid            also try the address-overlaid downlink formats
  --bursts              FILE is a flat list of bursts, as captures/bursts.bin,
                        not a raw USB log
  -h, --help            show this help
";

/// Print `msg` and the usage text to stderr, and exit with status 2.
fn usage_error(msg: &str) -> ! {
    eprint!("anrb-replay: {msg}\n\n{USAGE}");
    std::process::exit(2);
}

fn main() {
    let mut path = None;
    let mut opts = Options::default();
    let mut raw_bursts = false;
    for a in env::args().skip(1) {
        match a.as_str() {
            "--no-soft" => opts.soft = false,
            "--2bit" => opts.blind2 = true,
            "--overlaid" => opts.overlaid = true,
            "--bursts" => raw_bursts = true,
            "-h" | "--help" => { print!("{USAGE}"); return; }
            _ if a.starts_with('-') => usage_error(&format!("unknown argument {a:?}")),
            _ if path.is_none() => path = Some(a),
            _ => usage_error(&format!("more than one file: {a:?}")),
        }
    }
    let path = path.unwrap_or_else(|| "captures/tuning_15min_v2.raw".into());

    // The timer includes reading the file and excludes tracking.
    let start = std::time::Instant::now();
    let d = std::fs::read(&path).unwrap_or_else(|e| { eprintln!("{path}: {e}"); std::process::exit(1) });
    let mut dec = Decoder::new();
    dec.opts = opts;

    let mut fp = Fingerprint::new();
    let mut npong = 0usize;
    let mut icaos = std::collections::HashSet::new();
    let mut frames = Vec::new();

    if raw_bursts {
        for b in corpus::burst_file(&d) {
            if let Some(fr) = dec.decode_burst(b, 0) {
                icaos.insert(fr.icao());
                fp.add(&fr);
            }
        }
    } else {
        let Some(log) = corpus::raw_log(&d) else {
            eprintln!("{path}: bad magic");
            std::process::exit(1);
        };
        for (ms, data) in log {
            if protocol::is_pong(data) { npong += 1; continue; }
            for (off, len) in protocol::segments(data) {
                if let Some(fr) = dec.decode_burst(&data[off..off + len], ms) {
                    icaos.insert(fr.icao());
                    fp.add(&fr);
                    frames.push(fr);
                }
            }
        }
    }

    let dt = start.elapsed().as_secs_f64();
    let s = dec.stats;

    // The timer includes reading the file and excludes tracking.
    let mut trk = Tracker::new();
    let mut placed = std::collections::HashSet::new();
    for f in &frames {
        if let Some((icao, _)) = trk.update(f) {
            if trk.table[&icao].lat.is_some() { placed.insert(icao); }
        }
    }

    println!("{fp}");
    println!("  bursts       {} data + {} PONG", s.bursts, npong);
    println!("  frames       {}  (clean={} 1bit={} 2bit={})  yield={:.1}%",
             s.frames, s.clean, s.fixed1, s.fixed2,
             s.yield_pct());
    println!("  aircraft     {} distinct", icaos.len());
    println!("  tracker      {} placed; refused_pos={}", placed.len(), trk.implausible);
    println!("  soft         {} hits from {} attempts", s.soft_hit, s.soft_tried);
    println!("  whitelist    {} addresses; overlaid tried={} hit={}",
             dec.whitelist_len(), s.overlaid_tried, s.overlaid_hit);
    println!("  backend      {}", Backend::detect().name());
    println!("  per burst    {:.1} demodulator runs, {:.1} framing offsets, {:.1} CRC candidates",
             s.stage_runs as f64 / s.bursts.max(1) as f64,
             s.offsets as f64 / s.bursts.max(1) as f64,
             s.crc_calls as f64 / s.bursts.max(1) as f64);
    println!("  offsets      {} examined, {} kept ({:.1}%)  per pass: clean={} 1bit={} soft={} overlaid={} blind={}",
             s.offsets, s.offsets_kept, 100.0 * s.offsets_kept as f64 / s.offsets.max(1) as f64,
             s.offsets_by_pass[0], s.offsets_by_pass[1], s.offsets_by_pass[2], s.offsets_by_pass[3],
             s.offsets_by_pass[4]);
    println!("  configs      entered per pass: clean={} 1bit={} soft={} overlaid={} blind={}",
             s.cfgs_by_pass[0], s.cfgs_by_pass[1], s.cfgs_by_pass[2], s.cfgs_by_pass[3],
             s.cfgs_by_pass[4]);
    #[cfg(feature = "profile")]
    {
        let p = dec.phases;
        let t = p.total.max(1) as f64;
        println!("  phase breakdown (cycles, share of decode_burst):");
        let mut rows = [("demodulator", p.demod), ("framing + CRC", p.frame),
                        ("  of which soft search", p.soft),
                        ("cache copy", p.cache_copy)];
        rows.sort_by_key(|r| std::cmp::Reverse(r.1));
        for (n, v) in rows {
            println!("     {:<24} {:>14}  {:>5.1}%", n, v, 100.0 * v as f64 / t);
        }
        let acct = p.demod + p.frame + p.cache_copy;
        println!("     {:<24} {:>14}  {:>5.1}%", "unattributed", p.total.saturating_sub(acct),
                 100.0 * p.total.saturating_sub(acct) as f64 / t);
        println!("     {:<24} {:>14}", "total", p.total);
        let g = dec.stage_phases();
        println!("  inside the demodulator ({} memo misses, {} hits):", g.memo_misses, g.memo_hits);
        let mut gr = [("find first set bit", g.find_s), ("bit-shift align", g.shift),
                      ("nibble popcount (SIMD)", g.popcount), ("margin computation", g.margin),
                      ("threshold, no hysteresis (SIMD)", g.thresh_plain),
                      ("threshold, hysteresis (serial)", g.thresh_hyst)];
        gr.sort_by_key(|r| std::cmp::Reverse(r.1));
        for (n, v) in gr {
            println!("     {:<34} {:>14}  {:>5.1}% of decode", n, v, 100.0 * v as f64 / t);
        }
        let q = dec.soft_phases();
        println!("  inside the soft search ({} calls):", q.calls);
        let mut sr = [("crc24", q.crc), ("candidate selection", q.select),
                      ("single-bit checks", q.single),
                      ("pair checks", q.pair)];
        sr.sort_by_key(|r| std::cmp::Reverse(r.1));
        for (n, v) in sr {
            println!("     {:<24} {:>14}  {:>5.1}% of decode", n, v, 100.0 * v as f64 / t);
        }
    }
    println!("  decode time  {:.2} s for {} bursts ({:.1} us/burst)",
             dt, s.bursts, if s.bursts > 0 { dt * 1e6 / s.bursts as f64 } else { 0.0 });
}
