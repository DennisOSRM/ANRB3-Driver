//! Replay a recorded burst log through the decoder, with no device present.
//!
//! Decodes a recorded log offline and prints a fingerprint of the recovered
//! frames plus decoder statistics, for comparing decoder changes and for
//! profiling.

use anrb::corpus::{self, Fingerprint};
use anrb::{protocol, Backend, Decoder, Frame, Options, Tracker};
use std::collections::HashSet;
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

/// What the command line asks for.
struct Config {
    path: String,
    opts: Options,
    /// The file is a flat list of bursts rather than a raw USB log.
    raw_bursts: bool,
}

/// The outcome of reading the command line.
enum Command {
    Run(Config),
    Help,
}

/// Read the arguments that follow the program name. An error is the message
/// to show above the usage text.
fn parse_args(args: impl IntoIterator<Item = String>) -> Result<Command, String> {
    let mut path = None;
    let mut opts = Options::default();
    let mut raw_bursts = false;
    for a in args {
        match a.as_str() {
            "--no-soft" => opts.soft = false,
            "--2bit" => opts.blind2 = true,
            "--overlaid" => opts.overlaid = true,
            "--bursts" => raw_bursts = true,
            "-h" | "--help" => return Ok(Command::Help),
            _ if a.starts_with('-') => return Err(format!("unknown argument {a:?}")),
            _ if path.is_none() => path = Some(a),
            _ => return Err(format!("more than one file: {a:?}")),
        }
    }
    let path = path.unwrap_or_else(|| "captures/tuning_15min_v2.raw".into());
    Ok(Command::Run(Config {
        path,
        opts,
        raw_bursts,
    }))
}

/// What decoding a whole file produced.
struct Replay {
    dec: Decoder,
    fp: Fingerprint,
    npong: usize,
    icaos: HashSet<u32>,
    /// The decoded frames of a raw log, in order, for the tracker. Empty for
    /// a burst file.
    frames: Vec<Frame>,
}

/// Decode every burst in `d`. None when a raw log does not start with the
/// log header.
fn decode(d: &[u8], opts: Options, raw_bursts: bool) -> Option<Replay> {
    let mut dec = Decoder::new();
    dec.opts = opts;

    let mut fp = Fingerprint::new();
    let mut npong = 0usize;
    let mut icaos = HashSet::new();
    let mut frames = Vec::new();

    if raw_bursts {
        for b in corpus::burst_file(d) {
            if let Some(fr) = dec.decode_burst(b, 0) {
                icaos.insert(fr.icao());
                fp.add(&fr);
            }
        }
    } else {
        for (ms, data) in corpus::raw_log(d)? {
            if protocol::is_pong(data) {
                npong += 1;
                continue;
            }
            for (off, len) in protocol::segments(data) {
                if let Some(fr) = dec.decode_burst(&data[off..off + len], ms) {
                    icaos.insert(fr.icao());
                    fp.add(&fr);
                    frames.push(fr);
                }
            }
        }
    }
    Some(Replay {
        dec,
        fp,
        npong,
        icaos,
        frames,
    })
}

/// What the tracker made of the frames.
struct Tracked {
    /// Aircraft that got a position.
    placed: usize,
    /// Positions the tracker refused as implausible.
    refused: u64,
}

fn track(frames: &[Frame]) -> Tracked {
    let mut trk = Tracker::new();
    let mut placed = HashSet::new();
    for f in frames {
        if let Some((icao, _)) = trk.update(f) {
            if trk.table[&icao].lat.is_some() {
                placed.insert(icao);
            }
        }
    }
    Tracked {
        placed: placed.len(),
        refused: trk.implausible,
    }
}

/// The summary printed at the end: the fingerprint, then the decoder's
/// counters. `dt` is the decode time in seconds.
fn report(r: &Replay, t: &Tracked, dt: f64) -> String {
    let mut out = String::new();
    macro_rules! line {
        ($($arg:tt)*) => {{ out.push_str(&format!($($arg)*)); out.push('\n'); }};
    }
    let s = r.dec.stats;
    line!("{}", r.fp);
    line!("  bursts       {} data + {} PONG", s.bursts, r.npong);
    line!(
        "  frames       {}  (clean={} 1bit={} 2bit={})  yield={:.1}%",
        s.frames,
        s.clean,
        s.fixed1,
        s.fixed2,
        s.yield_pct()
    );
    line!("  aircraft     {} distinct", r.icaos.len());
    line!(
        "  tracker      {} placed; refused_pos={}",
        t.placed,
        t.refused
    );
    line!(
        "  soft         {} hits from {} attempts",
        s.soft_hit,
        s.soft_tried
    );
    line!(
        "  whitelist    {} addresses; overlaid tried={} hit={}",
        r.dec.whitelist_len(),
        s.overlaid_tried,
        s.overlaid_hit
    );
    line!("  backend      {}", Backend::detect().name());
    line!(
        "  per burst    {:.1} demodulator runs, {:.1} framing offsets, {:.1} CRC candidates",
        s.stage_runs as f64 / s.bursts.max(1) as f64,
        s.offsets as f64 / s.bursts.max(1) as f64,
        s.crc_calls as f64 / s.bursts.max(1) as f64
    );
    line!("  offsets      {} examined, {} kept ({:.1}%)  per pass: clean={} 1bit={} soft={} overlaid={} blind={}",
          s.offsets, s.offsets_kept, 100.0 * s.offsets_kept as f64 / s.offsets.max(1) as f64,
          s.offsets_by_pass[0], s.offsets_by_pass[1], s.offsets_by_pass[2], s.offsets_by_pass[3],
          s.offsets_by_pass[4]);
    line!(
        "  configs      entered per pass: clean={} 1bit={} soft={} overlaid={} blind={}",
        s.cfgs_by_pass[0],
        s.cfgs_by_pass[1],
        s.cfgs_by_pass[2],
        s.cfgs_by_pass[3],
        s.cfgs_by_pass[4]
    );
    #[cfg(feature = "profile")]
    {
        let dec = &r.dec;
        let p = dec.phases;
        let t = p.total.max(1) as f64;
        line!("  phase breakdown (cycles, share of decode_burst):");
        let mut rows = [
            ("demodulator", p.demod),
            ("framing + CRC", p.frame),
            ("  of which soft search", p.soft),
            ("cache copy", p.cache_copy),
        ];
        rows.sort_by_key(|r| std::cmp::Reverse(r.1));
        for (n, v) in rows {
            line!("     {:<24} {:>14}  {:>5.1}%", n, v, 100.0 * v as f64 / t);
        }
        let acct = p.demod + p.frame + p.cache_copy;
        line!(
            "     {:<24} {:>14}  {:>5.1}%",
            "unattributed",
            p.total.saturating_sub(acct),
            100.0 * p.total.saturating_sub(acct) as f64 / t
        );
        line!("     {:<24} {:>14}", "total", p.total);
        let g = dec.stage_phases();
        line!(
            "  inside the demodulator ({} memo misses, {} hits):",
            g.memo_misses,
            g.memo_hits
        );
        let mut gr = [
            ("find first set bit", g.find_s),
            ("bit-shift align", g.shift),
            ("nibble popcount (SIMD)", g.popcount),
            ("margin computation", g.margin),
            ("threshold, no hysteresis (SIMD)", g.thresh_plain),
            ("threshold, hysteresis (serial)", g.thresh_hyst),
        ];
        gr.sort_by_key(|r| std::cmp::Reverse(r.1));
        for (n, v) in gr {
            line!(
                "     {:<34} {:>14}  {:>5.1}% of decode",
                n,
                v,
                100.0 * v as f64 / t
            );
        }
        let q = dec.soft_phases();
        line!("  inside the soft search ({} calls):", q.calls);
        let mut sr = [
            ("crc24", q.crc),
            ("candidate selection", q.select),
            ("single-bit checks", q.single),
            ("pair checks", q.pair),
        ];
        sr.sort_by_key(|r| std::cmp::Reverse(r.1));
        for (n, v) in sr {
            line!(
                "     {:<24} {:>14}  {:>5.1}% of decode",
                n,
                v,
                100.0 * v as f64 / t
            );
        }
    }
    line!(
        "  decode time  {:.2} s for {} bursts ({:.1} us/burst)",
        dt,
        s.bursts,
        if s.bursts > 0 {
            dt * 1e6 / s.bursts as f64
        } else {
            0.0
        }
    );
    out
}

fn main() {
    let cfg = match parse_args(env::args().skip(1)) {
        Ok(Command::Run(cfg)) => cfg,
        Ok(Command::Help) => {
            print!("{USAGE}");
            return;
        }
        Err(msg) => usage_error(&msg),
    };
    let path = cfg.path;

    // The timer includes reading the file and excludes tracking.
    let start = std::time::Instant::now();
    let d = std::fs::read(&path).unwrap_or_else(|e| {
        eprintln!("{path}: {e}");
        std::process::exit(1)
    });
    let Some(replay) = decode(&d, cfg.opts, cfg.raw_bursts) else {
        eprintln!("{path}: bad magic");
        std::process::exit(1);
    };
    let dt = start.elapsed().as_secs_f64();

    let tracked = track(&replay.frames);
    print!("{}", report(&replay, &tracked, dt));
}

#[cfg(test)]
mod tests {
    use super::*;

    // An even and an odd airborne position of 4009DA, from the tracker's
    // tests. Together with a second even message they place the aircraft.
    const EVEN: &str = "8d4009da5833318e2bd82af8c6f5";
    const ODD: &str = "8d4009da583324fef1cbc5c7449d";

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
            .collect()
    }

    /// The samples the receiver sends for a frame: one byte of eight samples
    /// per bit, high then low for a one and low then high for a zero.
    fn samples(frame: &str) -> Vec<u8> {
        hex(frame)
            .iter()
            .flat_map(|b| {
                (0..8)
                    .rev()
                    .map(move |k| if (b >> k) & 1 == 1 { 0xF0 } else { 0x0F })
            })
            .collect()
    }

    fn raw_log(bursts: &[(u32, Vec<u8>)]) -> Vec<u8> {
        let mut d = corpus::RAW_MAGIC.to_vec();
        for (ms, b) in bursts {
            d.extend_from_slice(&ms.to_le_bytes());
            d.extend_from_slice(&(b.len() as u16).to_le_bytes());
            d.extend_from_slice(b);
        }
        d
    }

    fn args(a: &[&str]) -> Result<Command, String> {
        parse_args(a.iter().map(|s| s.to_string()))
    }

    fn config(a: &[&str]) -> Config {
        match args(a) {
            Ok(Command::Run(c)) => c,
            Ok(Command::Help) => panic!("{a:?} asked for help"),
            Err(e) => panic!("{a:?}: {e}"),
        }
    }

    #[test]
    fn no_arguments_read_the_default_log_with_default_options() {
        let c = config(&[]);
        assert_eq!(c.path, "captures/tuning_15min_v2.raw");
        assert!(c.opts.soft && !c.opts.blind2 && !c.opts.overlaid);
        assert!(!c.raw_bursts);
    }

    #[test]
    fn each_flag_sets_its_option_and_a_word_is_the_file() {
        let c = config(&["--no-soft", "my.raw", "--2bit", "--overlaid", "--bursts"]);
        assert_eq!(c.path, "my.raw");
        assert!(!c.opts.soft && c.opts.blind2 && c.opts.overlaid);
        assert!(c.raw_bursts);
    }

    #[test]
    fn help_wins_over_the_other_arguments() {
        assert!(matches!(args(&["-h"]), Ok(Command::Help)));
        assert!(matches!(args(&["a.raw", "--help"]), Ok(Command::Help)));
    }

    #[test]
    fn unknown_flags_and_a_second_file_are_errors() {
        assert_eq!(
            args(&["--fast"]).err().unwrap(),
            "unknown argument \"--fast\""
        );
        assert_eq!(
            args(&["a.raw", "b.raw"]).err().unwrap(),
            "more than one file: \"b.raw\""
        );
    }

    #[test]
    fn a_raw_log_decodes_counts_pongs_and_places_the_aircraft() {
        // The first two frames share one USB transfer, split by the 00 0a
        // that ends each frame.
        let mut two = samples(EVEN);
        two.extend_from_slice(&[0x00, 0x0a]);
        two.extend(samples(ODD));
        let d = raw_log(&[(0, two), (700, b"PONG".to_vec()), (1000, samples(EVEN))]);

        let r = decode(&d, Options::default(), false).expect("a raw log");
        assert_eq!(r.npong, 1);
        assert_eq!(r.dec.stats.bursts, 3);
        assert_eq!(r.dec.stats.frames, 3);
        assert_eq!(r.dec.stats.clean, 3);
        assert_eq!(r.fp.frames, 3);
        assert_eq!(r.icaos.iter().copied().collect::<Vec<_>>(), vec![0x4009DA]);
        let ms: Vec<u32> = r.frames.iter().map(|f| f.ms).collect();
        assert_eq!(ms, vec![0, 0, 1000]);
        let hexes: Vec<String> = r.frames.iter().map(|f| f.hex()).collect();
        assert_eq!(hexes, vec![EVEN, ODD, EVEN]);

        let t = track(&r.frames);
        assert_eq!(t.placed, 1);
        assert_eq!(t.refused, 0);
    }

    #[test]
    fn a_file_without_the_log_header_is_refused() {
        assert!(decode(b"not a log at all", Options::default(), false).is_none());
    }

    #[test]
    fn a_burst_file_decodes_but_leaves_nothing_to_track() {
        let mut d = Vec::new();
        for f in [EVEN, ODD] {
            let b = samples(f);
            d.extend_from_slice(&(b.len() as u16).to_le_bytes());
            d.extend(b);
        }
        let r = decode(&d, Options::default(), true).expect("burst files have no header");
        assert_eq!(r.dec.stats.bursts, 2);
        assert_eq!(r.fp.frames, 2);
        assert_eq!(r.icaos.len(), 1);
        assert_eq!(r.npong, 0);
        assert!(r.frames.is_empty());
    }

    #[test]
    fn noise_is_counted_as_a_burst_without_a_frame() {
        let d = raw_log(&[(5, vec![0x55; 112])]);
        let r = decode(&d, Options::default(), false).unwrap();
        assert_eq!(r.dec.stats.bursts, 1);
        assert_eq!(r.dec.stats.frames, 0);
        assert!(r.frames.is_empty() && r.icaos.is_empty());
    }

    #[test]
    fn the_report_states_the_counts() {
        let d = raw_log(&[
            (0, samples(EVEN)),
            (500, samples(ODD)),
            (600, b"PONG".to_vec()),
            (1000, samples(EVEN)),
            (1100, vec![0x55; 112]),
        ]);
        let r = decode(&d, Options::default(), false).unwrap();
        let t = track(&r.frames);
        let text = report(&r, &t, 0.5);
        let lines: Vec<&str> = text.lines().collect();
        assert_eq!(lines[0], r.fp.to_string());
        assert!(
            lines[0].starts_with("FINGERPRINT frames=3 hash="),
            "{}",
            lines[0]
        );
        assert_eq!(lines[1], "  bursts       4 data + 1 PONG");
        assert_eq!(
            lines[2],
            "  frames       3  (clean=3 1bit=0 2bit=0)  yield=75.0%"
        );
        assert_eq!(lines[3], "  aircraft     1 distinct");
        assert_eq!(lines[4], "  tracker      1 placed; refused_pos=0");
        assert!(lines[5].starts_with("  soft         "), "{}", lines[5]);
        assert!(
            lines[6].starts_with("  whitelist    1 addresses; overlaid tried=0 hit=0"),
            "{}",
            lines[6]
        );
        assert_eq!(
            lines[7],
            format!("  backend      {}", Backend::detect().name())
        );
        assert!(lines[8].starts_with("  per burst    "), "{}", lines[8]);
        assert!(lines[9].starts_with("  offsets      "), "{}", lines[9]);
        assert!(
            lines[10].starts_with("  configs      entered per pass: clean="),
            "{}",
            lines[10]
        );
        assert_eq!(
            *lines.last().unwrap(),
            "  decode time  0.50 s for 4 bursts (125000.0 us/burst)"
        );
        assert!(text.ends_with('\n'));
    }

    #[test]
    fn an_empty_log_reports_zero_time_per_burst() {
        let r = decode(corpus::RAW_MAGIC, Options::default(), false).unwrap();
        let text = report(&r, &track(&r.frames), 0.0);
        assert!(
            text.contains("  frames       0  (clean=0 1bit=0 2bit=0)  yield=0.0%\n"),
            "{text}"
        );
        assert!(
            text.ends_with("  decode time  0.00 s for 0 bursts (0.0 us/burst)\n"),
            "{text}"
        );
    }
}
