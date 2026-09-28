//! `anrb-replay` on generated logs: what it prints, what its flags change,
//! and how it exits.

mod common;

use anrb::corpus::Fingerprint;
use anrb::{Decoder, Options};
use common::*;
use std::path::{Path, PathBuf};
use std::process::{Command, Output};

const EVEN: &str = "8d4009da5833318e2bd82af8c6f5";
const ODD: &str = "8d4009da583324fef1cbc5c7449d";
const ADDR: u32 = 0x4CA2D6;

/// A directory of its own for each test, removed when dropped.
struct Scratch(PathBuf);

impl Scratch {
    fn new(name: &str) -> Scratch {
        let d =
            std::env::temp_dir().join(format!("anrb-replay-test-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(&d).unwrap();
        Scratch(d)
    }
}

impl Drop for Scratch {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.0);
    }
}

impl std::ops::Deref for Scratch {
    type Target = Path;
    fn deref(&self) -> &Path {
        &self.0
    }
}

fn replay(dir: &Path, args: &[&str]) -> Output {
    Command::new(env!("CARGO_BIN_EXE_anrb-replay"))
        .args(args)
        .current_dir(dir)
        .output()
        .expect("run anrb-replay")
}

fn stdout(o: &Output) -> String {
    String::from_utf8_lossy(&o.stdout).into_owned()
}
fn stderr(o: &Output) -> String {
    String::from_utf8_lossy(&o.stderr).into_owned()
}

/// The line of `out` that starts with `key` after the indent.
fn line<'a>(out: &'a str, key: &str) -> &'a str {
    out.lines()
        .find(|l| l.trim_start().starts_with(key))
        .unwrap_or_else(|| panic!("no {key} line in\n{out}"))
}

/// The bursts in the generated log, in order:
/// - two clean DF17 positions from 4009DA, in one transfer
/// - a PONG
/// - two clean DF11 replies from 4CA2D6 and a noise burst, in one transfer
/// - a DF17 with two low-confidence bits, which only the soft pass recovers
/// - a DF17 with two hard errors, which only the blind pass recovers
/// - a DF4 reply addressed to 4CA2D6, which only the overlaid pass recovers
fn bursts() -> Vec<(u32, Vec<u8>)> {
    let even = burst(&hex(EVEN));
    let odd = burst(&hex(ODD));
    let short = burst(&df11(ADDR));
    let mut soft = samples(&hex(EVEN));
    erase(&mut soft, 9);
    erase(&mut soft, 23);
    let mut hard = hex(ODD);
    flip(&mut hard, 20);
    flip(&mut hard, 60);
    vec![
        (100, transfer(&[&even, &odd])),
        (200, b"PONG".to_vec()),
        (300, transfer(&[&short, &short, &noise(1, 112)])),
        (400, pack(&soft)),
        (500, burst(&hard)),
        (600, burst(&df4(ADDR))),
    ]
}

fn write_log(dir: &Path) -> PathBuf {
    let b = bursts();
    let records: Vec<(u32, &[u8])> = b.iter().map(|(ms, d)| (*ms, &d[..])).collect();
    let p = dir.join("log.raw");
    std::fs::write(&p, raw_log(&records)).unwrap();
    p
}

/// The fingerprint line the library gives for the same log and options.
fn expected_fingerprint(opts: Options) -> String {
    let mut dec = Decoder::new();
    dec.opts = opts;
    let mut fp = Fingerprint::new();
    for (ms, d) in bursts() {
        for f in dec.feed(&d, ms) {
            fp.add(&f);
        }
    }
    fp.to_string()
}

#[test]
fn decodes_a_raw_log() {
    let dir = Scratch::new("default");
    let log = write_log(&dir);
    let o = replay(&dir, &[log.to_str().unwrap()]);
    assert_eq!(o.status.code(), Some(0), "{}", stderr(&o));
    let out = stdout(&o);
    assert_eq!(
        out.lines().next(),
        Some(expected_fingerprint(Options::default()).as_str())
    );
    assert!(out.starts_with("FINGERPRINT frames=5 "), "{out}");
    assert_eq!(line(&out, "bursts"), "  bursts       8 data + 1 PONG");
    assert!(
        line(&out, "frames").starts_with("  frames       5  (clean=4 1bit=0 2bit=1)  yield=62.5%")
    );
    assert_eq!(line(&out, "aircraft"), "  aircraft     2 distinct");
    assert_eq!(
        line(&out, "tracker"),
        "  tracker      1 placed; refused_pos=0"
    );
    assert!(line(&out, "soft").starts_with("  soft         1 hits from "));
    assert!(
        line(&out, "whitelist").starts_with("  whitelist    2 addresses; overlaid tried=0 hit=0")
    );
    assert!(line(&out, "backend").len() > "  backend      ".len());
    assert!(line(&out, "decode time").contains("for 8 bursts"));
    assert!(line(&out, "configs").contains("overlaid=0 blind=0"));
}

#[test]
fn flags_choose_the_passes() {
    let dir = Scratch::new("flags");
    let log = write_log(&dir);
    let log = log.to_str().unwrap();
    let cases: [(&[&str], Options, usize, &str); 4] = [
        (
            &["--no-soft"],
            Options {
                soft: false,
                blind2: false,
                overlaid: false,
            },
            4,
            "(clean=4 1bit=0 2bit=0)",
        ),
        (
            &["--2bit"],
            Options {
                soft: true,
                blind2: true,
                overlaid: false,
            },
            6,
            "(clean=4 1bit=0 2bit=2)",
        ),
        (
            &["--overlaid"],
            Options {
                soft: true,
                blind2: false,
                overlaid: true,
            },
            6,
            "(clean=5 1bit=0 2bit=1)",
        ),
        (
            &["--no-soft", "--2bit", "--overlaid"],
            Options {
                soft: false,
                blind2: true,
                overlaid: true,
            },
            7,
            "(clean=5 1bit=0 2bit=2)",
        ),
    ];
    for (flags, opts, frames, counts) in cases {
        // The file may come before or after the flags.
        let mut args = vec![log];
        args.extend_from_slice(flags);
        let o = replay(&dir, &args);
        assert_eq!(o.status.code(), Some(0), "{flags:?}: {}", stderr(&o));
        let out = stdout(&o);
        assert_eq!(
            out.lines().next(),
            Some(expected_fingerprint(opts).as_str()),
            "{flags:?}"
        );
        assert!(
            out.starts_with(&format!("FINGERPRINT frames={frames} ")),
            "{flags:?}\n{out}"
        );
        assert!(line(&out, "frames").contains(counts), "{flags:?}\n{out}");
    }
    let o = replay(&dir, &["--overlaid", log]);
    assert!(line(&stdout(&o), "whitelist").contains("hit=1"));
}

#[test]
fn reads_a_burst_file() {
    let dir = Scratch::new("bursts");
    let even = burst(&hex(EVEN));
    let short = burst(&df11(ADDR));
    let p = dir.join("bursts.bin");
    std::fs::write(&p, burst_file(&[&even, &noise(2, 112), &short])).unwrap();
    let o = replay(&dir, &["--bursts", p.to_str().unwrap()]);
    assert_eq!(o.status.code(), Some(0), "{}", stderr(&o));
    let out = stdout(&o);
    let mut fp = Fingerprint::new();
    let mut dec = Decoder::new();
    fp.add(&dec.decode_burst(&even, 0).unwrap());
    fp.add(&dec.decode_burst(&short, 0).unwrap());
    assert_eq!(out.lines().next(), Some(fp.to_string().as_str()));
    assert_eq!(line(&out, "bursts"), "  bursts       3 data + 0 PONG");
    assert_eq!(line(&out, "aircraft"), "  aircraft     2 distinct");
}

#[test]
fn help_is_printed_and_exits_zero() {
    let dir = Scratch::new("help");
    for flag in ["-h", "--help"] {
        let o = replay(&dir, &[flag]);
        assert_eq!(o.status.code(), Some(0));
        let out = stdout(&o);
        assert!(
            out.starts_with("anrb-replay - decode a recorded log offline"),
            "{out}"
        );
        for f in ["--no-soft", "--2bit", "--overlaid", "--bursts", "--help"] {
            assert!(out.contains(f), "{f} missing from help");
        }
        assert!(stderr(&o).is_empty());
    }
}

#[test]
fn bad_arguments_exit_two() {
    let dir = Scratch::new("args");
    let o = replay(&dir, &["--frobnicate"]);
    assert_eq!(o.status.code(), Some(2));
    let err = stderr(&o);
    assert!(
        err.starts_with("anrb-replay: unknown argument \"--frobnicate\""),
        "{err}"
    );
    assert!(
        err.contains("usage: anrb-replay"),
        "usage follows the error"
    );
    assert!(stdout(&o).is_empty());

    let o = replay(&dir, &["a.raw", "b.raw"]);
    assert_eq!(o.status.code(), Some(2));
    assert!(stderr(&o).starts_with("anrb-replay: more than one file: \"b.raw\""));
}

#[test]
fn unreadable_input_exits_one() {
    let dir = Scratch::new("input");
    // With no file named, the default capture path is used, relative to the
    // working directory.
    let o = replay(&dir, &[]);
    assert_eq!(o.status.code(), Some(1));
    assert!(
        stderr(&o).starts_with("captures/tuning_15min_v2.raw: "),
        "{}",
        stderr(&o)
    );

    let o = replay(&dir, &["missing.raw"]);
    assert_eq!(o.status.code(), Some(1));
    assert!(stderr(&o).starts_with("missing.raw: "));

    std::fs::write(dir.join("bad.raw"), b"NOTALOG1").unwrap();
    let o = replay(&dir, &["bad.raw"]);
    assert_eq!(o.status.code(), Some(1));
    assert_eq!(stderr(&o), "bad.raw: bad magic\n");
    assert!(stdout(&o).is_empty());
}
