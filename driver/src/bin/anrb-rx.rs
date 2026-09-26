//! Live receiver.
//!
//! Three ways to run:
//!
//! * a dashboard, when stdout is a terminal;
//! * one line per frame otherwise, so piping into a file or another program
//!   still works;
//! * `--server`, which draws nothing, serves SBS and Beast until stopped, and
//!   logs a periodic status line, for running under systemd or Docker.

use anrb::beast::BeastServer;
use anrb::clock::hms;
use anrb::feed::{Event, LeaveReason};
use anrb::sbs::SbsServer;
use anrb::tracker::Tracker;
use anrb::tui::{FeedStat, Snapshot, Tui};
use anrb::{Options, RadarBox, Tick};
use std::io::IsTerminal;
use std::str::FromStr;
use std::time::{Duration, Instant};

const USAGE: &str = "\
anrb-rx - AirNav RadarBox live receiver

  --server              no dashboard, no per-frame output: serve and log.
                        Runs until stopped, and serves SBS on 30003 and Beast
                        on 30005 unless --sbs / --beast say otherwise.
  --status-every SECS   how often server mode writes a status line (default
                        60, 0 for never)
  --sbs PORT            serve BaseStation/SBS-1 on PORT
  --beast PORT          serve Mode-S Beast binary on PORT (no timestamps and
                        no signal level: this receiver has neither)
  --seconds N           stop after N seconds (default 120; server mode runs
                        until stopped unless this is given)
  --plain               one line per frame, even on a terminal
  --raw-log FILE        write the raw USB stream to FILE
  --no-soft             no soft-decision retry on a failed burst
  --2bit                also try blind two-bit corrections on DF11/DF17, not
                        guided by demodulator confidence
  -h, --help            show this help
";

/// How the run reports itself.
#[derive(PartialEq)]
enum Ui {
    /// The full-screen dashboard.
    Dashboard,
    /// One line per decoded frame.
    Lines,
    /// Nothing per frame; a status line on a timer and a line per event.
    Server,
}

/// Local wall clock, to the second - the same clock the SBS output uses, so a
/// line in the log and a line on the wire can be lined up.
fn stamp() -> String {
    let t = anrb::clock::local(std::time::SystemTime::now());
    format!("{:04}-{:02}-{:02} {:02}:{:02}:{:02}", t.year, t.month, t.day, t.hour, t.minute, t.second)
}

/// One stamped line. Line-buffered by `println!`, so journald and `tee` see it
/// as it happens rather than in four-kilobyte lumps.
fn log(msg: &str) {
    println!("{} {}", stamp(), msg);
}

/// Print `msg` and the usage text to stderr, and exit with status 2.
fn usage_error(msg: &str) -> ! {
    eprint!("anrb-rx: {msg}\n\n{USAGE}");
    std::process::exit(2);
}

/// The value that follows `flag` on the command line, parsed as a `T`.
fn value<T: FromStr>(args: &mut impl Iterator<Item = String>, flag: &str) -> T {
    let Some(v) = args.next() else { usage_error(&format!("{flag} needs a value")) };
    v.parse().unwrap_or_else(|_| usage_error(&format!("{flag}: invalid value {v:?}")))
}

fn main() {
    // Report a failure as a sentence rather than the Debug form of an io
    // error, which is what returning Result from main would print.
    if let Err(e) = run() {
        log(&format!("error: {e}"));
        std::process::exit(1);
    }
}

fn run() -> std::io::Result<()> {
    let mut secs: Option<u64> = None;
    let mut opts = Options::default();
    let mut raw_log: Option<String> = None;
    let mut sbs_port: Option<u16> = None;
    let mut beast_port: Option<u16> = None;
    let mut plain = false;
    let mut server = false;
    let mut status_every = 60u64;
    let mut args = std::env::args().skip(1);
    while let Some(a) = args.next() {
        match a.as_str() {
            "--seconds" => secs = Some(value(&mut args, &a)),
            "--raw-log" => raw_log = Some(value(&mut args, &a)),
            "--sbs" => sbs_port = Some(value(&mut args, &a)),
            "--beast" => beast_port = Some(value(&mut args, &a)),
            "--status-every" => status_every = value(&mut args, &a),
            "--no-soft" => opts.soft = false,
            "--2bit" => opts.blind2 = true,
            "--plain" => plain = true,
            "--server" => server = true,
            "-h" | "--help" => { print!("{USAGE}"); return Ok(()); }
            _ => usage_error(&format!("unknown argument {a:?}")),
        }
    }

    let tty = std::io::stdout().is_terminal();
    let ui = if server { Ui::Server }
             else if tty && !plain { Ui::Dashboard }
             else { Ui::Lines };
    // Server mode serves both feeds on their default ports unless told otherwise.
    if server && sbs_port.is_none() { sbs_port = Some(30003); }
    if server && beast_port.is_none() { beast_port = Some(30005); }
    // Interactive runs stop after 120 s unless --seconds is given; server mode
    // runs until stopped.
    let limit = secs.or(if server { None } else { Some(120) });

    let dashboard = ui == Ui::Dashboard;
    match ui {
        Ui::Dashboard => {}
        Ui::Lines => {
            println!("=== AirNav RadarBox ===");
            println!("decoder backend: {}", anrb::Backend::detect().name());
        }
        Ui::Server => {
            anrb::tui::catch_signals();
            log(&format!("starting; decoder backend {}", anrb::Backend::detect().name()));
        }
    }
    // The dashboard goes up before the device does, so the wait is visible
    // instead of being a blank terminal.
    let mut tui = if dashboard { Some(Tui::new()) } else { None };
    if let Some(t) = tui.as_mut() { t.status("opening the USB device"); }
    let mut rb = {
        let mut said = String::new();
        let mut show = |m: &str| {
            if let Some(t) = tui.as_mut() { t.status(m); }
            // Log each distinct step once, including the power-cycle fallback.
            else if server && m != said { said = m.to_string(); log(m); }
        };
        RadarBox::open_with(&mut show)?
    }.with_options(opts);
    if let Some(p) = &raw_log { rb = rb.record_to(p)?; }

    let mut sbs = sbs_port.map(SbsServer::bind).transpose()?;
    let mut beast = beast_port.map(BeastServer::bind).transpose()?;

    let mut frames = rb.frames();
    if let Some(n) = limit { frames = frames.until(Duration::from_secs(n)); }
    let mut tracker = Tracker::new();

    if server {
        let fw = frames.device().firmware().unwrap_or("?").to_string();
        log(&format!("open; firmware {} unlocked={} sbs={} beast={} limit={}",
                     fw, frames.device().is_unlocked(),
                     sbs_port.map(|p| p.to_string()).unwrap_or_else(|| "off".into()),
                     beast_port.map(|p| p.to_string()).unwrap_or_else(|| "off".into()),
                     limit.map(|n| format!("{n}s")).unwrap_or_else(|| "none".into())));
    }

    let started = Instant::now();
    // The previous tick's totals, so the display can show per-second rates.
    let mut last = Instant::now() - Duration::from_secs(1);   // draw immediately
    let (mut p_bursts, mut p_frames, mut p_bytes) = (0u64, 0u64, 0u64);
    let (mut d_bursts, mut d_frames, mut d_bytes);
    let mut bytes = 0u64;
    let mut n = 0usize;
    let mut now_ms = 0u32;          // receiver clock, taken from the frames

    // What the last server status line said, so only changes are reported.
    let mut was_unlocked = true;
    let mut was_relocks = 0u32;
    let mut was_failures = 0u32;
    let mut was_timeouts = 0u32;
    let mut record_failed = false;
    let mut last_status = Instant::now();

    loop {
        if anrb::tui::interrupted() { break; }
        match frames.poll()? {
            Tick::Done => break,
            Tick::Frame(f) => {
                n += 1;
                now_ms = f.ms;
                bytes += f.len() as u64;
                // Beast forwards every decoded frame as it arrives.
                if let Some(b) = beast.as_mut() { b.emit(&f); }
                let update = tracker.update(&f);
                if let Some(r) = tracker.refusal.take() {
                    let hex = |b: &[u8; 14]| b.iter().map(|x| format!("{x:02x}")).collect::<String>();
                    let other = r.other.map(|(c, b)| format!(", other half {c} ({})", hex(&b)))
                        .unwrap_or_default();
                    let line = format!(
                        "refused a position for {:06X}: {:.4},{:.4} is {:.1} km from {:.2},{:.2} where it \
was {:.1} s before ({}); bits repaired: this message {} ({}){}",
                        r.icao, r.lat, r.lon, r.metres / 1000.0, r.from.0, r.from.1,
                        r.after_ms as f64 / 1000.0,
                        if r.pair { "even/odd pair" } else { "one message against that position" },
                        r.this.0, hex(&r.this.1), other);
                    match tui.as_mut() {
                        Some(t) => t.push(line),
                        None => log(&line),
                    }
                }
                if let (Some(s), Some((icao, u))) = (sbs.as_mut(), update) {
                    if let Some(a) = tracker.table.get(&icao) {
                        s.emit(a, u, std::time::SystemTime::now());
                    }
                }
                let tc = f.type_code().map(|t| format!(" tc={t}")).unwrap_or_default();
                let fix = match f.corrected { 1 => " (1-bit)", 2 => " (2-bit)", _ => "" };
                let line = format!("{:7.2} {} DF{:<2} {:06X}{}{}",
                                   f.ms as f64 / 1000.0, f.hex(), f.df(),
                                   update.map_or(f.icao(), |(icao, _)| icao), tc, fix);
                match tui.as_mut() {
                    Some(t) => t.push(line),
                    // Server mode prints no per-frame lines.
                    None if server => {}
                    None => println!("[{line}]"),
                }
            }
            Tick::Idle => {}
        }

        if let Some(s) = sbs.as_mut() { s.poll(); }
        if let Some(b) = beast.as_mut() { b.poll(); }

        if last.elapsed() >= Duration::from_secs(1) {
            let st = frames.stats();
            d_bursts = st.bursts - p_bursts;
            d_frames = st.frames - p_frames;
            d_bytes  = bytes - p_bytes;
            p_bursts = st.bursts; p_frames = st.frames; p_bytes = bytes;
            last = Instant::now();
            // Drop aircraft not heard for 60 s.
            tracker.expire(now_ms, 60_000);

            // Drain feed events every second in every mode so the queue stays
            // bounded; only server mode logs them.
            let mut joins = Vec::new();
            if let Some(f) = sbs.as_mut() { joins.extend(f.take_events().into_iter().map(|e| ("sbs", e, f.clients()))); }
            if let Some(f) = beast.as_mut() { joins.extend(f.take_events().into_iter().map(|e| ("beast", e, f.clients()))); }
            if server {
                for (name, e, now) in joins {
                    log(&match e {
                        Event::Joined(a) => format!("{name} reader {a} joined ({now} now)"),
                        Event::Left(a, LeaveReason::Closed) => format!("{name} reader {a} left ({now} now)"),
                        Event::Left(a, LeaveReason::Stalled) => format!(
                            "{name} reader {a} stopped reading and was dropped, {} KiB behind ({now} now)",
                            anrb::feed::BACKLOG / 1024),
                    });
                }
            }

            if !record_failed {
                if let Some(e) = frames.record_error() {
                    let line = format!("recording stopped: {e}");
                    match tui.as_mut() {
                        Some(t) => t.push(line),
                        None => log(&line),
                    }
                    record_failed = true;
                }
            }

            let with_pos = tracker.table.values().filter(|a| a.lat.is_some()).count();
            let (unlocked, fw, pong, relocks, failures, timeouts) = {
                let d = frames.device();
                (d.is_unlocked(), d.firmware().map(str::to_owned),
                 d.since_pong(), d.relocks, d.relock_failures, d.pong_timeouts)
            };

            if server {
                // Log device state changes as they happen.
                if unlocked != was_unlocked {
                    log(if unlocked { "device unlocked" } else { "device LOCKED" });
                }
                // `relocks` counts attempts; whether one worked shows up as a
                // failure here or not at all.
                if relocks > was_relocks {
                    log(&format!("device re-locked; re-authenticating ({relocks} total)"));
                }
                if failures > was_failures {
                    log(&format!("re-authentication FAILED ({failures} total)"));
                }
                if timeouts > was_timeouts {
                    log(&format!("no PONG in time ({timeouts} total)"));
                }
                was_unlocked = unlocked; was_relocks = relocks; was_failures = failures;
                was_timeouts = timeouts;

                if status_every > 0 && last_status.elapsed() >= Duration::from_secs(status_every) {
                    last_status = Instant::now();
                    let st = frames.stats();
                    log(&format!(
                        "up={} bursts={} (+{}/s) frames={} (+{}/s) yield={:.1}% \
clean={} 1bit={} 2bit={} aircraft={} pos={} refused_pos={} whitelist={}/{} overlaid={}/{} sbs_clients={} sbs_lines={} sbs_dropped={} \
beast_clients={} beast_frames={} beast_dropped={} lock={} pong={}",
                        hms(started.elapsed()),
                        st.bursts, d_bursts, st.frames, d_frames,
                        st.yield_pct(),
                        st.clean, st.fixed1, st.fixed2,
                        tracker.len(), with_pos, tracker.implausible,
                        frames.decoder().whitelist_len(), frames.decoder().whitelist_evicted(),
                        st.overlaid_hit, st.overlaid_tried,
                        sbs.as_ref().map(|s| s.clients()).unwrap_or(0),
                        sbs.as_ref().map(|s| s.lines()).unwrap_or(0),
                        sbs.as_ref().map(|s| s.dropped()).unwrap_or(0),
                        beast.as_ref().map(|b| b.clients()).unwrap_or(0),
                        beast.as_ref().map(|b| b.frames()).unwrap_or(0),
                        beast.as_ref().map(|b| b.dropped()).unwrap_or(0),
                        if unlocked { "ok" } else { "locked" },
                        pong.map(|d| format!("{:.1}s", d.as_secs_f64()))
                            .unwrap_or_else(|| "-".into())));
                }
            }

            if let Some(t) = tui.as_mut() {
                let st = frames.stats();
                let with_cs = tracker.table.values().filter(|a| a.callsign.is_some()).count();
                t.draw(&Snapshot {
                    elapsed: t.uptime(),
                    unlocked, firmware: fw, since_pong: pong,
                    relocks, relock_failures: failures, pong_timeouts: timeouts,
                    bursts: st.bursts, frames: st.frames,
                    clean: st.clean, fixed1: st.fixed1, fixed2: st.fixed2,
                    d_bursts, d_frames, d_bytes,
                    aircraft: tracker.len(), with_pos, with_cs,
                    feeds: vec![
                        FeedStat { name: "SBS", port: sbs.as_ref().map(|s| s.port()), flag: "--sbs",
                                   clients: sbs.as_ref().map_or(0, |s| s.clients()),
                                   sent: sbs.as_ref().map_or(0, |s| s.lines()),
                                   dropped: sbs.as_ref().map_or(0, |s| s.dropped()),
                                   readers: sbs.as_ref().map(|s| s.readers()).unwrap_or_default() },
                        FeedStat { name: "Beast", port: beast.as_ref().map(|b| b.port()), flag: "--beast",
                                   clients: beast.as_ref().map_or(0, |b| b.clients()),
                                   sent: beast.as_ref().map_or(0, |b| b.frames()),
                                   dropped: beast.as_ref().map_or(0, |b| b.dropped()),
                                   readers: beast.as_ref().map(|b| b.readers()).unwrap_or_default() },
                    ],
                });
            }
        }
    }

    if let Some(t) = tui.as_mut() { t.restore(); }

    // End the session rather than leave the box streaming to nobody. Dropping
    // the device would do it too; doing it here means the outcome is reported.
    let lock = frames.device().lock();
    let lock_note = match &lock {
        Ok(l) if l.stopped => "locked the box; its stream stopped".to_string(),
        Ok(_) => "sent LOCK, but data was still arriving a second later".to_string(),
        Err(e) => format!("could not lock the box: {e}"),
    };

    let s = frames.stats();
    if server {
        log(&format!("stopping after {}; bursts={} frames={} (clean={}, 1bit={}, 2bit={}) \
yield={:.1}% aircraft={}{}",
                     hms(started.elapsed()), s.bursts, s.frames, s.clean, s.fixed1, s.fixed2,
                     s.yield_pct(),
                     tracker.len(),
                     sbs.as_ref().map(|sv| format!(" sbs_lines={}", sv.lines())).unwrap_or_default()
                     + &beast.as_ref().map(|b| format!(" beast_frames={}", b.frames())).unwrap_or_default()));
        log(&lock_note);
        return Ok(());
    }
    println!("{lock_note}");
    println!("=== bursts={} frames={} (clean={}, 1bit={}, 2bit={}) yield={:.1}% ===",
             s.bursts, s.frames, s.clean, s.fixed1, s.fixed2,
             s.yield_pct());
    println!("{n} frames delivered, {} aircraft tracked", tracker.len());
    if let Some(sv) = &sbs {
        println!("SBS on {}: {} lines to {} client(s)", sv.port(), sv.lines(), sv.clients());
    }
    if let Some(b) = &beast {
        println!("Beast on {}: {} frames to {} client(s)", b.port(), b.frames(), b.clients());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_stamp_is_the_shape_a_log_reader_expects() {
        let s = stamp();
        assert_eq!(s.len(), 19, "{s}");
        assert_eq!(&s[4..5], "-");
        assert_eq!(&s[10..11], " ");
        assert_eq!(&s[13..14], ":");
    }
}
