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
use anrb::tracker::{Refusal, Tracker, Update};
use anrb::tui::{FeedStat, Snapshot, Tui};
use anrb::{Frame, Locked, Options, RadarBox, Stats, Tick};
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
  --probe HOST:PORT     exit 0 if HOST:PORT accepts a TCP connection, 1 if
                        not; the container image's health check
  -h, --help            show this help
";

/// How the run reports itself.
#[derive(Debug, PartialEq)]
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
    format!(
        "{:04}-{:02}-{:02} {:02}:{:02}:{:02}",
        t.year, t.month, t.day, t.hour, t.minute, t.second
    )
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

/// What the command line asks for.
struct Config {
    secs: Option<u64>,
    opts: Options,
    raw_log: Option<String>,
    sbs_port: Option<u16>,
    beast_port: Option<u16>,
    plain: bool,
    server: bool,
    status_every: u64,
}

/// The outcome of reading the command line.
enum Command {
    Run(Config),
    Help,
    /// Check that something listens at this `host:port`, and do nothing else.
    Probe(String),
}

/// The value that follows `flag` on the command line, parsed as a `T`.
fn value<T: FromStr>(args: &mut impl Iterator<Item = String>, flag: &str) -> Result<T, String> {
    let Some(v) = args.next() else {
        return Err(format!("{flag} needs a value"));
    };
    v.parse()
        .map_err(|_| format!("{flag}: invalid value {v:?}"))
}

/// Read the arguments that follow the program name. An error is the message
/// to show above the usage text.
fn parse_args(args: impl IntoIterator<Item = String>) -> Result<Command, String> {
    let mut c = Config {
        secs: None,
        opts: Options::default(),
        raw_log: None,
        sbs_port: None,
        beast_port: None,
        plain: false,
        server: false,
        status_every: 60,
    };
    let mut args = args.into_iter();
    while let Some(a) = args.next() {
        match a.as_str() {
            "--seconds" => c.secs = Some(value(&mut args, &a)?),
            "--raw-log" => c.raw_log = Some(value(&mut args, &a)?),
            "--sbs" => c.sbs_port = Some(value(&mut args, &a)?),
            "--beast" => c.beast_port = Some(value(&mut args, &a)?),
            "--status-every" => c.status_every = value(&mut args, &a)?,
            "--no-soft" => c.opts.soft = false,
            "--2bit" => c.opts.blind2 = true,
            "--plain" => c.plain = true,
            "--server" => c.server = true,
            "--probe" => return Ok(Command::Probe(value(&mut args, &a)?)),
            "-h" | "--help" => return Ok(Command::Help),
            _ => return Err(format!("unknown argument {a:?}")),
        }
    }
    Ok(Command::Run(c))
}

/// The mode, the ports to serve and the time limit, from the command line and
/// whether stdout is a terminal.
#[derive(Debug, PartialEq)]
struct Plan {
    ui: Ui,
    sbs_port: Option<u16>,
    beast_port: Option<u16>,
    /// Seconds to run, or None to run until stopped.
    limit: Option<u64>,
}

fn plan(c: &Config, tty: bool) -> Plan {
    let ui = if c.server {
        Ui::Server
    } else if tty && !c.plain {
        Ui::Dashboard
    } else {
        Ui::Lines
    };
    // Server mode serves both feeds on their default ports unless told otherwise.
    let sbs_port = c.sbs_port.or(if c.server { Some(30003) } else { None });
    let beast_port = c.beast_port.or(if c.server { Some(30005) } else { None });
    // Interactive runs stop after 120 s unless --seconds is given; server mode
    // runs until stopped.
    let limit = c.secs.or(if c.server { None } else { Some(120) });
    Plan {
        ui,
        sbs_port,
        beast_port,
        limit,
    }
}

/// The server log line once the device is open.
fn open_line(
    firmware: &str,
    unlocked: bool,
    sbs_port: Option<u16>,
    beast_port: Option<u16>,
    limit: Option<u64>,
) -> String {
    format!(
        "open; firmware {} unlocked={} sbs={} beast={} limit={}",
        firmware,
        unlocked,
        sbs_port
            .map(|p| p.to_string())
            .unwrap_or_else(|| "off".into()),
        beast_port
            .map(|p| p.to_string())
            .unwrap_or_else(|| "off".into()),
        limit
            .map(|n| format!("{n}s"))
            .unwrap_or_else(|| "none".into())
    )
}

/// A position the tracker refused, and the messages it came from.
fn refusal_line(r: &Refusal) -> String {
    let hex = |b: &[u8; 14]| b.iter().map(|x| format!("{x:02x}")).collect::<String>();
    let other = r
        .other
        .map(|(c, b)| format!(", other half {c} ({})", hex(&b)))
        .unwrap_or_default();
    format!(
        "refused a position for {:06X}: {:.4},{:.4} is {:.1} km from {:.2},{:.2} where it \
was {:.1} s before ({}); bits repaired: this message {} ({}){}",
        r.icao,
        r.lat,
        r.lon,
        r.metres / 1000.0,
        r.from.0,
        r.from.1,
        r.after_ms as f64 / 1000.0,
        if r.pair {
            "even/odd pair"
        } else {
            "one message against that position"
        },
        r.this.0,
        hex(&r.this.1),
        other
    )
}

/// One decoded frame: receiver time, bytes, format, address, type code and
/// how many bits were repaired. The address is the tracker's when it has one.
fn frame_line(f: &Frame, update: Option<(u32, Update)>) -> String {
    let tc = f
        .type_code()
        .map(|t| format!(" tc={t}"))
        .unwrap_or_default();
    let fix = match f.corrected {
        1 => " (1-bit)",
        2 => " (2-bit)",
        _ => "",
    };
    format!(
        "{:7.2} {} DF{:<2} {:06X}{}{}",
        f.ms as f64 / 1000.0,
        f.hex(),
        f.df(),
        update.map_or(f.icao(), |(icao, _)| icao),
        tc,
        fix
    )
}

/// A reader joining or leaving the feed called `name`; `now` is how many
/// readers the feed has after it.
fn event_line(name: &str, e: &Event, now: usize) -> String {
    match e {
        Event::Joined(a) => format!("{name} reader {a} joined ({now} now)"),
        Event::Left(a, LeaveReason::Closed) => format!("{name} reader {a} left ({now} now)"),
        Event::Left(a, LeaveReason::Stalled) => format!(
            "{name} reader {a} stopped reading and was dropped, {} KiB behind ({now} now)",
            anrb::feed::BACKLOG / 1024
        ),
    }
}

/// Hand one decoded frame to the tracker and to the feeds being served.
/// Returns the line for a position the tracker refused, if it refused one,
/// and the frame's own line.
fn handle_frame(
    f: &Frame,
    tracker: &mut Tracker,
    sbs: Option<&mut SbsServer>,
    beast: Option<&mut BeastServer>,
) -> (Option<String>, String) {
    // Beast forwards every decoded frame as it arrives.
    if let Some(b) = beast {
        b.emit(f);
    }
    let update = tracker.update(f);
    let refusal = tracker.refusal.take().map(|r| refusal_line(&r));
    if let (Some(s), Some((icao, u))) = (sbs, update) {
        if let Some(a) = tracker.table.get(&icao) {
            s.emit(a, u, std::time::SystemTime::now());
        }
    }
    (refusal, frame_line(f, update))
}

/// Take the join and leave events of the feeds being served, as log lines.
/// Taking them keeps the queues bounded whether or not they are logged.
fn feed_events(sbs: Option<&mut SbsServer>, beast: Option<&mut BeastServer>) -> Vec<String> {
    let mut lines = Vec::new();
    if let Some(f) = sbs {
        lines.extend(
            f.take_events()
                .iter()
                .map(|e| event_line("sbs", e, f.clients())),
        );
    }
    if let Some(f) = beast {
        lines.extend(
            f.take_events()
                .iter()
                .map(|e| event_line("beast", e, f.clients())),
        );
    }
    lines
}

/// Both feeds as the dashboard shows them, served or not.
fn feed_stats(sbs: Option<&SbsServer>, beast: Option<&BeastServer>) -> Vec<FeedStat> {
    let (s, b) = (FeedCounts::sbs(sbs), FeedCounts::beast(beast));
    vec![
        FeedStat {
            name: "SBS",
            port: sbs.map(|s| s.port()),
            flag: "--sbs",
            clients: s.clients,
            sent: s.sent,
            dropped: s.dropped,
            readers: sbs.map(|s| s.readers()).unwrap_or_default(),
        },
        FeedStat {
            name: "Beast",
            port: beast.map(|b| b.port()),
            flag: "--beast",
            clients: b.clients,
            sent: b.sent,
            dropped: b.dropped,
            readers: beast.map(|b| b.readers()).unwrap_or_default(),
        },
    ]
}

/// Per-second rates from running totals: what each total was at the previous
/// step.
#[derive(Default)]
struct Rates {
    bursts: u64,
    frames: u64,
    bytes: u64,
}

impl Rates {
    /// The increase in each total since the previous call.
    fn step(&mut self, bursts: u64, frames: u64, bytes: u64) -> (u64, u64, u64) {
        let d = (
            bursts - self.bursts,
            frames - self.frames,
            bytes - self.bytes,
        );
        *self = Rates {
            bursts,
            frames,
            bytes,
        };
        d
    }
}

/// The device's lock state and its counters, as last logged.
#[derive(Clone, Copy)]
struct DeviceState {
    unlocked: bool,
    relocks: u32,
    failures: u32,
    timeouts: u32,
}

impl DeviceState {
    /// What server mode assumes before the first report: unlocked, nothing
    /// counted.
    fn new() -> DeviceState {
        DeviceState {
            unlocked: true,
            relocks: 0,
            failures: 0,
            timeouts: 0,
        }
    }

    /// A log line for each change from `self` to `now`. `relocks` counts
    /// attempts; whether one worked shows up as a failure here or not at all.
    fn changes(&self, now: &DeviceState) -> Vec<String> {
        let mut out = Vec::new();
        if now.unlocked != self.unlocked {
            out.push(
                if now.unlocked {
                    "device unlocked"
                } else {
                    "device LOCKED"
                }
                .to_string(),
            );
        }
        if now.relocks > self.relocks {
            out.push(format!(
                "device re-locked; re-authenticating ({} total)",
                now.relocks
            ));
        }
        if now.failures > self.failures {
            out.push(format!("re-authentication FAILED ({} total)", now.failures));
        }
        if now.timeouts > self.timeouts {
            out.push(format!("no PONG in time ({} total)", now.timeouts));
        }
        out
    }
}

/// A feed server's counters, all zero when the feed is off.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct FeedCounts {
    clients: usize,
    sent: u64,
    dropped: u64,
}

impl FeedCounts {
    fn sbs(s: Option<&SbsServer>) -> FeedCounts {
        s.map_or_else(FeedCounts::default, |s| FeedCounts {
            clients: s.clients(),
            sent: s.lines(),
            dropped: s.dropped(),
        })
    }

    fn beast(b: Option<&BeastServer>) -> FeedCounts {
        b.map_or_else(FeedCounts::default, |b| FeedCounts {
            clients: b.clients(),
            sent: b.frames(),
            dropped: b.dropped(),
        })
    }
}

/// Everything in server mode's periodic status line.
struct Status {
    up: Duration,
    st: Stats,
    d_bursts: u64,
    d_frames: u64,
    aircraft: usize,
    with_pos: usize,
    refused: u64,
    whitelist: usize,
    evicted: u64,
    sbs: FeedCounts,
    beast: FeedCounts,
    unlocked: bool,
    pong: Option<Duration>,
}

impl Status {
    fn line(&self) -> String {
        let st = &self.st;
        format!(
            "up={} bursts={} (+{}/s) frames={} (+{}/s) yield={:.1}% \
clean={} 1bit={} 2bit={} aircraft={} pos={} refused_pos={} whitelist={}/{} overlaid={}/{} sbs_clients={} sbs_lines={} sbs_dropped={} \
beast_clients={} beast_frames={} beast_dropped={} lock={} pong={}",
            hms(self.up),
            st.bursts, self.d_bursts, st.frames, self.d_frames,
            st.yield_pct(),
            st.clean, st.fixed1, st.fixed2,
            self.aircraft, self.with_pos, self.refused,
            self.whitelist, self.evicted,
            st.overlaid_hit, st.overlaid_tried,
            self.sbs.clients, self.sbs.sent, self.sbs.dropped,
            self.beast.clients, self.beast.sent, self.beast.dropped,
            if self.unlocked { "ok" } else { "locked" },
            self.pong.map(|d| format!("{:.1}s", d.as_secs_f64()))
                .unwrap_or_else(|| "-".into()))
    }
}

/// What came of locking the box at the end of a run.
fn lock_note(lock: &std::io::Result<Locked>) -> String {
    match lock {
        Ok(l) if l.stopped => "locked the box; its stream stopped".to_string(),
        Ok(_) => "sent LOCK, but data was still arriving a second later".to_string(),
        Err(e) => format!("could not lock the box: {e}"),
    }
}

/// Server mode's last status line. The feed totals are there only for the
/// feeds that were served.
fn stop_line(
    up: Duration,
    s: &Stats,
    aircraft: usize,
    sbs_lines: Option<u64>,
    beast_frames: Option<u64>,
) -> String {
    format!(
        "stopping after {}; bursts={} frames={} (clean={}, 1bit={}, 2bit={}) \
yield={:.1}% aircraft={}{}",
        hms(up),
        s.bursts,
        s.frames,
        s.clean,
        s.fixed1,
        s.fixed2,
        s.yield_pct(),
        aircraft,
        sbs_lines
            .map(|n| format!(" sbs_lines={n}"))
            .unwrap_or_default()
            + &beast_frames
                .map(|n| format!(" beast_frames={n}"))
                .unwrap_or_default()
    )
}

/// The totals printed after an interactive run, one line each, with the port
/// and counters of each feed that was served.
fn summary(
    s: &Stats,
    n: usize,
    aircraft: usize,
    sbs: Option<(u16, FeedCounts)>,
    beast: Option<(u16, FeedCounts)>,
) -> Vec<String> {
    let mut out = vec![
        format!(
            "=== bursts={} frames={} (clean={}, 1bit={}, 2bit={}) yield={:.1}% ===",
            s.bursts,
            s.frames,
            s.clean,
            s.fixed1,
            s.fixed2,
            s.yield_pct()
        ),
        format!("{n} frames delivered, {aircraft} aircraft tracked"),
    ];
    if let Some((port, c)) = sbs {
        out.push(format!(
            "SBS on {port}: {} lines to {} client(s)",
            c.sent, c.clients
        ));
    }
    if let Some((port, c)) = beast {
        out.push(format!(
            "Beast on {port}: {} frames to {} client(s)",
            c.sent, c.clients
        ));
    }
    out
}

fn main() {
    let cfg = match parse_args(std::env::args().skip(1)) {
        Ok(Command::Run(cfg)) => cfg,
        Ok(Command::Help) => {
            print!("{USAGE}");
            return;
        }
        Ok(Command::Probe(addr)) => {
            std::process::exit(if anrb::feed::probe(&addr, Duration::from_secs(3)) {
                0
            } else {
                1
            })
        }
        Err(msg) => usage_error(&msg),
    };
    // Report a failure as a sentence rather than the Debug form of an io
    // error, which is what returning Result from main would print.
    if let Err(e) = run(cfg) {
        log(&format!("error: {e}"));
        std::process::exit(1);
    }
}

fn run(cfg: Config) -> std::io::Result<()> {
    let Plan {
        ui,
        sbs_port,
        beast_port,
        limit,
    } = plan(&cfg, std::io::stdout().is_terminal());
    let server = ui == Ui::Server;
    let status_every = cfg.status_every;

    let dashboard = ui == Ui::Dashboard;
    match ui {
        Ui::Dashboard => {}
        Ui::Lines => {
            println!("=== AirNav RadarBox ===");
            println!("decoder backend: {}", anrb::Backend::detect().name());
        }
        Ui::Server => {
            anrb::tui::catch_signals();
            log(&format!(
                "starting; decoder backend {}",
                anrb::Backend::detect().name()
            ));
        }
    }
    // The dashboard goes up before the device does, so the wait is visible
    // instead of being a blank terminal.
    let mut tui = if dashboard { Some(Tui::new()) } else { None };
    if let Some(t) = tui.as_mut() {
        t.status("opening the USB device");
    }
    let mut rb = {
        let mut said = String::new();
        let mut show = |m: &str| {
            if let Some(t) = tui.as_mut() {
                t.status(m);
            }
            // Log each distinct step once, including the power-cycle fallback.
            else if server && m != said {
                said = m.to_string();
                log(m);
            }
        };
        RadarBox::open_with(&mut show)?
    }
    .with_options(cfg.opts);
    if let Some(p) = &cfg.raw_log {
        rb = rb.record_to(p)?;
    }

    let mut sbs = sbs_port.map(SbsServer::bind).transpose()?;
    let mut beast = beast_port.map(BeastServer::bind).transpose()?;

    let mut frames = rb.frames();
    if let Some(n) = limit {
        frames = frames.until(Duration::from_secs(n));
    }
    let mut tracker = Tracker::new();

    if server {
        let fw = frames.device().firmware().unwrap_or("?").to_string();
        let unlocked = frames.device().is_unlocked();
        log(&open_line(&fw, unlocked, sbs_port, beast_port, limit));
    }

    let started = Instant::now();
    // The previous tick's totals, so the display can show per-second rates.
    let mut last = Instant::now() - Duration::from_secs(1); // draw immediately
    let mut rates = Rates::default();
    let mut bytes = 0u64;
    let mut n = 0usize;
    let mut now_ms = 0u32; // receiver clock, taken from the frames

    // What the last server status line said, so only changes are reported.
    let mut was = DeviceState::new();
    let mut record_failed = false;
    let mut last_status = Instant::now();

    loop {
        if anrb::tui::interrupted() {
            break;
        }
        match frames.poll()? {
            Tick::Done => break,
            Tick::Frame(f) => {
                n += 1;
                now_ms = f.ms;
                bytes += f.len() as u64;
                let (refusal, line) = handle_frame(&f, &mut tracker, sbs.as_mut(), beast.as_mut());
                if let Some(line) = refusal {
                    match tui.as_mut() {
                        Some(t) => t.push(line),
                        None => log(&line),
                    }
                }
                match tui.as_mut() {
                    Some(t) => t.push(line),
                    // Server mode prints no per-frame lines.
                    None if server => {}
                    None => println!("[{line}]"),
                }
            }
            Tick::Idle => {}
        }

        if let Some(s) = sbs.as_mut() {
            s.poll();
        }
        if let Some(b) = beast.as_mut() {
            b.poll();
        }

        if last.elapsed() >= Duration::from_secs(1) {
            let st = frames.stats();
            let (d_bursts, d_frames, d_bytes) = rates.step(st.bursts, st.frames, bytes);
            last = Instant::now();
            // Drop aircraft not heard for 60 s.
            tracker.expire(now_ms, 60_000);

            // Drain feed events every second in every mode so the queue stays
            // bounded; only server mode logs them.
            let events = feed_events(sbs.as_mut(), beast.as_mut());
            if server {
                for line in events {
                    log(&line);
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
                (
                    d.is_unlocked(),
                    d.firmware().map(str::to_owned),
                    d.since_pong(),
                    d.relocks,
                    d.relock_failures,
                    d.pong_timeouts,
                )
            };

            if server {
                // Log device state changes as they happen.
                let now = DeviceState {
                    unlocked,
                    relocks,
                    failures,
                    timeouts,
                };
                for line in was.changes(&now) {
                    log(&line);
                }
                was = now;

                if status_every > 0 && last_status.elapsed() >= Duration::from_secs(status_every) {
                    last_status = Instant::now();
                    log(&Status {
                        up: started.elapsed(),
                        st: frames.stats(),
                        d_bursts,
                        d_frames,
                        aircraft: tracker.len(),
                        with_pos,
                        refused: tracker.implausible,
                        whitelist: frames.decoder().whitelist_len(),
                        evicted: frames.decoder().whitelist_evicted(),
                        sbs: FeedCounts::sbs(sbs.as_ref()),
                        beast: FeedCounts::beast(beast.as_ref()),
                        unlocked,
                        pong,
                    }
                    .line());
                }
            }

            if let Some(t) = tui.as_mut() {
                let st = frames.stats();
                let with_cs = tracker
                    .table
                    .values()
                    .filter(|a| a.callsign.is_some())
                    .count();
                t.draw(&Snapshot {
                    elapsed: t.uptime(),
                    unlocked,
                    firmware: fw,
                    since_pong: pong,
                    relocks,
                    relock_failures: failures,
                    pong_timeouts: timeouts,
                    bursts: st.bursts,
                    frames: st.frames,
                    clean: st.clean,
                    fixed1: st.fixed1,
                    fixed2: st.fixed2,
                    d_bursts,
                    d_frames,
                    d_bytes,
                    aircraft: tracker.len(),
                    with_pos,
                    with_cs,
                    feeds: feed_stats(sbs.as_ref(), beast.as_ref()),
                });
            }
        }
    }

    if let Some(t) = tui.as_mut() {
        t.restore();
    }

    // End the session rather than leave the box streaming to nobody. Dropping
    // the device would do it too; doing it here means the outcome is reported.
    let note = lock_note(&frames.device().lock());

    let s = frames.stats();
    if server {
        log(&stop_line(
            started.elapsed(),
            &s,
            tracker.len(),
            sbs.as_ref().map(|sv| sv.lines()),
            beast.as_ref().map(|b| b.frames()),
        ));
        log(&note);
        return Ok(());
    }
    println!("{note}");
    let feed = |port: Option<u16>, c: FeedCounts| port.map(|p| (p, c));
    for line in summary(
        &s,
        n,
        tracker.len(),
        feed(
            sbs.as_ref().map(|s| s.port()),
            FeedCounts::sbs(sbs.as_ref()),
        ),
        feed(
            beast.as_ref().map(|b| b.port()),
            FeedCounts::beast(beast.as_ref()),
        ),
    ) {
        println!("{line}");
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{BufRead, BufReader, Read};
    use std::net::TcpStream;

    #[test]
    fn the_stamp_is_the_shape_a_log_reader_expects() {
        let s = stamp();
        assert_eq!(s.len(), 19, "{s}");
        assert_eq!(&s[4..5], "-");
        assert_eq!(&s[10..11], " ");
        assert_eq!(&s[13..14], ":");
    }

    fn args(a: &[&str]) -> Result<Command, String> {
        parse_args(a.iter().map(|s| s.to_string()))
    }

    fn config(a: &[&str]) -> Config {
        match args(a) {
            Ok(Command::Run(c)) => c,
            Ok(Command::Help) => panic!("{a:?} asked for help"),
            Ok(Command::Probe(p)) => panic!("{a:?} asked for a probe of {p}"),
            Err(e) => panic!("{a:?}: {e}"),
        }
    }

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
            .collect()
    }

    fn bytes14(s: &str) -> [u8; 14] {
        hex(s).try_into().unwrap()
    }

    // An even airborne position of 4009DA, from the tracker's tests.
    const EVEN: &str = "8d4009da5833318e2bd82af8c6f5";
    const ODD: &str = "8d4009da583324fef1cbc5c7449d";

    #[test]
    fn no_arguments_give_the_defaults() {
        let c = config(&[]);
        assert_eq!(c.secs, None);
        assert!(c.opts.soft && !c.opts.blind2);
        assert_eq!(c.raw_log, None);
        assert_eq!((c.sbs_port, c.beast_port), (None, None));
        assert!(!c.plain && !c.server);
        assert_eq!(c.status_every, 60);
    }

    #[test]
    fn every_flag_is_read() {
        let c = config(&[
            "--seconds",
            "30",
            "--raw-log",
            "out.raw",
            "--sbs",
            "4003",
            "--beast",
            "4005",
            "--status-every",
            "0",
            "--no-soft",
            "--2bit",
            "--plain",
            "--server",
        ]);
        assert_eq!(c.secs, Some(30));
        assert_eq!(c.raw_log.as_deref(), Some("out.raw"));
        assert_eq!((c.sbs_port, c.beast_port), (Some(4003), Some(4005)));
        assert_eq!(c.status_every, 0);
        assert!(!c.opts.soft && c.opts.blind2);
        assert!(c.plain && c.server);
    }

    #[test]
    fn a_probe_takes_an_address_and_nothing_else_runs() {
        assert!(matches!(
            args(&["--server", "--probe", "127.0.0.1:30005"]),
            Ok(Command::Probe(a)) if a == "127.0.0.1:30005"
        ));
        assert_eq!(args(&["--probe"]).err().unwrap(), "--probe needs a value");
    }

    #[test]
    fn help_is_asked_for_with_either_spelling() {
        assert!(matches!(args(&["-h"]), Ok(Command::Help)));
        assert!(matches!(
            args(&["--plain", "--help", "--bogus"]),
            Ok(Command::Help)
        ));
    }

    #[test]
    fn bad_arguments_say_what_is_wrong() {
        assert_eq!(
            args(&["--bogus"]).err().unwrap(),
            "unknown argument \"--bogus\""
        );
        assert_eq!(args(&["--sbs"]).err().unwrap(), "--sbs needs a value");
        assert_eq!(
            args(&["--sbs", "70000"]).err().unwrap(),
            "--sbs: invalid value \"70000\""
        );
        assert_eq!(
            args(&["--seconds", "-1"]).err().unwrap(),
            "--seconds: invalid value \"-1\""
        );
        assert_eq!(
            args(&["--status-every", "x"]).err().unwrap(),
            "--status-every: invalid value \"x\""
        );
    }

    #[test]
    fn a_terminal_gets_the_dashboard_and_stops_after_two_minutes() {
        let p = plan(&config(&[]), true);
        assert_eq!(
            p,
            Plan {
                ui: Ui::Dashboard,
                sbs_port: None,
                beast_port: None,
                limit: Some(120)
            }
        );
    }

    #[test]
    fn a_pipe_or_plain_gets_lines() {
        assert_eq!(plan(&config(&[]), false).ui, Ui::Lines);
        let p = plan(
            &config(&["--plain", "--sbs", "4003", "--seconds", "5"]),
            true,
        );
        assert_eq!(
            p,
            Plan {
                ui: Ui::Lines,
                sbs_port: Some(4003),
                beast_port: None,
                limit: Some(5)
            }
        );
    }

    #[test]
    fn server_mode_serves_both_feeds_and_runs_until_stopped() {
        let p = plan(&config(&["--server"]), true);
        assert_eq!(
            p,
            Plan {
                ui: Ui::Server,
                sbs_port: Some(30003),
                beast_port: Some(30005),
                limit: None
            }
        );
        let p = plan(
            &config(&["--server", "--beast", "1", "--seconds", "9"]),
            false,
        );
        assert_eq!(
            p,
            Plan {
                ui: Ui::Server,
                sbs_port: Some(30003),
                beast_port: Some(1),
                limit: Some(9)
            }
        );
    }

    #[test]
    fn the_open_line_names_the_ports_and_the_limit() {
        assert_eq!(
            open_line("1.2", true, Some(30003), Some(30005), None),
            "open; firmware 1.2 unlocked=true sbs=30003 beast=30005 limit=none"
        );
        assert_eq!(
            open_line("?", false, None, None, Some(60)),
            "open; firmware ? unlocked=false sbs=off beast=off limit=60s"
        );
    }

    #[test]
    fn a_refused_pair_names_both_halves() {
        let r = Refusal {
            icao: 0x4009DA,
            lat: 51.123456,
            lon: 9.5,
            from: (50.33265, 8.73717),
            metres: 123_456.0,
            after_ms: 2500,
            pair: true,
            this: (1, bytes14(EVEN)),
            other: Some((0, bytes14(ODD))),
        };
        assert_eq!(refusal_line(&r),
                   format!("refused a position for 4009DA: 51.1235,9.5000 is 123.5 km from 50.33,8.74 \
                            where it was 2.5 s before (even/odd pair); bits repaired: this message 1 \
                            ({EVEN}), other half 0 ({ODD})"));
    }

    #[test]
    fn a_refused_single_message_has_no_other_half() {
        let r = Refusal {
            icao: 0xABC,
            lat: -1.0,
            lon: -2.0,
            from: (0.0, 0.0),
            metres: 50.0,
            after_ms: 0,
            pair: false,
            this: (0, bytes14(EVEN)),
            other: None,
        };
        assert_eq!(
            refusal_line(&r),
            format!(
                "refused a position for 000ABC: -1.0000,-2.0000 is 0.1 km from 0.00,0.00 \
                            where it was 0.0 s before (one message against that position); bits \
                            repaired: this message 0 ({EVEN})"
            )
        );
    }

    #[test]
    fn a_frame_line_shows_time_format_address_type_and_repair() {
        let mut f = Frame::new(&hex(EVEN), 1234).unwrap();
        assert_eq!(
            frame_line(&f, None),
            format!("   1.23 {EVEN} DF17 4009DA tc=11")
        );
        f.corrected = 1;
        assert_eq!(
            frame_line(&f, None),
            format!("   1.23 {EVEN} DF17 4009DA tc=11 (1-bit)")
        );
        f.corrected = 2;
        assert_eq!(
            frame_line(&f, None),
            format!("   1.23 {EVEN} DF17 4009DA tc=11 (2-bit)")
        );
    }

    #[test]
    fn a_short_frame_has_no_type_code_and_takes_the_trackers_address() {
        // DF4, an altitude reply: its address is overlaid on the parity, so
        // the tracker's answer is the one to show.
        let f = Frame::new(&hex("20001838ca3e51"), 123_456).unwrap();
        assert_eq!(
            frame_line(&f, Some((0x3C6DD0, Update::Address))),
            " 123.46 20001838ca3e51 DF4  3C6DD0"
        );
    }

    #[test]
    fn feed_events_read_as_sentences() {
        let a: std::net::SocketAddr = "192.0.2.7:5000".parse().unwrap();
        assert_eq!(
            event_line("sbs", &Event::Joined(a), 1),
            "sbs reader 192.0.2.7:5000 joined (1 now)"
        );
        assert_eq!(
            event_line("beast", &Event::Left(a, LeaveReason::Closed), 0),
            "beast reader 192.0.2.7:5000 left (0 now)"
        );
        assert_eq!(
            event_line("sbs", &Event::Left(a, LeaveReason::Stalled), 2),
            "sbs reader 192.0.2.7:5000 stopped reading and was dropped, 256 KiB behind (2 now)"
        );
    }

    #[test]
    fn rates_are_the_increase_since_the_last_step() {
        let mut r = Rates::default();
        assert_eq!(r.step(10, 4, 56), (10, 4, 56));
        assert_eq!(r.step(25, 4, 70), (15, 0, 14));
        assert_eq!(r.step(25, 4, 70), (0, 0, 0));
    }

    #[test]
    fn only_device_changes_are_logged() {
        let start = DeviceState::new();
        assert!(start.changes(&start).is_empty());

        let locked = DeviceState {
            unlocked: false,
            relocks: 1,
            failures: 1,
            timeouts: 2,
        };
        assert_eq!(
            start.changes(&locked),
            vec![
                "device LOCKED".to_string(),
                "device re-locked; re-authenticating (1 total)".to_string(),
                "re-authentication FAILED (1 total)".to_string(),
                "no PONG in time (2 total)".to_string(),
            ]
        );
        assert!(locked.changes(&locked).is_empty());

        let back = DeviceState {
            unlocked: true,
            ..locked
        };
        assert_eq!(locked.changes(&back), vec!["device unlocked".to_string()]);
    }

    #[test]
    fn feed_counts_are_zero_for_a_feed_that_is_off() {
        assert_eq!(FeedCounts::sbs(None), FeedCounts::default());
        assert_eq!(FeedCounts::beast(None), FeedCounts::default());
    }

    /// Poll a feed server until `poll` says it has seen what the test waits
    /// for, or fail after five seconds.
    fn poll_until(mut poll: impl FnMut() -> bool) {
        let deadline = Instant::now() + Duration::from_secs(5);
        while !poll() {
            assert!(
                Instant::now() < deadline,
                "the feed server did not get there in time"
            );
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    /// Both feed servers on ports of their own, each with one reader
    /// attached.
    fn served() -> (SbsServer, TcpStream, BeastServer, TcpStream) {
        let mut sbs = SbsServer::bind(0).unwrap();
        let mut beast = BeastServer::bind(0).unwrap();
        let s = TcpStream::connect(("127.0.0.1", sbs.port())).unwrap();
        let b = TcpStream::connect(("127.0.0.1", beast.port())).unwrap();
        for c in [&s, &b] {
            c.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
        }
        poll_until(|| {
            sbs.poll();
            sbs.clients() == 1
        });
        poll_until(|| {
            beast.poll();
            beast.clients() == 1
        });
        (sbs, s, beast, b)
    }

    #[test]
    fn a_frame_goes_to_the_tracker_and_to_both_feeds() {
        let (mut sbs, s, mut beast, mut b) = served();
        let mut tracker = Tracker::new();
        let f = Frame::new(&hex(EVEN), 1000).unwrap();
        let (refusal, line) = handle_frame(&f, &mut tracker, Some(&mut sbs), Some(&mut beast));
        assert_eq!(refusal, None);
        assert_eq!(line, format!("   1.00 {EVEN} DF17 4009DA tc=11"));
        assert_eq!(tracker.table[&0x4009DA].frames, 1);

        sbs.poll();
        beast.poll();
        assert_eq!(
            FeedCounts::sbs(Some(&sbs)),
            FeedCounts {
                clients: 1,
                sent: 1,
                dropped: 0
            }
        );
        assert_eq!(
            FeedCounts::beast(Some(&beast)),
            FeedCounts {
                clients: 1,
                sent: 1,
                dropped: 0
            }
        );

        let mut want = Vec::new();
        anrb::beast::encode(&hex(EVEN), &mut want);
        let mut got = vec![0u8; want.len()];
        b.read_exact(&mut got).unwrap();
        assert_eq!(got, want);

        let mut msg = String::new();
        BufReader::new(&s).read_line(&mut msg).unwrap();
        assert!(msg.starts_with("MSG,3,1,1,4009DA,"), "{msg:?}");
        assert!(msg.ends_with("\r\n"), "{msg:?}");
    }

    #[test]
    fn a_frame_is_tracked_with_no_feeds_served() {
        let mut tracker = Tracker::new();
        let f = Frame::new(&hex(ODD), 0).unwrap();
        let (refusal, line) = handle_frame(&f, &mut tracker, None, None);
        assert_eq!(refusal, None);
        assert_eq!(line, format!("   0.00 {ODD} DF17 4009DA tc=11"));
        assert_eq!(tracker.len(), 1);
    }

    #[test]
    fn feed_events_are_taken_once_as_lines() {
        let (mut sbs, _s, mut beast, _b) = served();
        let lines = feed_events(Some(&mut sbs), Some(&mut beast));
        assert_eq!(lines.len(), 2, "{lines:?}");
        assert!(
            lines[0].starts_with("sbs reader 127.0.0.1:") && lines[0].ends_with(" joined (1 now)"),
            "{}",
            lines[0]
        );
        assert!(
            lines[1].starts_with("beast reader 127.0.0.1:")
                && lines[1].ends_with(" joined (1 now)"),
            "{}",
            lines[1]
        );
        assert!(feed_events(Some(&mut sbs), Some(&mut beast)).is_empty());
        assert!(feed_events(None, None).is_empty());
    }

    #[test]
    fn the_dashboard_shows_both_feeds_served_or_not() {
        let off = feed_stats(None, None);
        assert_eq!(off.len(), 2);
        assert_eq!(
            (off[0].name, off[0].port, off[0].flag),
            ("SBS", None, "--sbs")
        );
        assert_eq!(
            (off[1].name, off[1].port, off[1].flag),
            ("Beast", None, "--beast")
        );
        assert!(off
            .iter()
            .all(|f| f.clients == 0 && f.sent == 0 && f.dropped == 0 && f.readers.is_empty()));

        let (sbs, _s, beast, _b) = served();
        let on = feed_stats(Some(&sbs), Some(&beast));
        assert_eq!(on[0].port, Some(sbs.port()));
        assert_eq!(on[1].port, Some(beast.port()));
        assert!(on.iter().all(|f| f.clients == 1 && f.readers.len() == 1));
    }

    fn stats() -> Stats {
        Stats {
            bursts: 200,
            frames: 50,
            clean: 40,
            fixed1: 7,
            fixed2: 3,
            overlaid_hit: 1,
            overlaid_tried: 9,
            ..Stats::default()
        }
    }

    #[test]
    fn the_status_line_has_every_counter() {
        let s = Status {
            up: Duration::from_secs(3723),
            st: stats(),
            d_bursts: 12,
            d_frames: 3,
            aircraft: 5,
            with_pos: 2,
            refused: 1,
            whitelist: 30,
            evicted: 4,
            sbs: FeedCounts {
                clients: 2,
                sent: 99,
                dropped: 1,
            },
            beast: FeedCounts {
                clients: 1,
                sent: 50,
                dropped: 0,
            },
            unlocked: true,
            pong: Some(Duration::from_millis(1250)),
        };
        assert_eq!(s.line(),
                   "up=01:02:03 bursts=200 (+12/s) frames=50 (+3/s) yield=25.0% clean=40 1bit=7 2bit=3 \
                    aircraft=5 pos=2 refused_pos=1 whitelist=30/4 overlaid=1/9 sbs_clients=2 sbs_lines=99 \
                    sbs_dropped=1 beast_clients=1 beast_frames=50 beast_dropped=0 lock=ok pong=1.2s");
        let quiet = Status {
            unlocked: false,
            pong: None,
            ..s
        };
        assert!(
            quiet.line().ends_with(" lock=locked pong=-"),
            "{}",
            quiet.line()
        );
    }

    #[test]
    fn the_lock_note_says_how_the_session_ended() {
        assert_eq!(
            lock_note(&Ok(Locked {
                stopped: true,
                bytes: 0
            })),
            "locked the box; its stream stopped"
        );
        assert_eq!(
            lock_note(&Ok(Locked {
                stopped: false,
                bytes: 512
            })),
            "sent LOCK, but data was still arriving a second later"
        );
        let e = std::io::Error::new(std::io::ErrorKind::TimedOut, "no reply");
        assert_eq!(lock_note(&Err(e)), "could not lock the box: no reply");
    }

    #[test]
    fn the_stop_line_adds_the_feeds_that_ran() {
        let up = Duration::from_secs(61);
        assert_eq!(stop_line(up, &stats(), 5, None, None),
                   "stopping after 00:01:01; bursts=200 frames=50 (clean=40, 1bit=7, 2bit=3) yield=25.0% aircraft=5");
        assert_eq!(
            stop_line(up, &stats(), 5, Some(10), Some(20)),
            "stopping after 00:01:01; bursts=200 frames=50 (clean=40, 1bit=7, 2bit=3) yield=25.0% \
                    aircraft=5 sbs_lines=10 beast_frames=20"
        );
        assert!(stop_line(up, &stats(), 0, None, Some(3)).ends_with("aircraft=0 beast_frames=3"));
    }

    #[test]
    fn the_summary_lists_the_totals_and_each_feed() {
        assert_eq!(
            summary(&stats(), 50, 5, None, None),
            vec![
                "=== bursts=200 frames=50 (clean=40, 1bit=7, 2bit=3) yield=25.0% ===".to_string(),
                "50 frames delivered, 5 aircraft tracked".to_string(),
            ]
        );
        let c = FeedCounts {
            clients: 2,
            sent: 7,
            dropped: 1,
        };
        let lines = summary(&Stats::default(), 0, 0, Some((30003, c)), Some((30005, c)));
        assert_eq!(
            lines[0],
            "=== bursts=0 frames=0 (clean=0, 1bit=0, 2bit=0) yield=0.0% ==="
        );
        assert_eq!(lines[2], "SBS on 30003: 7 lines to 2 client(s)");
        assert_eq!(lines[3], "Beast on 30005: 7 frames to 2 client(s)");
    }
}
