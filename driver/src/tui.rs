//! A terminal dashboard for the live receiver.
//!
//! Written with raw ANSI escape sequences rather than a TUI crate, to keep
//! dependencies few.
//!
//! Stats on the left, the messages as they decode on the right. Redrawn on a
//! tick; nothing is read from the keyboard, so the terminal is left in its
//! normal mode and Ctrl-C behaves as usual.

use std::collections::VecDeque;
use std::io::{self, Write};
use std::sync::atomic::{AtomicBool, AtomicU8, Ordering};
use std::time::{Duration, Instant};

use crate::clock::hms;

/// Set when the user interrupts. The loop notices and shuts down cleanly, so
/// the closing statistics still print.
static INTERRUPTED: AtomicBool = AtomicBool::new(false);
/// Stop signals received so far. The second one exits at once.
static SIGNALS: AtomicU8 = AtomicU8::new(0);
/// Whether anything is on the alternate screen. The handler must not write the
/// escape that leaves it when there is no dashboard - in server mode stdout is
/// a log file or a journal, and those bytes would land in it.
static ALT_ACTIVE: AtomicBool = AtomicBool::new(false);
/// Set once the signal handler is installed, so it is installed only once.
static ARMED: AtomicBool = AtomicBool::new(false);

/// Maximum number of lines kept in the message list.
const LOG_LINES: usize = 512;

/// True once a stop signal (SIGINT, SIGTERM, SIGHUP; Ctrl-C or close on
/// Windows) has arrived. Callers should stop and return.
pub fn interrupted() -> bool {
    INTERRUPTED.load(Ordering::Relaxed)
}

/// Ask for a clean stop on SIGINT, SIGTERM or SIGHUP (Ctrl-C or close on
/// Windows) without putting anything on screen. The dashboard calls this
/// itself; server mode calls it to stop and report with no terminal
/// involved. Safe to call twice.
pub fn catch_signals() {
    if ARMED.swap(true, Ordering::Relaxed) {
        return;
    }
    // The first signal asks the loop to stop, so the closing figures still
    // print. A second one leaves the alternate screen and exits at once;
    // without that step the terminal is left with no cursor and the wrong
    // buffer showing. The handler runs as an ordinary thread, so printing
    // here is safe.
    let _ = ctrlc::set_handler(|| {
        INTERRUPTED.store(true, Ordering::Relaxed);
        if SIGNALS.fetch_add(1, Ordering::Relaxed) >= 1 {
            if ALT_ACTIVE.load(Ordering::Relaxed) {
                let mut out = io::stdout();
                let _ = out.write_all(ALT_OFF.as_bytes());
                let _ = out.flush();
            }
            std::process::exit(130);
        }
    });
}

const ALT_ON: &str = "\x1b[?1049h\x1b[?25l"; // alternate screen, hide cursor
const ALT_OFF: &str = "\x1b[?25h\x1b[?1049l";
const DIM: &str = "\x1b[38;5;245m";
const HEAD: &str = "\x1b[1;38;5;214m";
const VAL: &str = "\x1b[38;5;252m";
const GOOD: &str = "\x1b[38;5;114m";
const WARN: &str = "\x1b[38;5;203m";
const OFF: &str = "\x1b[0m";

/// One network output, as the dashboard shows it.
pub struct FeedStat {
    pub name: &'static str,
    /// None when not serving; `flag` says how to turn it on.
    pub port: Option<u16>,
    pub flag: &'static str,
    pub clients: usize,
    /// Messages sent: lines for SBS, frames for Beast.
    pub sent: u64,
    /// Messages dropped because a reader fell too far behind.
    pub dropped: u64,
    /// Who is reading it, oldest first.
    pub readers: Vec<crate::feed::ReaderInfo>,
}

/// Everything the dashboard shows, gathered once per tick.
pub struct Snapshot {
    pub elapsed: Duration,
    pub unlocked: bool,
    pub firmware: Option<String>,
    pub since_pong: Option<Duration>,
    pub relocks: u32,
    /// Re-authentications that were tried and did not get UNLOCKED back.
    pub relock_failures: u32,
    pub pong_timeouts: u32,
    pub bursts: u64,
    pub frames: u64,
    pub clean: u64,
    pub fixed1: u64,
    pub fixed2: u64,
    /// Change over the last second, for the per-second rates.
    pub d_bursts: u64,
    pub d_frames: u64,
    pub d_bytes: u64,
    pub aircraft: usize,
    pub with_pos: usize,
    pub with_cs: usize,
    /// The network outputs, SBS then Beast.
    pub feeds: Vec<FeedStat>,
}

/// The full-screen dashboard. Restores the terminal when dropped.
pub struct Tui {
    log: VecDeque<String>,
    started: Instant,
    active: bool,
    /// When the bring-up began, so the holding screen can show it passing.
    opening_since: Option<Instant>,
}

/// Typical bring-up: a BREAK, 1.5 s of settling, then the challenge, just
/// under 2 s in all. If the fallback port reset is needed, the MCU reboots and
/// the bar runs past full instead of stalling.
const TYPICAL_OPEN: f64 = 2.0;

impl Tui {
    // No Default impl: new() switches the terminal to the alternate screen.
    #[allow(clippy::new_without_default)]
    pub fn new() -> Self {
        catch_signals();
        let mut t = Self::detached();
        t.enter(&mut io::stdout());
        t
    }

    /// A dashboard that has not taken over the terminal.
    fn detached() -> Self {
        Self {
            log: VecDeque::with_capacity(LOG_LINES),
            started: Instant::now(),
            active: false,
            opening_since: None,
        }
    }

    /// Switch `out` to the alternate screen.
    fn enter(&mut self, out: &mut impl Write) {
        ALT_ACTIVE.store(true, Ordering::Relaxed);
        self.active = true;
        let _ = out.write_all(ALT_ON.as_bytes());
        let _ = out.flush();
    }

    /// Draw a holding screen while the device is being brought up, so the
    /// dashboard is on screen from the first moment rather than after it.
    pub fn status(&mut self, what: &str) {
        let (_, h) = size();
        let o = self.status_screen(what, h);
        let _ = io::stdout().write_all(o.as_bytes());
        let _ = io::stdout().flush();
    }

    /// The holding screen for a terminal `h` rows high, as the escape
    /// sequences that draw it.
    fn status_screen(&mut self, what: &str, h: usize) -> String {
        let t0 = *self.opening_since.get_or_insert_with(Instant::now);
        let secs = t0.elapsed().as_secs_f64();

        const W: usize = 34;
        let frac = (secs / TYPICAL_OPEN).min(1.0);
        let full = (frac * W as f64).round() as usize;
        // The block characters are multi-byte, so the bar is built from two
        // repeated strings rather than by slicing one.
        let done = "█".repeat(full);
        let todo = "░".repeat(W - full);
        let note = if secs <= TYPICAL_OPEN + 1.0 {
            format!("{DIM}{:.1}s of about {:.0}s{OFF}", secs, TYPICAL_OPEN)
        } else {
            format!("{WARN}{:.1}s - longer than usual, still trying{OFF}", secs)
        };

        let mut o = String::with_capacity(1024);
        o.push_str("\x1b[H\x1b[2J");
        o.push_str(&format!(
            "\x1b[2;3H{HEAD}AirNav RadarBox{OFF}  {DIM}live receiver{OFF}"
        ));
        o.push_str(&format!("\x1b[4;3H{VAL}Opening the device{OFF}"));
        o.push_str(&format!(
            "\x1b[6;3H{GOOD}{done}{OFF}{DIM}{todo}{OFF}  {note}"
        ));
        o.push_str(&format!("\x1b[8;3H{DIM}{what}{OFF}"));
        o.push_str(&format!(
            "\x1b[10;3H{DIM}The receiver needs a BREAK and about 1.5 s to settle.{OFF}"
        ));
        o.push_str(&format!(
            "\x1b[11;3H{DIM}If it does not answer, it is power-cycled and the MCU{OFF}"
        ));
        o.push_str(&format!("\x1b[12;3H{DIM}takes about 5 s to boot.{OFF}"));
        o.push_str(&format!("\x1b[{};1H", h));
        o
    }

    /// Append a line to the message list.
    pub fn push(&mut self, line: String) {
        if self.log.len() == LOG_LINES {
            self.log.pop_front();
        }
        self.log.push_back(line);
    }

    /// Redraw the whole screen from `s`.
    pub fn draw(&mut self, s: &Snapshot) {
        let (w, h) = size();
        let o = self.frame(s, w, h);
        let _ = io::stdout().write_all(o.as_bytes());
        let _ = io::stdout().flush();
    }

    /// One whole screen for a terminal `w` columns by `h` rows, as the escape
    /// sequences that draw it. Separate from `draw` so it can be looked at and
    /// tested without a terminal or a device.
    fn frame(&self, s: &Snapshot, w: usize, h: usize) -> String {
        let left = 46usize.min(w.saturating_sub(24));
        let mut o = String::with_capacity(8192);
        o.push_str("\x1b[H\x1b[2J");

        let mut row = 1usize;
        let put = |o: &mut String, r: &mut usize, col: usize, text: &str| {
            o.push_str(&format!("\x1b[{};{}H{}", *r, col, text));
            *r += 1;
        };

        put(
            &mut o,
            &mut row,
            1,
            &format!("{HEAD}AirNav RadarBox{OFF}  {DIM}live receiver{OFF}"),
        );
        row += 1;

        // ---- device
        let state = if s.unlocked {
            format!("{GOOD}UNLOCKED{OFF}")
        } else {
            format!("{WARN}LOCKED{OFF}")
        };
        let pong = match s.since_pong {
            Some(d) if d < Duration::from_secs(6) => {
                format!("{GOOD}{:.1}s ago{OFF}", d.as_secs_f64())
            }
            Some(d) => format!("{WARN}{:.1}s ago{OFF}", d.as_secs_f64()),
            None => format!("{DIM}never{OFF}"),
        };
        put(&mut o, &mut row, 1, &format!("{HEAD}DEVICE{OFF}"));
        for (k, v) in [
            ("state", state),
            (
                "firmware",
                s.firmware
                    .clone()
                    .map(|f| format!("{VAL}{f}{OFF}"))
                    .unwrap_or_else(|| format!("{DIM}unknown{OFF}")),
            ),
            ("last PONG", pong),
            (
                "re-auths",
                if s.relock_failures > 0 {
                    format!(
                        "{} {WARN}({} failed){OFF}",
                        num(s.relocks as u64),
                        s.relock_failures
                    )
                } else {
                    num(s.relocks as u64)
                },
            ),
            (
                "PONG timeouts",
                if s.pong_timeouts > 0 {
                    format!("{WARN}{}{OFF}", s.pong_timeouts)
                } else {
                    num(0)
                },
            ),
            ("uptime", format!("{VAL}{}{OFF}", hms(s.elapsed))),
        ] {
            put(&mut o, &mut row, 3, &field(&k_pad(k), &v));
        }
        row += 1;

        // ---- rates
        put(
            &mut o,
            &mut row,
            1,
            &format!("{HEAD}RATE{OFF}  {DIM}per second{OFF}"),
        );
        for (k, v) in [
            ("bursts", format!("{VAL}{:>7}{OFF} /s", s.d_bursts)),
            ("frames", format!("{VAL}{:>7}{OFF} /s", s.d_frames)),
            ("bytes", format!("{VAL}{:>7}{OFF} /s", human(s.d_bytes))),
            ("yield", pct_of(s.d_frames, s.d_bursts)),
        ] {
            put(&mut o, &mut row, 3, &field(&k_pad(k), &v));
        }
        row += 1;

        // ---- decode totals
        put(
            &mut o,
            &mut row,
            1,
            &format!("{HEAD}DECODE{OFF}  {DIM}since start{OFF}"),
        );
        for (k, v) in [
            ("bursts", num(s.bursts)),
            (
                "frames",
                format!("{VAL}{:>9}{OFF}  {}", s.frames, pct_of(s.frames, s.bursts)),
            ),
            ("clean CRC", share(s.clean, s.frames)),
            ("1-bit fix", share(s.fixed1, s.frames)),
            ("2-bit fix", share(s.fixed2, s.frames)),
        ] {
            put(&mut o, &mut row, 3, &field(&k_pad(k), &v));
        }
        row += 1;

        // ---- aircraft
        put(&mut o, &mut row, 1, &format!("{HEAD}AIRCRAFT{OFF}"));
        for (k, v) in [
            ("tracked", num(s.aircraft as u64)),
            ("with position", num(s.with_pos as u64)),
            ("with callsign", num(s.with_cs as u64)),
        ] {
            put(&mut o, &mut row, 3, &field(&k_pad(k), &v));
        }
        row += 1;

        // ---- feeds
        // One row per feed, the same figures for each, in columns.
        put(
            &mut o,
            &mut row,
            1,
            &format!("{HEAD}FEEDS{OFF}       {DIM} port readers      sent dropped{OFF}"),
        );
        for f in &s.feeds {
            let line = match f.port {
                Some(p) => {
                    let who = if f.clients > 0 {
                        format!("{GOOD}{:>7}{OFF}", f.clients)
                    } else {
                        format!("{DIM}{:>7}{OFF}", 0)
                    };
                    let lost = if f.dropped > 0 {
                        format!("{WARN}{:>8}{OFF}", f.dropped)
                    } else {
                        format!("{DIM}{:>8}{OFF}", 0)
                    };
                    format!(
                        "{:<10}{VAL}{:>6}{OFF}{who}{VAL}{:>10}{OFF}{lost}",
                        f.name, p, f.sent
                    )
                }
                None => format!("{:<10}{DIM}off  ({} PORT){OFF}", f.name, f.flag),
            };
            put(&mut o, &mut row, 3, &line);
        }

        // Then who they are, as far as the rows go.
        let readers: Vec<_> = s
            .feeds
            .iter()
            .flat_map(|f| f.readers.iter().map(move |r| (f.name, r)))
            .collect();
        if !readers.is_empty() && row + 1 < h {
            row += 1;
            put(
                &mut o,
                &mut row,
                1,
                &format!("{HEAD}READERS{OFF}{DIM}                     for  behind{OFF}"),
            );
            let room = (h + 1).saturating_sub(row); // rows row..=h
            for (k, (name, r)) in readers.iter().enumerate() {
                if k + 1 == room && readers.len() > room {
                    put(
                        &mut o,
                        &mut row,
                        3,
                        &format!("{DIM}and {} more{OFF}", readers.len() - k),
                    );
                    break;
                }
                let behind = if r.behind > 0 {
                    format!("{WARN}{:>7}{OFF}", format!("{}K", r.behind.div_ceil(1024)))
                } else {
                    format!("{DIM}{:>7}{OFF}", "-")
                };
                put(
                    &mut o,
                    &mut row,
                    3,
                    &format!(
                        "{:<6}{VAL}{:<22}{OFF}{DIM}{:>5}{OFF} {behind}",
                        name,
                        r.peer.to_string(),
                        short(r.connected)
                    ),
                );
            }
        }

        // ---- messages, right-hand column
        let col = left + 2;
        let width = w.saturating_sub(col).max(10);
        o.push_str(&format!(
            "\x1b[1;{}H{HEAD}MESSAGES{OFF}  {DIM}newest last{OFF}",
            col
        ));
        let rows = h.saturating_sub(2);
        let start = self.log.len().saturating_sub(rows);
        for (i, line) in self.log.iter().skip(start).enumerate() {
            let mut t: String = line.chars().take(width).collect();
            if line.chars().count() > width {
                t.pop();
                t.push('…');
            }
            o.push_str(&format!("\x1b[{};{}H{VAL}{t}{OFF}", i + 2, col));
        }

        // vertical rule between the two halves
        for r in 1..=h {
            o.push_str(&format!("\x1b[{};{}H{DIM}│{OFF}", r, left));
        }

        o.push_str(&format!("\x1b[{};1H", h));
        o
    }

    /// Put the terminal back. Idempotent, so Drop and an explicit call agree.
    pub fn restore(&mut self) {
        self.leave(&mut io::stdout());
    }

    /// Leave the alternate screen on `out`, once.
    fn leave(&mut self, out: &mut impl Write) {
        if !self.active {
            return;
        }
        self.active = false;
        ALT_ACTIVE.store(false, Ordering::Relaxed);
        let _ = out.write_all(ALT_OFF.as_bytes());
        let _ = out.flush();
    }

    /// Time since the dashboard was created.
    pub fn uptime(&self) -> Duration {
        self.started.elapsed()
    }
}

impl Drop for Tui {
    fn drop(&mut self) {
        self.restore();
    }
}

// ---- small formatting helpers ---------------------------------------------

/// A duration in at most five characters: 95s, 10m, 1h01.
fn short(d: Duration) -> String {
    let s = d.as_secs();
    if s < 60 {
        format!("{s}s")
    } else if s < 3600 {
        format!("{}m", s / 60)
    } else {
        format!("{}h{:02}", s / 3600, (s / 60) % 60)
    }
}

fn k_pad(k: &str) -> String {
    format!("{k:<14}")
}
fn field(k: &str, v: &str) -> String {
    format!("{DIM}{k}{OFF}{v}")
}
fn num(v: u64) -> String {
    format!("{VAL}{v:>9}{OFF}")
}

fn pct_of(n: u64, d: u64) -> String {
    if d == 0 {
        return format!("{DIM}    -  {OFF}");
    }
    format!("{VAL}{:>5.1}%{OFF}", 100.0 * n as f64 / d as f64)
}

fn share(n: u64, total: u64) -> String {
    if total == 0 {
        return format!("{VAL}{:>9}{OFF}", n);
    }
    format!(
        "{VAL}{n:>9}{OFF}  {DIM}{:>5.1}%{OFF}",
        100.0 * n as f64 / total as f64
    )
}

fn human(b: u64) -> String {
    if b >= 1_000_000 {
        format!("{:.1}M", b as f64 / 1e6)
    } else if b >= 1_000 {
        format!("{:.1}k", b as f64 / 1e3)
    } else {
        b.to_string()
    }
}

/// Terminal size, or a conservative default when stdout is not a terminal.
fn size() -> (usize, usize) {
    use terminal_size::{terminal_size, Height, Width};
    match terminal_size() {
        Some((Width(w), Height(h))) if w > 20 && h > 8 => (w as usize, h as usize),
        _ => (100, 30),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The screen as plain text: interpret the cursor moves, drop the colours.
    fn screen(esc: &str, w: usize, h: usize) -> Vec<String> {
        let mut g = vec![vec![' '; w]; h];
        let (mut r, mut c) = (0usize, 0usize);
        let mut it = esc.chars().peekable();
        while let Some(ch) = it.next() {
            if ch == '\x1b' && it.peek() == Some(&'[') {
                it.next();
                let mut arg = String::new();
                let mut fin = ' ';
                for x in it.by_ref() {
                    if x.is_ascii_alphabetic() {
                        fin = x;
                        break;
                    }
                    arg.push(x);
                }
                if fin == 'H' {
                    let n: Vec<usize> = arg.split(';').map(|v| v.parse().unwrap_or(1)).collect();
                    r = n.first().copied().unwrap_or(1).saturating_sub(1);
                    c = n.get(1).copied().unwrap_or(1).saturating_sub(1);
                }
                continue;
            }
            if r < h && c < w {
                g[r][c] = ch;
            }
            c += 1;
        }
        g.into_iter()
            .map(|l| l.into_iter().collect::<String>().trim_end().to_string())
            .collect()
    }

    fn reader(peer: &str, secs: u64, behind: usize) -> crate::feed::ReaderInfo {
        crate::feed::ReaderInfo {
            peer: peer.parse().unwrap(),
            connected: Duration::from_secs(secs),
            behind,
        }
    }

    fn sample() -> Snapshot {
        Snapshot {
            elapsed: Duration::from_secs(3723),
            unlocked: true,
            firmware: Some("Fw: 02.03.1".into()),
            since_pong: Some(Duration::from_millis(700)),
            relocks: 0,
            relock_failures: 0,
            pong_timeouts: 0,
            bursts: 147_897,
            frames: 23_106,
            clean: 10_573,
            fixed1: 6_753,
            fixed2: 5_780,
            d_bursts: 196,
            d_frames: 33,
            d_bytes: 700,
            aircraft: 10,
            with_pos: 8,
            with_cs: 7,
            feeds: vec![
                FeedStat {
                    name: "SBS",
                    port: Some(30003),
                    flag: "--sbs",
                    clients: 1,
                    sent: 23_091,
                    dropped: 0,
                    readers: vec![reader("127.0.0.1:35138", 3700, 0)],
                },
                FeedStat {
                    name: "Beast",
                    port: Some(30005),
                    flag: "--beast",
                    clients: 2,
                    sent: 23_106,
                    dropped: 1,
                    readers: vec![
                        reader("192.168.179.30:51234", 610, 0),
                        reader("192.168.179.41:40022", 95, 70_000),
                    ],
                },
            ],
        }
    }

    /// Prints the dashboard for a sample snapshot:
    /// `cargo test --release preview -- --nocapture`.
    #[test]
    fn preview() {
        let t = Tui {
            log: VecDeque::new(),
            started: Instant::now(),
            active: false,
            opening_since: None,
        };
        let (w, h) = (118, 36);
        let rows = screen(&t.frame(&sample(), w, h), w, h);
        for l in &rows {
            println!("{l}");
        }
        let left: Vec<String> = rows.iter().map(|l| l.chars().take(45).collect()).collect();
        assert!(
            left.iter()
                .any(|l| l.contains("Beast") && l.trim_end().ends_with('1')),
            "the Beast row, dropped count included, fits left of the divider"
        );
        assert!(
            left.iter()
                .any(|l| l.contains("192.168.179.41:40022") && l.contains("69K")),
            "a lagging reader is named and says how far behind, left of the divider"
        );
    }

    /// A message wider than its column ends in an ellipsis, also when it
    /// holds multi-byte characters; one that fits is shown whole.
    #[test]
    fn long_messages_are_cut_with_an_ellipsis() {
        let mut t = Tui {
            log: VecDeque::new(),
            started: Instant::now(),
            active: false,
            opening_since: None,
        };
        let (w, h) = (118, 36);
        t.push("é".repeat(80));
        t.push("fits".into());
        let rows = screen(&t.frame(&sample(), w, h), w, h);
        assert!(rows[1].ends_with('…'), "{}", rows[1]);
        assert!(rows[2].ends_with("fits"), "{}", rows[2]);
    }

    #[test]
    fn percentages_do_not_divide_by_zero() {
        assert!(
            pct_of(0, 0).contains('-'),
            "no traffic yet reads as a dash, not NaN"
        );
        assert!(share(0, 0).contains('0'));
        assert!(pct_of(1, 4).contains("25.0"));
    }

    #[test]
    fn byte_counts_stay_short() {
        assert_eq!(human(999), "999");
        assert_eq!(human(1_500), "1.5k");
        assert_eq!(human(2_500_000), "2.5M");
    }

    /// The message list stays bounded however long the receiver runs.
    #[test]
    fn message_log_is_bounded() {
        let mut t = Tui {
            log: VecDeque::new(),
            started: Instant::now(),
            active: false,
            opening_since: None,
        };
        for i in 0..2000 {
            t.push(format!("line {i}"));
        }
        assert_eq!(t.log.len(), LOG_LINES);
        assert_eq!(t.log.back().unwrap(), "line 1999", "the newest is kept");
    }

    /// Entering writes the escape that switches to the alternate screen, and
    /// leaving writes the one that switches back, once however often it is
    /// asked.
    #[test]
    fn entering_and_leaving_the_alternate_screen() {
        let mut t = Tui::detached();
        let mut out = Vec::new();
        t.leave(&mut out);
        assert!(
            out.is_empty(),
            "a dashboard that never entered has nothing to undo"
        );
        t.enter(&mut out);
        assert_eq!(out, ALT_ON.as_bytes());
        assert!(t.active);
        out.clear();
        t.leave(&mut out);
        t.leave(&mut out);
        assert_eq!(out, ALT_OFF.as_bytes());
        assert!(!t.active);
        assert!(t.uptime() < Duration::from_secs(60));
        assert!(!interrupted(), "no stop signal in a test");
    }

    /// The holding screen: an empty bar at first, a full one and a warning
    /// once the bring-up runs long.
    #[test]
    fn the_holding_screen_shows_progress() {
        let mut t = Tui::detached();
        let (w, h) = (80, 24);
        let esc = t.status_screen("opening the USB device", h);
        assert!(
            esc.ends_with("\x1b[24;1H"),
            "the cursor is parked on the last row"
        );
        let rows = screen(&esc, w, h);
        assert!(rows[1].contains("AirNav RadarBox"));
        assert!(rows[3].contains("Opening the device"));
        assert!(
            rows[5].contains(&"░".repeat(34)) && rows[5].contains("of about 2s"),
            "{}",
            rows[5]
        );
        assert!(rows[7].contains("opening the USB device"));

        // The same bring-up four seconds on.
        t.opening_since = Instant::now().checked_sub(Duration::from_secs(4));
        let rows = screen(&t.status_screen("resetting the port", h), w, h);
        assert!(rows[5].contains(&"█".repeat(34)), "{}", rows[5]);
        assert!(
            rows[5].contains("longer than usual, still trying"),
            "{}",
            rows[5]
        );
        assert!(rows[7].contains("resetting the port"));
    }

    /// A locked device with an old PONG, failed re-authentications and PONG
    /// timeouts, and a feed that is turned off.
    #[test]
    fn warnings_are_shown() {
        let t = Tui::detached();
        let mut s = sample();
        s.unlocked = false;
        s.firmware = None;
        s.since_pong = Some(Duration::from_secs(9));
        s.relocks = 3;
        s.relock_failures = 2;
        s.pong_timeouts = 4;
        s.feeds[0].port = None;
        s.feeds[1].clients = 0;
        s.feeds[1].dropped = 0;
        let (w, h) = (118, 36);
        let esc = t.frame(&s, w, h);
        let rows = screen(&esc, w, h);
        let find = |k: &str| {
            rows.iter()
                .find(|l| l.contains(k))
                .unwrap_or_else(|| panic!("no {k}"))
                .clone()
        };
        assert!(find("state").contains("LOCKED"));
        assert!(find("firmware").contains("unknown"));
        assert!(find("last PONG").contains("9.0s ago"));
        assert!(
            esc.contains(&format!("{WARN}9.0s ago")),
            "an old PONG is a warning"
        );
        assert!(find("re-auths").contains("3 (2 failed)"));
        assert!(find("PONG timeouts").contains('4'));
        assert!(find("SBS").contains("off  (--sbs PORT)"), "{}", find("SBS"));

        s.since_pong = None;
        let rows = screen(&t.frame(&s, w, h), w, h);
        assert!(rows
            .iter()
            .any(|l| l.contains("last PONG") && l.contains("never")));
    }

    /// With more readers than rows, the last row says how many are not shown.
    #[test]
    fn readers_that_do_not_fit_are_counted() {
        let t = Tui::detached();
        let mut s = sample();
        s.feeds[1].readers = (0..20)
            .map(|i| reader(&format!("10.0.0.{i}:4000"), 5, 0))
            .collect();
        let (w, h) = (118, 36);
        let rows = screen(&t.frame(&s, w, h), w, h);
        let heading = rows
            .iter()
            .position(|l| l.starts_with("READERS"))
            .expect("the readers heading");
        // Rows after the heading, to the bottom of the screen.
        let room = h - 1 - heading;
        assert!(
            rows[h - 1].contains(&format!("and {} more", 21 - (room - 1))),
            "{}",
            rows[h - 1]
        );
        assert!(
            rows[heading + 1].contains("127.0.0.1:35138") && rows[heading + 1].contains("1h01")
        );
        assert_eq!(
            rows.iter().filter(|l| l.contains("10.0.0.")).count(),
            room - 2
        );

        // A screen too short for any reader leaves the list out.
        let rows = screen(&t.frame(&s, w, 28), w, 28);
        assert!(!rows.iter().any(|l| l.starts_with("READERS")));
    }

    #[test]
    fn durations_are_short() {
        assert_eq!(short(Duration::from_secs(95)), "1m");
        assert_eq!(short(Duration::from_secs(59)), "59s");
        assert_eq!(short(Duration::from_secs(600)), "10m");
        assert_eq!(short(Duration::from_secs(3660)), "1h01");
    }

    /// The terminal size is a real one or the fallback, never something too
    /// small to draw in.
    #[test]
    fn the_size_is_usable() {
        let (w, h) = size();
        assert!((w > 20 && h > 8), "{w}x{h}");
    }
}
