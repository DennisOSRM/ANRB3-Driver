//! The RadarBox driver: bring-up, authentication, keepalive, burst assembly.
//!
//! USB I/O, timing and the session state machine. What the box
//! says is interpreted by `protocol`; what its sample bursts contain is for
//! `decode`, which this module never touches.

use crate::ftdi::{self, Ftdi};
use crate::protocol::{self, Reply};
use std::io;
use std::time::{Duration, Instant};

/// USB vendor id: FTDI.
const VID: u16 = 0x0403;
/// USB product id the RadarBox uses.
const PID: u16 = 0xA2E0;

/// An open RadarBox: the USB link, the session state and the burst being
/// assembled.
pub struct Device {
    ftdi: Ftdi,
    t0: Instant,
    next_ping: Instant,
    acc: Vec<u8>,
    last_byte: Option<Instant>,
    scratch: Vec<u8>,
    out: Vec<u8>,
    /// A handshake has succeeded at least once.
    authed: bool,
    /// Times the device dropped the session and the driver re-authenticated.
    pub relocks: u32,
    /// PINGs sent with no PONG seen since.
    unanswered: u32,
    /// Times the watchdog gave up on a silent link and re-authenticated.
    pub pong_timeouts: u32,
    /// When a PONG was last seen, for showing how healthy the link is.
    last_pong: Option<Instant>,
    /// Firmware string from VERSION, asked for once after the handshake.
    firmware: Option<String>,
    /// Re-authentication attempts that did not recover the link.
    pub relock_failures: u32,
}

/// Largest burst assembled, in bytes. Bytes beyond it are dropped.
const MAX_BURST: usize = 8192;

/// Time between keepalives.
const PING_INTERVAL: Duration = Duration::from_secs(2);

/// Consecutive unanswered PINGs before the link is treated as dead. Five is
/// ten seconds of silence at the [`PING_INTERVAL`] of 2 s - long enough that a
/// burst of traffic delaying a PONG cannot trip it, short enough to recover
/// inside one aircraft's transit.
const PONG_MISSES: u32 = 5;

/// The value ANRB.exe puts after `UNLOCK`: milliseconds since local midnight.
///
/// Built by the vendor as `IntToHex(DateTimeToTimeStamp(Now).Time, 8)`,
/// recovered from the vendor binary at `0x6fc3f5`. The device
/// does not check the argument at all, so this changes nothing observable; it
/// is here so the traffic matches what the vendor software puts on the wire.
/// Always below 86_400_000, so it always fits the eight hex digits.
pub fn unlock_value() -> u32 {
    crate::clock::local(std::time::SystemTime::now()).millis_of_day()
}

fn other(e: rusb::Error) -> io::Error {
    io::Error::other(e)
}

impl Device {
    /// Open the device as it stands and apply the line settings it needs.
    ///
    /// No USB port reset: on this board the FT232R's PWREN# feeds the MCU's
    /// supply, so a port reset power-cycles the MCU and the first two wake
    /// attempts then land while it is still booting. Measured, that is the
    /// difference between a 1.7 s bring-up and a 6.4 s one. The vendor driver
    /// does not do it either - rb.dll and ANRB.exe call FT_ResetDevice, which
    /// is the FTDI SIO reset below, and nothing that re-enumerates.
    pub fn open() -> io::Result<Self> { Self::open_inner(false) }

    /// As [`Device::open`], but power-cycle the MCU first by way of a USB port
    /// reset. The recovery path: slow, and the only thing that helps when the
    /// part is wedged rather than merely busy.
    pub fn power_cycle_open() -> io::Result<Self> { Self::open_inner(true) }

    fn open_inner(power_cycle: bool) -> io::Result<Self> {
        if power_cycle {
            if let Ok(mut f) = Ftdi::open(VID, PID) {
                let _ = f.reset_device();
            }
            std::thread::sleep(Duration::from_millis(200));
        }

        let deadline = Instant::now() + Duration::from_secs(10);
        let mut ftdi = loop {
            match Ftdi::open(VID, PID) {
                Ok(f) => break f,
                Err(rusb::Error::Access) => {
                    return Err(io::Error::new(io::ErrorKind::PermissionDenied,
                                              Ftdi::access_hint()));
                }
                // Nothing on the bus by the deadline. The message names the
                // device rather than reporting a libusb enum, because the
                // usual causes are it being unplugged or, in a container,
                // the bus not being passed through.
                Err(rusb::Error::NoDevice) | Err(rusb::Error::NotFound) if Instant::now() >= deadline => {
                    return Err(io::Error::new(io::ErrorKind::NotFound, format!(
                        "no AirNav RadarBox on the USB bus (looking for {VID:04x}:{PID:04x}). \
In a container, pass the bus through: -v /dev/bus/usb:/dev/bus/usb \
--device-cgroup-rule='c 189:* rmw'")));
                }
                Err(e) if Instant::now() >= deadline => return Err(other(e)),
                Err(_) => std::thread::sleep(Duration::from_millis(10)),
            }
        };

        ftdi.reset().map_err(other)?;
        ftdi.set_baudrate(1_250_000).map_err(other)?;
        ftdi.set_line_8n1().map_err(other)?;
        ftdi.set_latency(2).map_err(other)?;
        // Hardware flow control must stay off: RTS# is wired to the MCU reset.
        ftdi.set_flowctrl(ftdi::DISABLE_FLOW_CTRL).map_err(other)?;
        ftdi.set_dtr(true).map_err(other)?;
        ftdi.set_rts(true).map_err(other)?;
        ftdi.purge().map_err(other)?;

        let now = Instant::now();
        Ok(Device {
            ftdi,
            t0: now,
            next_ping: now,
            acc: Vec::with_capacity(MAX_BURST),
            last_byte: None,
            authed: false,
            relocks: 0,
            unanswered: 0,
            pong_timeouts: 0,
            last_pong: None,
            firmware: None,
            relock_failures: 0,
            scratch: Vec::with_capacity(MAX_BURST),
            out: Vec::with_capacity(MAX_BURST),
        })
    }

    /// UNLOCK, ANSWER, PING and VERSION accept either terminator; this sends
    /// CR+LF and ANRB.exe sends a bare LF. Only the `~2*` family and SIGNAL
    /// are strict, and they require LF - see `send_ctl`.
    pub(crate) fn send_cmd(&self, s: &str) -> io::Result<()> {
        self.ftdi.write(format!("{s}\r\n").as_bytes()).map(|_| ()).map_err(other)
    }
    /// The `~2*` control commands and SIGNAL are only accepted with a bare LF.
    pub(crate) fn send_ctl(&self, s: &str) -> io::Result<()> {
        self.ftdi.write(format!("{s}\n").as_bytes()).map(|_| ()).map_err(other)
    }
    /// Pulse the front-panel LED.
    pub fn signal(&self) -> io::Result<()> { self.send_ctl(protocol::SIGNAL) }
    pub(crate) fn purge(&self) -> io::Result<()> { self.ftdi.purge().map_err(other) }

    fn read_line(&mut self, timeout: Duration) -> io::Result<Option<String>> {
        let deadline = Instant::now() + timeout;
        let mut line = Vec::new();
        while Instant::now() < deadline {
            let mut buf = Vec::new();
            self.ftdi.read(&mut buf, 20).map_err(other)?;
            for &c in &buf {
                if c == b'\n' || c == b'\r' {
                    if !line.is_empty() {
                        return Ok(Some(String::from_utf8_lossy(&line).into_owned()));
                    }
                } else {
                    line.push(c);
                }
            }
        }
        if line.is_empty() {
            Ok(None)
        } else {
            Ok(Some(String::from_utf8_lossy(&line).into_owned()))
        }
    }

    fn send_break(&self, d: Duration) -> io::Result<()> {
        self.ftdi.set_break(true).map_err(other)?;
        std::thread::sleep(d);
        self.ftdi.set_break(false).map_err(other)
    }

    /// The three-way handshake: UNLOCK, QUESTION, then the ANSWER computed by
    /// [`protocol::hs_answer`].
    ///
    /// A BREAK burst first: without it the device intermittently ignores
    /// UNLOCK, because BREAK is what wakes the MCU.
    pub fn authenticate(&mut self) -> io::Result<()> {
        self.authenticate_tries(&mut |_| {}, 8)
    }

    /// Sleep, but let the caller redraw while it happens.
    ///
    /// The bring-up is mostly waiting - a BREAK, then about 1.5 s of settling,
    /// repeated until the device answers - and a display that only hears about
    /// it at phase boundaries cannot show the time passing.
    fn wait(&self, d: Duration, progress: &mut dyn FnMut(&str), what: &str) {
        let end = Instant::now() + d;
        while Instant::now() < end {
            progress(what);
            let left = end.saturating_duration_since(Instant::now());
            std::thread::sleep(left.min(Duration::from_millis(80)));
        }
    }

    /// As [`Device::authenticate`], reporting each phase as it starts and
    /// giving up after `tries` wake attempts, so a caller can fall back to
    /// something heavier instead of waiting out all of them.
    ///
    /// Bring-up takes a couple of seconds even when it goes well - the device
    /// needs a BREAK and about 1.5 s to settle. A caller with a display should
    /// say so rather than show nothing. Wakes are ignored only while the MCU is
    /// still booting, which happens only after [`Device::power_cycle_open`].
    pub fn authenticate_tries(&mut self, progress: &mut dyn FnMut(&str), tries: u32)
        -> io::Result<()>
    {
        let mut question = None;
        for attempt in 0..tries {
            let phase = if attempt == 0 { "waking the device" }
                              else { "no answer yet, waking it again" };
            progress(phase);
            self.send_break(Duration::from_millis(10))?;
            self.wait(Duration::from_millis(40), progress, phase);
            self.send_break(Duration::from_millis(120))?;
            self.wait(Duration::from_millis(1500), progress, phase);
            self.purge()?;
            // Bounded. A device a crashed session left streaming never goes
            // quiet, so draining until it does is a hang - and it answers
            // UNLOCK perfectly well with data still coming.
            let drain = Instant::now() + Duration::from_millis(300);
            while Instant::now() < drain {
                if self.read_line(Duration::from_millis(100))?.is_none() { break; }
            }

            self.send_cmd(&protocol::unlock(unlock_value()))?;
            // Measured on this hardware, a QUESTION arrives 4 to 6 ms after
            // UNLOCK; 600 ms leaves a hundredfold margin.
            let give_up = Instant::now() + Duration::from_millis(600);
            while Instant::now() < give_up {
                progress("waiting for the challenge");
                let Some(line) = self.read_line(Duration::from_millis(150))? else { continue };
                if let Reply::Question(q) = protocol::reply(&line) {
                    question = Some(q);
                    break;
                }
            }
            if question.is_some() {
                break;
            }
        }
        let Some(q) = question else {
            return Err(io::Error::new(io::ErrorKind::TimedOut, "no QUESTION from device"));
        };

        progress("answering the challenge");
        self.send_cmd(&protocol::answer(q))?;

        let deadline = Instant::now() + Duration::from_secs(4);
        while Instant::now() < deadline {
            if let Some(line) = self.read_line(Duration::from_secs(1))? {
                if protocol::reply(&line) == Reply::Unlocked {
                    self.authed = true;
                    self.unanswered = 0;
                    self.last_pong = Some(Instant::now());
                    progress("unlocked, asking for the version");
                    self.read_version();
                    self.next_ping = Instant::now();   // keepalive due at once
                    return Ok(());
                }
            }
        }
        Err(io::Error::new(io::ErrorKind::PermissionDenied, "device did not unlock"))
    }

    /// End the session, as the vendor software does on shutdown: the box stops
    /// streaming at once and wants the full handshake before it sends again.
    ///
    /// It does not reply - measured, LOCK draws no LOCKED - so what is waited
    /// for is the stream going quiet: 300 ms without a byte, a second at most.
    pub fn lock(&mut self) -> io::Result<Locked> {
        self.send_ctl(protocol::LOCK)?;
        self.authed = false;
        let t0 = Instant::now();
        let (mut quiet_since, mut bytes, mut buf) = (Instant::now(), 0usize, Vec::new());
        let stopped = loop {
            if quiet_since.elapsed() >= Duration::from_millis(300) { break true; }
            if t0.elapsed() >= Duration::from_secs(1) { break false; }
            buf.clear();
            self.ftdi.read(&mut buf, 50).map_err(other)?;
            if !buf.is_empty() { bytes += buf.len(); quiet_since = Instant::now(); }
        };
        self.acc.clear();
        self.out.clear();
        Ok(Locked { stopped, bytes })
    }

    /// True once the device has answered a challenge correctly.
    pub fn is_unlocked(&self) -> bool { self.authed }

    /// How long since the device last answered a keepalive.
    pub fn since_pong(&self) -> Option<Duration> {
        self.last_pong.map(|t| t.elapsed())
    }

    /// Firmware string, e.g. "Fw: 02.03.1". Asked for once, after unlocking;
    /// VERSION is the only query this device answers.
    pub fn firmware(&self) -> Option<&str> { self.firmware.as_deref() }

    /// Ask for the firmware string, up to three times: with the box streaming
    /// samples a reply can be lost in the data, or arrive attached to sample
    /// data.
    /// The first answer ends it, so a clean link pays for one round trip.
    fn read_version(&mut self) {
        for _ in 0..3 {
            if self.firmware.is_some() { return; }
            if self.send_cmd(protocol::VERSION).is_err() { return; }
            let deadline = Instant::now() + Duration::from_millis(1200);
            while Instant::now() < deadline {
                match self.read_line(Duration::from_millis(300)) {
                    Ok(Some(l)) => {
                        if let Some(fw) = protocol::firmware(&l) {
                            self.firmware = Some(fw.to_string());
                            return;
                        }
                    }
                    Ok(None) => continue,
                    Err(_) => return,
                }
            }
        }
    }

    /// Send PING if one is due. The device drops the session without them.
    /// Returns true if one was sent, so periodic work can hang off the tick.
    ///
    /// Also the watchdog. The device can stop answering without ever saying
    /// LOCKED, and then this driver would go on pinging a dead link for as
    /// long as it was left running. After `PONG_MISSES` (five) unanswered
    /// keepalives the handshake is run again - ANRB.exe does the same, under
    /// "PONG timeout... forcing a Unconnect."
    pub fn keepalive(&mut self) -> io::Result<bool> {
        if Instant::now() < self.next_ping {
            return Ok(false);
        }
        if self.authed && self.unanswered >= PONG_MISSES {
            self.pong_timeouts += 1;
            self.unanswered = 0;
            if self.authenticate().is_err() {
                self.relock_failures += 1;
            }
            self.acc.clear();
            self.out.clear();
            return Ok(false);
        }
        self.send_cmd(protocol::PING)?;
        self.unanswered += 1;
        self.next_ping = Instant::now() + PING_INTERVAL;
        Ok(true)
    }

    /// Collect bytes until a gap longer than 4 ms closes a burst.
    ///
    /// Returns the burst timestamp when one completed; the bytes are then in
    /// [`Device::burst`]. Bytes that arrived in the same USB read as the gap
    /// belong to the next burst and are carried over.
    pub fn poll_burst(&mut self) -> io::Result<Option<u32>> {
        let mut incoming = std::mem::take(&mut self.scratch);
        incoming.clear();
        self.ftdi.read(&mut incoming, 5).map_err(other)?;
        let now = Instant::now();

        let closed = !self.acc.is_empty()
            && self.last_byte.is_some_and(|t| now.duration_since(t) > Duration::from_millis(4));

        let ms = if closed {
            std::mem::swap(&mut self.out, &mut self.acc);
            self.acc.clear();
            Some(now.duration_since(self.t0).as_millis() as u32)
        } else {
            None
        };

        if !incoming.is_empty() {
            if self.acc.len() + incoming.len() <= MAX_BURST {
                self.acc.extend_from_slice(&incoming);
            }
            self.last_byte = Some(now);
        }
        self.scratch = incoming;

        // Any PONG clears the watchdog, wherever in the burst it landed.
        if protocol::contains_pong(&self.out) {
            self.unanswered = 0;
            self.last_pong = Some(Instant::now());
        }

        // The device ends the session on its own - a missed keepalive, or a
        // power blip - and says so. Recover instead of streaming nothing until
        // someone notices. Only once a handshake has succeeded: before that,
        // LOCKED is the ordinary power-on banner and the caller is about to
        // authenticate anyway.
        if ms.is_some() && self.authed && protocol::is_locked_notice(&self.out) {
            self.relocks += 1;
            if self.authenticate().is_err() {
                self.relock_failures += 1;
            }
            self.acc.clear();           // anything queued predates the reset
            self.out.clear();
            return Ok(None);
        }

        Ok(ms)
    }

    /// The most recently completed burst.
    pub fn burst(&self) -> &[u8] { &self.out }
}

/// What the box did when told to LOCK.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Locked {
    /// The stream went quiet within a second.
    pub stopped: bool,
    /// Bytes still in flight after LOCK, before it did.
    pub bytes: usize,
}

/// Leaving an open session behind leaves the box streaming to nobody until
/// its own watchdog notices, and the next start finds it mid-stream. So a
/// device that is dropped with a session open locks it on the way out: every
/// clean exit, early error returns included. A killed process cannot.
impl Drop for Device {
    fn drop(&mut self) {
        if self.authed { let _ = self.lock(); }
    }
}

#[cfg(test)]
mod tests {
    use super::unlock_value;

    /// The UNLOCK argument is milliseconds since local midnight, so it must
    /// always fit the eight hex digits the vendor format uses.
    #[test]
    fn unlock_value_fits_eight_hex_digits() {
        let v = unlock_value();
        assert!(v < 86_400_000, "{v} is not inside one day");
        let s = format!("{v:08X}");
        assert_eq!(s.len(), 8);
        assert!(matches!(s.as_bytes()[0], b'0'..=b'5'),
                "leading digit must be 0-5, below 0x05265C00, got {s}");
    }
}
