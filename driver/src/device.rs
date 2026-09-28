//! The RadarBox driver: bring-up, authentication, keepalive, burst assembly.
//!
//! USB I/O, timing and the session state machine. What the box
//! says is interpreted by `protocol`; what its sample bursts contain is for
//! `decode`, which this module never touches.

use crate::ftdi::{self, Ftdi, Port};
use crate::protocol::{self, Reply};
use std::io;
use std::time::{Duration, Instant};

/// USB vendor id: FTDI.
const VID: u16 = 0x0403;
/// USB product id the RadarBox uses.
const PID: u16 = 0xA2E0;

/// An open RadarBox: the USB link, the session state and the burst being
/// assembled.
///
/// `P` is the link, [`Ftdi`] unless a test puts something else there.
pub struct Device<P: Port = Ftdi> {
    port: P,
    t0: Instant,
    next_ping: Instant,
    acc: Vec<u8>,
    last_byte: Option<Instant>,
    scratch: Vec<u8>,
    out: Vec<u8>,
    /// Bytes read after the end of the last text line, for the next one.
    pending: Vec<u8>,
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
    pub fn open() -> io::Result<Self> {
        Self::open_inner(false)
    }

    /// As [`Device::open`], but power-cycle the MCU first by way of a USB port
    /// reset. The recovery path: slow, and the only thing that helps when the
    /// part is wedged rather than merely busy.
    pub fn power_cycle_open() -> io::Result<Self> {
        Self::open_inner(true)
    }
}

impl<P: Port> Device<P> {
    pub(crate) fn open_inner(power_cycle: bool) -> io::Result<Self> {
        if power_cycle {
            if let Ok(mut f) = P::open(VID, PID) {
                let _ = f.reset_device();
            }
            P::sleep(Duration::from_millis(200));
        }

        let deadline = P::now() + Duration::from_secs(10);
        let mut port = loop {
            match P::open(VID, PID) {
                Ok(f) => break f,
                Err(rusb::Error::Access) => {
                    return Err(io::Error::new(
                        io::ErrorKind::PermissionDenied,
                        Ftdi::access_hint(),
                    ));
                }
                // Nothing on the bus by the deadline. The message names the
                // device rather than reporting a libusb enum, because the
                // usual causes are it being unplugged or, in a container,
                // the bus not being passed through.
                Err(rusb::Error::NoDevice) | Err(rusb::Error::NotFound) if P::now() >= deadline => {
                    return Err(io::Error::new(
                        io::ErrorKind::NotFound,
                        format!(
                            "no AirNav RadarBox on the USB bus (looking for {VID:04x}:{PID:04x}). \
In a container, pass the bus through: -v /dev/bus/usb:/dev/bus/usb \
--device-cgroup-rule='c 189:* rmw'"
                        ),
                    ));
                }
                Err(e) if P::now() >= deadline => return Err(other(e)),
                Err(_) => P::sleep(Duration::from_millis(10)),
            }
        };

        port.reset().map_err(other)?;
        port.set_baudrate(1_250_000).map_err(other)?;
        port.set_line_8n1().map_err(other)?;
        port.set_latency(2).map_err(other)?;
        // Hardware flow control must stay off: RTS# is wired to the MCU reset.
        port.set_flowctrl(ftdi::DISABLE_FLOW_CTRL).map_err(other)?;
        port.set_dtr(true).map_err(other)?;
        port.set_rts(true).map_err(other)?;
        port.purge().map_err(other)?;

        let now = P::now();
        Ok(Device {
            port,
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
            pending: Vec::new(),
        })
    }

    /// UNLOCK, ANSWER, PING and VERSION accept either terminator; this sends
    /// CR+LF and ANRB.exe sends a bare LF. Only the `~2*` family and SIGNAL
    /// are strict, and they require LF - see `send_ctl`.
    pub(crate) fn send_cmd(&self, s: &str) -> io::Result<()> {
        self.port
            .write(format!("{s}\r\n").as_bytes())
            .map(|_| ())
            .map_err(other)
    }
    /// The `~2*` control commands and SIGNAL are only accepted with a bare LF.
    pub(crate) fn send_ctl(&self, s: &str) -> io::Result<()> {
        self.port
            .write(format!("{s}\n").as_bytes())
            .map(|_| ())
            .map_err(other)
    }
    /// Pulse the front-panel LED.
    pub fn signal(&self) -> io::Result<()> {
        self.send_ctl(protocol::SIGNAL)
    }
    pub(crate) fn purge(&self) -> io::Result<()> {
        self.port.purge().map_err(other)
    }

    /// The next text line, from what the last call left over and then from
    /// reads of up to 20 ms each until `timeout`.
    ///
    /// One read can hold more than one line - a streaming box ends every
    /// burst with `00 0a`, and a reply that follows a burst comes in the same
    /// read - so the bytes after the line end are kept for the next call.
    fn read_line(&mut self, timeout: Duration) -> io::Result<Option<String>> {
        let deadline = P::now() + timeout;
        let mut line = Vec::new();
        let mut buf = std::mem::take(&mut self.pending);
        loop {
            let mut end = None;
            for (i, &c) in buf.iter().enumerate() {
                if c == b'\n' || c == b'\r' {
                    if !line.is_empty() {
                        end = Some(i);
                        break;
                    }
                } else {
                    line.push(c);
                }
            }
            if let Some(i) = end {
                self.pending = buf.split_off(i + 1);
                return Ok(Some(String::from_utf8_lossy(&line).into_owned()));
            }
            if P::now() >= deadline {
                break;
            }
            buf.clear();
            self.port.read(&mut buf, 20).map_err(other)?;
        }
        if line.is_empty() {
            Ok(None)
        } else {
            Ok(Some(String::from_utf8_lossy(&line).into_owned()))
        }
    }

    fn send_break(&self, d: Duration) -> io::Result<()> {
        self.port.set_break(true).map_err(other)?;
        P::sleep(d);
        self.port.set_break(false).map_err(other)
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
        let end = P::now() + d;
        while P::now() < end {
            progress(what);
            let left = end.saturating_duration_since(P::now());
            P::sleep(left.min(Duration::from_millis(80)));
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
    pub fn authenticate_tries(
        &mut self,
        progress: &mut dyn FnMut(&str),
        tries: u32,
    ) -> io::Result<()> {
        let mut question = None;
        for attempt in 0..tries {
            let phase = if attempt == 0 {
                "waking the device"
            } else {
                "no answer yet, waking it again"
            };
            progress(phase);
            self.send_break(Duration::from_millis(10))?;
            self.wait(Duration::from_millis(40), progress, phase);
            self.send_break(Duration::from_millis(120))?;
            self.wait(Duration::from_millis(1500), progress, phase);
            self.purge()?;
            // Bounded. A device a crashed session left streaming never goes
            // quiet, so draining until it does is a hang - and it answers
            // UNLOCK perfectly well with data still coming.
            let drain = P::now() + Duration::from_millis(300);
            while P::now() < drain {
                if self.read_line(Duration::from_millis(100))?.is_none() {
                    break;
                }
            }

            // What the drain left is older than UNLOCK; do not read it as
            // part of the reply.
            self.pending.clear();
            self.send_cmd(&protocol::unlock(unlock_value()))?;
            // Measured on this hardware, a QUESTION arrives 4 to 6 ms after
            // UNLOCK; 600 ms leaves a hundredfold margin.
            let give_up = P::now() + Duration::from_millis(600);
            while P::now() < give_up {
                progress("waiting for the challenge");
                let Some(line) = self.read_line(Duration::from_millis(150))? else {
                    continue;
                };
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
            return Err(io::Error::new(
                io::ErrorKind::TimedOut,
                "no QUESTION from device",
            ));
        };

        progress("answering the challenge");
        self.send_cmd(&protocol::answer(q))?;

        let deadline = P::now() + Duration::from_secs(4);
        while P::now() < deadline {
            if let Some(line) = self.read_line(Duration::from_secs(1))? {
                if protocol::reply(&line) == Reply::Unlocked {
                    self.authed = true;
                    self.unanswered = 0;
                    self.last_pong = Some(P::now());
                    progress("unlocked, asking for the version");
                    self.read_version();
                    self.next_ping = P::now(); // keepalive due at once
                    return Ok(());
                }
            }
        }
        Err(io::Error::new(
            io::ErrorKind::PermissionDenied,
            "device did not unlock",
        ))
    }

    /// End the session, as the vendor software does on shutdown: the box stops
    /// streaming at once and wants the full handshake before it sends again.
    ///
    /// It does not reply - measured, LOCK draws no LOCKED - so what is waited
    /// for is the stream going quiet: 300 ms without a byte, a second at most.
    pub fn lock(&mut self) -> io::Result<Locked> {
        self.send_ctl(protocol::LOCK)?;
        self.authed = false;
        let t0 = P::now();
        let (mut quiet_since, mut bytes, mut buf) = (P::now(), 0usize, Vec::new());
        let stopped = loop {
            if P::now().duration_since(quiet_since) >= Duration::from_millis(300) {
                break true;
            }
            if P::now().duration_since(t0) >= Duration::from_secs(1) {
                break false;
            }
            buf.clear();
            self.port.read(&mut buf, 50).map_err(other)?;
            if !buf.is_empty() {
                bytes += buf.len();
                quiet_since = P::now();
            }
        };
        self.acc.clear();
        self.out.clear();
        Ok(Locked { stopped, bytes })
    }

    /// True once the device has answered a challenge correctly.
    pub fn is_unlocked(&self) -> bool {
        self.authed
    }

    /// How long since the device last answered a keepalive.
    pub fn since_pong(&self) -> Option<Duration> {
        self.last_pong.map(|t| P::now().duration_since(t))
    }

    /// Firmware string, e.g. "Fw: 02.03.1". Asked for once, after unlocking;
    /// VERSION is the only query this device answers.
    pub fn firmware(&self) -> Option<&str> {
        self.firmware.as_deref()
    }

    /// Ask for the firmware string, up to three times: with the box streaming
    /// samples a reply can be lost in the data, or arrive attached to sample
    /// data.
    /// The first answer ends it, so a clean link pays for one round trip.
    fn read_version(&mut self) {
        for _ in 0..3 {
            if self.firmware.is_some() {
                return;
            }
            if self.send_cmd(protocol::VERSION).is_err() {
                return;
            }
            let deadline = P::now() + Duration::from_millis(1200);
            while P::now() < deadline {
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
        if P::now() < self.next_ping {
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
        self.next_ping = P::now() + PING_INTERVAL;
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
        self.port.read(&mut incoming, 5).map_err(other)?;
        let now = P::now();

        let closed = !self.acc.is_empty()
            && self
                .last_byte
                .is_some_and(|t| now.duration_since(t) > Duration::from_millis(4));

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
            self.last_pong = Some(P::now());
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
            self.acc.clear(); // anything queued predates the reset
            self.out.clear();
            return Ok(None);
        }

        Ok(ms)
    }

    /// The most recently completed burst.
    pub fn burst(&self) -> &[u8] {
        &self.out
    }
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
impl<P: Port> Drop for Device<P> {
    fn drop(&mut self) {
        if self.authed {
            let _ = self.lock();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ftdi::fake::{self, Ev, FakePort, Script};
    use crate::ftdi::Ctrl;
    use std::cell::RefCell;
    use std::rc::Rc;

    type Dev = Device<FakePort>;
    type Shared = Rc<RefCell<Script>>;

    fn ms(n: u64) -> Duration {
        Duration::from_millis(n)
    }

    /// A device opened on the fake, not yet authenticated.
    fn open() -> (Dev, Shared) {
        let (p, s) = FakePort::new();
        fake::expect_open(Ok(p));
        (Dev::open_inner(false).expect("open"), s)
    }

    /// A device opened on the fake and authenticated.
    fn unlocked() -> (Dev, Shared) {
        let (mut d, s) = open();
        d.authenticate().expect("authenticate");
        (d, s)
    }

    fn clear(s: &Shared) {
        let mut s = s.borrow_mut();
        s.log.clear();
        s.at.clear();
    }

    fn unlocks(s: &Shared) -> usize {
        s.borrow()
            .writes()
            .iter()
            .filter(|w| w.starts_with("UNLOCK "))
            .count()
    }

    // ---- opening ----------------------------------------------------------

    /// The UNLOCK argument is milliseconds since local midnight, so it must
    /// always fit the eight hex digits the vendor format uses.
    #[test]
    fn unlock_value_fits_eight_hex_digits() {
        let v = unlock_value();
        assert!(v < 86_400_000, "{v} is not inside one day");
        let s = format!("{v:08X}");
        assert_eq!(s.len(), 8);
        assert!(
            matches!(s.as_bytes()[0], b'0'..=b'5'),
            "leading digit must be 0-5, below 0x05265C00, got {s}"
        );
    }

    #[test]
    fn opening_sends_the_line_settings_in_order() {
        let (d, s) = open();
        assert_eq!(
            s.borrow().ctrls(),
            vec![
                Ctrl::reset(),
                Ctrl::baudrate(1_250_000),
                Ctrl::line_8n1(false),
                Ctrl::latency(2),
                Ctrl::flowctrl(ftdi::DISABLE_FLOW_CTRL),
                Ctrl::dtr(true),
                Ctrl::rts(true),
                Ctrl::purge_rx(),
                Ctrl::purge_tx(),
            ]
        );
        assert!(
            s.borrow().writes().is_empty(),
            "nothing is written while opening"
        );
        assert!(!d.is_unlocked());
        assert_eq!(d.since_pong(), None);
        assert_eq!(d.firmware(), None);
    }

    #[test]
    fn opening_waits_for_the_device_to_appear() {
        let (p, _s) = FakePort::new();
        for _ in 0..3 {
            fake::expect_open(Err(rusb::Error::NoDevice));
        }
        fake::expect_open(Ok(p));
        let t = fake::now();
        Dev::open_inner(false).expect("open");
        assert_eq!(fake::now() - t, ms(30), "three retries 10 ms apart");
    }

    #[test]
    fn a_missing_device_is_reported_after_ten_seconds() {
        let t = fake::now();
        let e = Dev::open_inner(false).err().expect("no device");
        assert_eq!(e.kind(), io::ErrorKind::NotFound);
        assert!(
            e.to_string()
                .contains("no AirNav RadarBox on the USB bus (looking for 0403:a2e0)"),
            "{e}"
        );
        assert_eq!(fake::now() - t, Duration::from_secs(10));
    }

    #[test]
    fn a_denied_claim_is_reported_at_once_with_the_hint() {
        fake::expect_open(Err(rusb::Error::Access));
        let e = Dev::open_inner(false).err().expect("denied");
        assert_eq!(e.kind(), io::ErrorKind::PermissionDenied);
        assert_eq!(e.to_string(), Ftdi::access_hint());
    }

    #[test]
    fn any_other_open_error_is_retried_until_the_deadline() {
        // Opens at 0, 10, ... 10000 ms: the last one is past the deadline.
        for _ in 0..=1000 {
            fake::expect_open(Err(rusb::Error::Busy));
        }
        let e = Dev::open_inner(false).err().expect("busy");
        assert_eq!(e.kind(), io::ErrorKind::Other);
        assert_eq!(e.to_string(), rusb::Error::Busy.to_string());
    }

    #[test]
    fn a_failed_control_request_fails_the_open() {
        for n in 0..9 {
            let (p, s) = FakePort::new();
            s.borrow_mut().fail_ctrl_at = Some(n);
            fake::expect_open(Ok(p));
            assert!(Dev::open_inner(false).is_err(), "request {n}");
            assert_eq!(
                s.borrow().ctrls().len(),
                n + 1,
                "nothing is sent after request {n}"
            );
        }
    }

    #[test]
    fn power_cycle_resets_the_port_and_opens_again() {
        let (p, s) = FakePort::new();
        fake::expect_open(Ok(p));
        fake::expect_open(Ok(FakePort::again(&s)));
        let _d = Dev::open_inner(true).expect("open");
        let s = s.borrow();
        assert_eq!(s.log[0], Ev::ResetDevice);
        assert_eq!(s.log[1], Ev::Ctrl(Ctrl::reset()));
        assert_eq!(
            s.at[1] - s.at[0],
            ms(200),
            "the MCU is given 200 ms to go down"
        );
    }

    #[test]
    fn power_cycle_goes_on_when_the_first_open_fails() {
        let (p, s) = FakePort::new();
        fake::expect_open(Err(rusb::Error::NoDevice));
        fake::expect_open(Ok(p));
        let _d = Dev::open_inner(true).expect("open");
        assert!(!s.borrow().log.contains(&Ev::ResetDevice));
    }

    // ---- the handshake ----------------------------------------------------

    #[test]
    fn the_handshake_sends_break_unlock_answer_and_version() {
        let (mut d, s) = open();
        clear(&s);
        let mut said = Vec::<String>::new();
        let mut progress = |m: &str| {
            if said.last().map(String::as_str) != Some(m) {
                said.push(m.to_string());
            }
        };
        d.authenticate_tries(&mut progress, 1).expect("unlocks");

        assert_eq!(
            said,
            [
                "waking the device",
                "waiting for the challenge",
                "answering the challenge",
                "unlocked, asking for the version",
            ]
        );
        let s = s.borrow();
        assert_eq!(
            s.ctrls(),
            vec![
                Ctrl::line_8n1(true),
                Ctrl::line_8n1(false), // BREAK, 10 ms
                Ctrl::line_8n1(true),
                Ctrl::line_8n1(false), // BREAK, 120 ms
                Ctrl::purge_rx(),
                Ctrl::purge_tx(),
            ]
        );
        let gaps: Vec<Duration> = s.at.windows(2).take(4).map(|w| w[1] - w[0]).collect();
        assert_eq!(gaps, [ms(10), ms(40), ms(120), ms(1500)]);
        // The bounded drain finds nothing and stops after one 100 ms line.
        assert_eq!(s.at[6] - s.at[5], ms(100));

        let w = s.writes();
        assert_eq!(w.len(), 3);
        let arg = w[0]
            .strip_prefix("UNLOCK ")
            .and_then(|r| r.strip_suffix("\r\n"))
            .expect("UNLOCK");
        assert!(
            arg.len() == 8 && arg.bytes().all(|c| c.is_ascii_hexdigit()),
            "{arg}"
        );
        assert_eq!(w[1], "ANSWER 021A928C\r\n");
        assert_eq!(w[2], "VERSION\r\n");

        assert!(d.is_unlocked());
        assert_eq!(d.firmware(), Some("Fw: 02.03.1"));
        assert!(d.since_pong().expect("counted from the unlock") < ms(10));
    }

    #[test]
    fn authenticate_wakes_the_device_again_until_it_answers() {
        let (mut d, s) = open();
        s.borrow_mut().ignore_unlocks = 2;
        let mut said = Vec::<String>::new();
        d.authenticate_tries(&mut |m| said.push(m.to_string()), 3)
            .expect("third try");
        assert_eq!(unlocks(&s), 3);
        assert!(said.iter().any(|m| m == "no answer yet, waking it again"));
        assert!(d.is_unlocked());
    }

    #[test]
    fn authenticate_gives_up_after_its_tries() {
        let (mut d, s) = open();
        s.borrow_mut().question_reply = None;
        let e = d
            .authenticate_tries(&mut |_| {}, 2)
            .expect_err("no QUESTION");
        assert_eq!(e.kind(), io::ErrorKind::TimedOut);
        assert_eq!(unlocks(&s), 2);
        assert!(!s.borrow().writes().iter().any(|w| w.starts_with("ANSWER")));
        assert!(!d.is_unlocked());
    }

    #[test]
    fn authenticate_tries_eight_times_by_default() {
        let (mut d, s) = open();
        s.borrow_mut().question_reply = None;
        assert!(d.authenticate().is_err());
        assert_eq!(unlocks(&s), 8);
    }

    #[test]
    fn a_malformed_question_is_not_answered() {
        let (mut d, s) = open();
        s.borrow_mut().question_reply = Some(b"QUESTION ZZ\r\n".to_vec());
        let e = d
            .authenticate_tries(&mut |_| {}, 1)
            .expect_err("no usable QUESTION");
        assert_eq!(e.kind(), io::ErrorKind::TimedOut);
    }

    #[test]
    fn a_refused_answer_fails_after_four_seconds() {
        let (mut d, s) = open();
        s.borrow_mut().accept = false;
        let e = d.authenticate_tries(&mut |_| {}, 1).expect_err("refused");
        assert_eq!(e.kind(), io::ErrorKind::PermissionDenied);
        let s = s.borrow();
        let answered = s
            .log
            .iter()
            .position(|e| matches!(e, Ev::Write(w) if w.starts_with("ANSWER")))
            .unwrap();
        assert_eq!(fake::now() - s.at[answered], Duration::from_secs(4));
        assert!(!d.is_unlocked());
        assert_eq!(d.firmware(), None);
    }

    #[test]
    fn a_device_that_checks_another_challenge_does_not_unlock() {
        // The device checks the answer against a different challenge from the
        // one it sent, so the answer is wrong and it stays silent.
        let (mut d, s) = open();
        s.borrow_mut().question = 0x1234_5678;
        let e = d
            .authenticate_tries(&mut |_| {}, 1)
            .expect_err("wrong answer");
        assert_eq!(e.kind(), io::ErrorKind::PermissionDenied);
    }

    #[test]
    fn a_device_left_streaming_still_unlocks() {
        // A crashed session leaves the box sending samples, with no line
        // ends, and it never goes quiet. The drain before UNLOCK is bounded.
        let (mut d, s) = open();
        s.borrow_mut().stream = Some(vec![0xF0; 60]);
        d.authenticate_tries(&mut |_| {}, 1).expect("unlocks");
        assert!(d.is_unlocked());
        assert_eq!(d.firmware(), Some("Fw: 02.03.1"));
    }

    #[test]
    fn a_question_after_the_end_of_a_burst_in_the_same_read_is_found() {
        // A streaming box ends each burst with 00 0a. When the QUESTION comes
        // in the same USB read as the end of a burst, it follows a line end.
        let (mut d, s) = open();
        let mut reply = vec![0x8d, 0x4b, 0x17, 0x00, 0x0a];
        reply.extend_from_slice(b"QUESTION AA006688\r\n");
        s.borrow_mut().question_reply = Some(reply);
        d.authenticate_tries(&mut |_| {}, 1)
            .expect("unlocks on the first try");
        assert_eq!(unlocks(&s), 1);
    }

    #[test]
    fn unlocked_after_another_line_in_the_same_read_is_found() {
        let (mut d, s) = open();
        s.borrow_mut().hook = Some(Box::new(|cmd, s| {
            if cmd.starts_with("ANSWER ") {
                s.rx.pop_back();
                s.push(b"\x8d\x4b\x00\x0aUNLOCKED\r\n");
            }
        }));
        d.authenticate_tries(&mut |_| {}, 1).expect("unlocks");
        assert_eq!(d.firmware(), Some("Fw: 02.03.1"));
    }

    #[test]
    fn data_left_from_the_drain_is_not_read_into_the_question() {
        // Every read ends part way through a line, so the drain always stops
        // with bytes left over. They are dropped before UNLOCK.
        let (mut d, s) = open();
        s.borrow_mut().stream = Some(b"x\ny".to_vec());
        d.authenticate_tries(&mut |_| {}, 1)
            .expect("unlocks on the first try");
    }

    #[test]
    fn a_failed_break_or_purge_fails_the_handshake() {
        // Requests 0 to 8 are the bring-up; 9 and 10 are the first BREAK,
        // 13 the purge.
        for n in [9, 10, 13] {
            let (mut d, s) = open();
            s.borrow_mut().fail_ctrl_at = Some(n);
            assert!(d.authenticate_tries(&mut |_| {}, 1).is_err(), "request {n}");
        }
    }

    #[test]
    fn a_failed_read_or_write_fails_the_handshake() {
        let (mut d, s) = open();
        s.borrow_mut().fail_read = true;
        assert!(d.authenticate_tries(&mut |_| {}, 1).is_err());

        let (mut d, s) = open();
        s.borrow_mut().fail_write = true;
        assert!(d.authenticate_tries(&mut |_| {}, 1).is_err());
        assert_eq!(unlocks(&s), 1);
    }

    #[test]
    fn a_failed_answer_after_the_challenge_fails_the_handshake() {
        let (mut d, s) = open();
        s.borrow_mut().hook = Some(Box::new(|cmd, s| {
            if cmd.starts_with("UNLOCK ") {
                s.fail_write = true;
            }
        }));
        assert!(d.authenticate_tries(&mut |_| {}, 1).is_err());
    }

    // ---- firmware version -------------------------------------------------

    #[test]
    fn the_version_is_found_attached_to_sample_data() {
        let (mut d, s) = open();
        let mut reply = vec![0x8d, 0x4b, 0x17, 0xfc, 0x59];
        reply.extend_from_slice(b"Fw: 02.04.7\r\n");
        s.borrow_mut().version_reply = Some(reply);
        d.authenticate().unwrap();
        assert_eq!(d.firmware(), Some("Fw: 02.04.7"));
        assert_eq!(s.borrow().count("VERSION\r\n"), 1);
    }

    #[test]
    fn the_version_is_asked_for_three_times_at_most() {
        let (mut d, s) = open();
        s.borrow_mut().version_reply = None;
        d.authenticate().expect("unlocks without a version");
        assert_eq!(d.firmware(), None);
        assert_eq!(s.borrow().count("VERSION\r\n"), 3);
    }

    #[test]
    fn a_reply_without_the_marker_is_not_a_version() {
        let (mut d, s) = open();
        s.borrow_mut().version_reply = Some(b"AB12\r\n".to_vec());
        d.authenticate().unwrap();
        assert_eq!(d.firmware(), None);
        assert_eq!(s.borrow().count("VERSION\r\n"), 3);
    }

    #[test]
    fn a_lost_version_reply_is_asked_for_again() {
        let (mut d, s) = open();
        let mut lost = false;
        s.borrow_mut().hook = Some(Box::new(move |cmd, s| {
            if cmd == "VERSION\r\n" && !lost {
                lost = true;
                s.rx.pop_back();
            }
        }));
        d.authenticate().unwrap();
        assert_eq!(d.firmware(), Some("Fw: 02.03.1"));
        assert_eq!(s.borrow().count("VERSION\r\n"), 2);
    }

    #[test]
    fn the_version_is_asked_for_once_per_session() {
        let (mut d, s) = unlocked();
        d.authenticate().unwrap();
        assert_eq!(s.borrow().count("VERSION\r\n"), 1);
    }

    #[test]
    fn a_link_error_while_asking_the_version_leaves_it_unknown() {
        // The write of VERSION fails.
        let (mut d, s) = open();
        s.borrow_mut().hook = Some(Box::new(|cmd, s| {
            if cmd.starts_with("ANSWER ") {
                s.fail_write = true;
            }
        }));
        d.authenticate().expect("unlocked all the same");
        assert_eq!(d.firmware(), None);

        // The read of its reply fails.
        let (mut d, s) = open();
        s.borrow_mut().hook = Some(Box::new(|cmd, s| {
            if cmd == "VERSION\r\n" {
                s.fail_read = true;
            }
        }));
        d.authenticate().expect("unlocked all the same");
        assert_eq!(d.firmware(), None);
        assert_eq!(s.borrow().count("VERSION\r\n"), 1);
    }

    // ---- keepalive --------------------------------------------------------

    #[test]
    fn a_ping_is_due_at_once_and_then_every_two_seconds() {
        let (mut d, s) = unlocked();
        assert!(d.keepalive().unwrap());
        assert!(!d.keepalive().unwrap());
        fake::advance(ms(1999));
        assert!(!d.keepalive().unwrap());
        fake::advance(ms(1));
        assert!(d.keepalive().unwrap());
        assert_eq!(s.borrow().count("PING\r\n"), 2);
    }

    #[test]
    fn a_pong_clears_the_watchdog() {
        let (mut d, _s) = unlocked();
        d.keepalive().unwrap();
        assert_eq!(d.unanswered, 1);
        fake::advance(ms(500));
        assert_eq!(d.poll_burst().unwrap(), None, "PONG read");
        assert!(d.poll_burst().unwrap().is_some(), "and closed by the gap");
        assert_eq!(d.burst(), b"PONG\r\n");
        assert_eq!(d.unanswered, 0);
        assert!(d.since_pong().unwrap() < ms(10));
    }

    #[test]
    fn a_pong_inside_sample_data_clears_the_watchdog() {
        let (mut d, s) = unlocked();
        s.borrow_mut().pong_reply = None;
        d.keepalive().unwrap();
        let mut burst = vec![0xF0; 30];
        burst.extend_from_slice(b"PONG");
        burst.extend_from_slice(&[0x0F; 30]);
        s.borrow_mut().push(&burst);
        d.poll_burst().unwrap();
        d.poll_burst().unwrap();
        assert_eq!(d.unanswered, 0);
    }

    #[test]
    fn five_missed_pongs_run_the_handshake_again() {
        let (mut d, s) = unlocked();
        s.borrow_mut().pong_reply = None;
        for n in 1..=PONG_MISSES {
            assert!(d.keepalive().unwrap());
            assert_eq!(d.unanswered, n);
            fake::advance(PING_INTERVAL);
        }
        s.borrow_mut().push(&[0xF0; 40]);
        d.poll_burst().unwrap();
        assert!(!d.acc.is_empty());

        assert!(
            !d.keepalive().unwrap(),
            "the handshake runs instead of a PING"
        );
        assert_eq!(d.pong_timeouts, 1);
        assert_eq!(d.relock_failures, 0);
        assert_eq!(d.unanswered, 0);
        assert_eq!(unlocks(&s), 2);
        assert!(
            d.acc.is_empty() && d.burst().is_empty(),
            "data from before is dropped"
        );
        assert!(d.is_unlocked());
        assert!(d.keepalive().unwrap(), "and a PING is due at once");
    }

    #[test]
    fn a_watchdog_handshake_that_fails_is_counted() {
        let (mut d, s) = unlocked();
        s.borrow_mut().pong_reply = None;
        for _ in 0..PONG_MISSES {
            d.keepalive().unwrap();
            fake::advance(PING_INTERVAL);
        }
        s.borrow_mut().question_reply = None;
        assert!(!d.keepalive().unwrap());
        assert_eq!(d.pong_timeouts, 1);
        assert_eq!(d.relock_failures, 1);
    }

    #[test]
    fn the_watchdog_waits_for_a_session() {
        let (mut d, s) = open();
        for _ in 0..=PONG_MISSES {
            assert!(d.keepalive().unwrap());
            fake::advance(PING_INTERVAL);
        }
        assert_eq!(d.pong_timeouts, 0);
        assert_eq!(unlocks(&s), 0);
    }

    #[test]
    fn a_failed_ping_is_an_error() {
        let (mut d, s) = unlocked();
        s.borrow_mut().fail_write = true;
        assert!(d.keepalive().is_err());
    }

    #[test]
    fn signal_pulses_the_led_with_a_bare_line_feed() {
        let (d, s) = open();
        d.signal().unwrap();
        assert_eq!(s.borrow().writes(), ["SIGNAL\n"]);
    }

    // ---- bursts -----------------------------------------------------------

    #[test]
    fn a_burst_ends_at_a_gap_of_more_than_four_ms() {
        let (mut d, s) = open();
        assert_eq!(d.poll_burst().unwrap(), None, "nothing to read");
        s.borrow_mut().push(&[1; 30]);
        s.borrow_mut().push(&[2; 30]);
        assert_eq!(d.poll_burst().unwrap(), None);
        assert_eq!(d.poll_burst().unwrap(), None, "1 ms apart: the same burst");
        let t = d
            .poll_burst()
            .unwrap()
            .expect("a 5 ms read with nothing closes it");
        assert_eq!(u128::from(t), (fake::now() - d.t0).as_millis());
        let mut want = vec![1; 30];
        want.extend([2; 30]);
        assert_eq!(d.burst(), want);
        assert_eq!(
            d.poll_burst().unwrap(),
            None,
            "and it is not returned twice"
        );
    }

    #[test]
    fn bytes_after_the_gap_start_the_next_burst() {
        let (mut d, s) = open();
        s.borrow_mut().push(&[1; 30]);
        d.poll_burst().unwrap();
        fake::advance(ms(10));
        s.borrow_mut().push(&[2; 30]);
        assert!(d.poll_burst().unwrap().is_some());
        assert_eq!(d.burst(), [1; 30]);
        assert!(d.poll_burst().unwrap().is_some());
        assert_eq!(d.burst(), [2; 30]);
    }

    #[test]
    fn a_burst_stops_growing_at_its_limit() {
        let (mut d, s) = open();
        s.borrow_mut().push(&[1; 8000]);
        s.borrow_mut().push(&[2; 500]);
        s.borrow_mut().push(&[3; 192]);
        for _ in 0..3 {
            d.poll_burst().unwrap();
        }
        assert!(d.poll_burst().unwrap().is_some());
        assert_eq!(d.burst().len(), MAX_BURST);
        assert_eq!(
            d.burst()[8000..],
            [3; 192],
            "a read that does not fit is dropped whole"
        );
    }

    #[test]
    fn a_failed_read_is_an_error() {
        let (mut d, s) = open();
        s.borrow_mut().fail_read = true;
        assert!(d.poll_burst().is_err());
    }

    // ---- the device ending the session ------------------------------------

    #[test]
    fn a_lock_notice_runs_the_handshake_again() {
        let (mut d, s) = unlocked();
        s.borrow_mut().drop_lock();
        s.borrow_mut().push(&[0xF0; 30]);
        assert_eq!(d.poll_burst().unwrap(), None);
        assert_eq!(d.poll_burst().unwrap(), None);
        assert_eq!(d.poll_burst().unwrap(), None, "a relock yields no burst");
        assert_eq!(d.relocks, 1);
        assert_eq!(d.relock_failures, 0);
        assert_eq!(unlocks(&s), 2);
        assert!(d.is_unlocked() && s.borrow().unlocked);
        assert!(d.burst().is_empty());
    }

    #[test]
    fn a_relock_that_fails_is_counted() {
        let (mut d, s) = unlocked();
        s.borrow_mut().question_reply = None;
        s.borrow_mut().drop_lock();
        d.poll_burst().unwrap();
        assert_eq!(d.poll_burst().unwrap(), None);
        assert_eq!(d.relocks, 1);
        assert_eq!(d.relock_failures, 1);
    }

    #[test]
    fn the_power_on_banner_is_not_a_relock() {
        let (mut d, s) = open();
        s.borrow_mut().push(b"LOCKED\r\n");
        d.poll_burst().unwrap();
        assert!(d.poll_burst().unwrap().is_some());
        assert_eq!(d.burst(), b"LOCKED\r\n");
        assert_eq!(d.relocks, 0);
        assert_eq!(unlocks(&s), 0);
    }

    // ---- shutdown ---------------------------------------------------------

    #[test]
    fn lock_ends_the_session_and_counts_the_bytes_in_flight() {
        let (mut d, s) = unlocked();
        s.borrow_mut().push(&[0xF0; 10]);
        s.borrow_mut().push(&[0xF0; 30]);
        d.poll_burst().unwrap();
        let t = fake::now();
        let l = d.lock().unwrap();
        assert_eq!(
            l,
            Locked {
                stopped: true,
                bytes: 30
            }
        );
        assert_eq!(s.borrow().writes().last().unwrap(), "LOCK\n");
        assert!(!d.is_unlocked() && !s.borrow().unlocked);
        assert!(d.burst().is_empty() && d.acc.is_empty());
        // 1 ms for the read with data, then 300 ms of quiet in 50 ms reads.
        assert_eq!(fake::now() - t, ms(301));
    }

    #[test]
    fn lock_gives_up_on_a_stream_that_does_not_stop() {
        let (mut d, s) = unlocked();
        s.borrow_mut().stream = Some(vec![0xF0; 62]);
        s.borrow_mut().stream_after_lock = true;
        let l = d.lock().unwrap();
        assert!(!l.stopped);
        assert_eq!(l.bytes, 62 * 1000, "one read a millisecond for one second");
    }

    #[test]
    fn a_stream_stops_at_lock() {
        let (mut d, s) = unlocked();
        s.borrow_mut().stream = Some(vec![0xF0; 62]);
        assert_eq!(
            d.lock().unwrap(),
            Locked {
                stopped: true,
                bytes: 0
            }
        );
    }

    #[test]
    fn lock_reports_link_errors() {
        let (mut d, s) = unlocked();
        s.borrow_mut().fail_write = true;
        assert!(d.lock().is_err());

        let (mut d, s) = unlocked();
        s.borrow_mut().fail_read = true;
        assert!(d.lock().is_err());
    }

    #[test]
    fn dropping_an_open_session_locks_it() {
        let (d, s) = unlocked();
        drop(d);
        assert_eq!(s.borrow().count("LOCK\n"), 1);

        let (d, s) = open();
        drop(d);
        assert_eq!(s.borrow().count("LOCK\n"), 0, "no session, nothing to lock");
    }
}
