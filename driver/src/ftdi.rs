//! FTDI FT232R vendor protocol over libusb.
//!
//! The RadarBox's product id is not in the kernel's `ftdi_sio` table, so no
//! `/dev/ttyUSB*` appears and the vendor requests are issued directly.

use rusb::{DeviceHandle, GlobalContext};
use std::time::{Duration, Instant};

const REQ_RESET: u8 = 0x00;
const REQ_SET_MODEM_CTRL: u8 = 0x01;
const REQ_SET_FLOW_CTRL: u8 = 0x02;
const REQ_SET_BAUDRATE: u8 = 0x03;
const REQ_SET_DATA: u8 = 0x04;
const REQ_SET_LATENCY: u8 = 0x09;

const TYPE_OUT: u8 = 0x40; // vendor, host-to-device, device recipient

/// Bulk endpoints of interface 0.
const EP_OUT: u8 = 0x02;
const EP_IN: u8 = 0x81;

/// Line-control word for 8 data bits, no parity, 1 stop bit.
const LINE_8N1: u16 = 0x0008;
/// Line-control bit that holds the line in the break state.
const LINE_BREAK: u16 = 0x4000;

/// USB packet size. Every IN packet starts with two modem-status bytes.
const PACKET: usize = 64;

/// Timeout for control requests and bulk writes.
const TIMEOUT: Duration = Duration::from_millis(300);

pub const DISABLE_FLOW_CTRL: u16 = 0x0000;

pub struct Ftdi {
    handle: DeviceHandle<GlobalContext>,
    /// Cached line-control word so the BREAK bit can be toggled without
    /// disturbing the data/parity/stop settings.
    data_cfg: u16,
}

/// Encode a baud rate as the FT232R's fractional divisor.
///
/// The part divides a 3 MHz clock by an integer plus one of eight fractions,
/// so most requested rates are only approximated. 1250000 becomes 1263158
/// (divisor 2.375), 1.05% high, which 8N1 framing absorbs without trouble.
fn encode_baud(baud: u32) -> (u16, u16, u32) {
    /// Encoding of the eighths: the part accepts 0, 1/8, 2/8 ... 7/8 but not
    /// in numeric order.
    const DIVFRAC: [u32; 8] = [0, 3, 2, 4, 1, 5, 6, 7];
    if baud == 0 {
        return (0, 0, 0);
    }
    let (encoded, actual) = if baud >= 3_000_000 {
        (0u32, 3_000_000u32) // divisor 0 means 3 MBaud exactly
    } else if baud >= 2_000_000 {
        (1u32, 2_000_000u32) // divisor 1 is a special case
    } else {
        // Divisor in eighths, rounded to nearest.
        let mut div8 = (24_000_000u32 + baud / 2) / baud;
        if div8 < 8 {
            div8 = 8; // clamp to divisor >= 1
        }
        let enc = (div8 >> 3) | (DIVFRAC[(div8 & 7) as usize] << 14);
        let act = (24_000_000u32 + div8 / 2) / div8;
        (enc, act)
    };
    ((encoded & 0xFFFF) as u16, (encoded >> 16) as u16, actual)
}

/// One vendor control request: the request code and its value and index
/// words. None of the requests here carries data.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Ctrl {
    pub request: u8,
    pub value: u16,
    pub index: u16,
}

impl Ctrl {
    const fn new(request: u8, value: u16, index: u16) -> Self {
        Ctrl {
            request,
            value,
            index,
        }
    }

    /// SIO reset of the part.
    pub const fn reset() -> Self {
        Self::new(REQ_RESET, 0, 1)
    }
    /// Clear the receive buffer.
    pub const fn purge_rx() -> Self {
        Self::new(REQ_RESET, 1, 1)
    }
    /// Clear the transmit buffer.
    pub const fn purge_tx() -> Self {
        Self::new(REQ_RESET, 2, 1)
    }
    /// Set the baud rate to the nearest one the part can produce.
    pub fn baudrate(baud: u32) -> Self {
        let (value, index, _) = encode_baud(baud);
        Self::new(REQ_SET_BAUDRATE, value, index)
    }
    /// The line-control word `cfg`, with the BREAK bit set if `brk`.
    pub const fn line(cfg: u16, brk: bool) -> Self {
        Self::new(REQ_SET_DATA, if brk { cfg | LINE_BREAK } else { cfg }, 1)
    }
    /// 8 data bits, no parity, 1 stop bit, and the BREAK bit set if `brk`.
    pub const fn line_8n1(brk: bool) -> Self {
        Self::line(LINE_8N1, brk)
    }
    /// The latency timer, in milliseconds.
    pub const fn latency(ms: u8) -> Self {
        Self::new(REQ_SET_LATENCY, ms as u16, 1)
    }
    /// The flow-control mode. The low bit of the index is the interface.
    pub const fn flowctrl(mode: u16) -> Self {
        Self::new(REQ_SET_FLOW_CTRL, 0, mode | 1)
    }
    /// Drive DTR high or low. The high byte of the value says DTR is set.
    pub const fn dtr(on: bool) -> Self {
        Self::new(REQ_SET_MODEM_CTRL, if on { 0x0101 } else { 0x0100 }, 1)
    }
    /// Drive RTS high or low. The high byte of the value says RTS is set.
    pub const fn rts(on: bool) -> Self {
        Self::new(REQ_SET_MODEM_CTRL, if on { 0x0202 } else { 0x0200 }, 1)
    }
}

/// The serial link to the box: what the driver needs from the FTDI part, and
/// the clock its waits are measured with.
///
/// [`Ftdi`] is the real link, and the default everywhere. The driver is
/// generic over this trait rather than holding a trait object, so each call
/// on real hardware goes straight to [`Ftdi`].
pub trait Port: Sized {
    /// Open the device with this vendor and product id and claim interface 0.
    fn open(vid: u16, pid: u16) -> rusb::Result<Self>;
    /// USB port reset. On this board it power-cycles the MCU.
    fn reset_device(&mut self) -> rusb::Result<()>;
    /// SIO reset of the part.
    fn reset(&self) -> rusb::Result<()>;
    /// Clear the receive buffer, then the transmit buffer.
    fn purge(&self) -> rusb::Result<()>;
    /// Returns the rate the part will produce, which is rarely the
    /// one asked for.
    fn set_baudrate(&self, baud: u32) -> rusb::Result<u32>;
    /// 8 data bits, no parity, 1 stop bit.
    fn set_line_8n1(&mut self) -> rusb::Result<()>;
    /// Hold the line in the break state, or release it.
    fn set_break(&self, on: bool) -> rusb::Result<()>;
    /// The latency timer, in milliseconds.
    fn set_latency(&self, ms: u8) -> rusb::Result<()>;
    /// Must stay disabled: RTS# is wired to the MCU reset on this board, and
    /// letting the FTDI drive it automatically resets the receiver.
    fn set_flowctrl(&self, mode: u16) -> rusb::Result<()>;
    fn set_dtr(&self, on: bool) -> rusb::Result<()>;
    fn set_rts(&self, on: bool) -> rusb::Result<()>;
    /// Write to the device, waiting at most 300 ms.
    fn write(&self, data: &[u8]) -> rusb::Result<usize>;
    /// Append the payload bytes that arrive within `timeout_ms` to `out`.
    /// Nothing arriving is not an error.
    fn read(&self, out: &mut Vec<u8>, timeout_ms: u64) -> rusb::Result<()>;

    /// The current time, which every deadline in the driver is measured
    /// against.
    fn now() -> Instant {
        Instant::now()
    }
    /// Wait for `d`.
    fn sleep(d: Duration) {
        std::thread::sleep(d)
    }
}

impl Ftdi {
    /// Explain an Access error in terms of what the platform needs, since the
    /// remedy differs and the bare errno says nothing useful.
    pub fn access_hint() -> &'static str {
        if cfg!(target_os = "linux") {
            "permission denied claiming the interface: install deploy/99-anrb.rules, or check that ftdi_sio has not bound the device (lsmod | grep ftdi_sio)"
        } else if cfg!(target_os = "macos") {
            "permission denied claiming the interface: a kernel extension holds it. Check with `ioreg -p IOUSB -l | grep -i ftdi`; if AppleUSBFTDI has bound the device it must be unloaded, as libusb cannot detach a driver on macOS"
        } else {
            "permission denied claiming the interface"
        }
    }

    fn ctrl(&self, c: Ctrl) -> rusb::Result<()> {
        self.handle
            .write_control(TYPE_OUT, c.request, c.value, c.index, &[], TIMEOUT)
            .map(|_| ())
    }
}

impl Port for Ftdi {
    fn open(vid: u16, pid: u16) -> rusb::Result<Self> {
        let handle = rusb::open_device_with_vid_pid(vid, pid).ok_or(rusb::Error::NoDevice)?;
        // Linux only; other platforms return NotSupported, which is ignored
        // because no kernel driver binds this product id there.
        match handle.set_auto_detach_kernel_driver(true) {
            Ok(()) | Err(rusb::Error::NotSupported) => {}
            Err(e) => return Err(e),
        }
        handle.claim_interface(0)?;
        Ok(Ftdi {
            handle,
            data_cfg: LINE_8N1,
        })
    }

    fn reset_device(&mut self) -> rusb::Result<()> {
        self.handle.reset()
    }

    fn reset(&self) -> rusb::Result<()> {
        self.ctrl(Ctrl::reset())
    }
    fn purge(&self) -> rusb::Result<()> {
        self.ctrl(Ctrl::purge_rx())?;
        self.ctrl(Ctrl::purge_tx())
    }

    fn set_baudrate(&self, baud: u32) -> rusb::Result<u32> {
        self.ctrl(Ctrl::baudrate(baud))?;
        Ok(encode_baud(baud).2)
    }

    fn set_line_8n1(&mut self) -> rusb::Result<()> {
        self.data_cfg = LINE_8N1;
        self.ctrl(Ctrl::line(self.data_cfg, false))
    }

    fn set_break(&self, on: bool) -> rusb::Result<()> {
        self.ctrl(Ctrl::line(self.data_cfg, on))
    }

    fn set_latency(&self, ms: u8) -> rusb::Result<()> {
        self.ctrl(Ctrl::latency(ms))
    }
    fn set_flowctrl(&self, mode: u16) -> rusb::Result<()> {
        self.ctrl(Ctrl::flowctrl(mode))
    }
    fn set_dtr(&self, on: bool) -> rusb::Result<()> {
        self.ctrl(Ctrl::dtr(on))
    }
    fn set_rts(&self, on: bool) -> rusb::Result<()> {
        self.ctrl(Ctrl::rts(on))
    }

    fn write(&self, data: &[u8]) -> rusb::Result<usize> {
        self.handle.write_bulk(EP_OUT, data, TIMEOUT)
    }

    fn read(&self, out: &mut Vec<u8>, timeout_ms: u64) -> rusb::Result<()> {
        let mut raw = [0u8; 4096];
        let n = match self
            .handle
            .read_bulk(EP_IN, &mut raw, Duration::from_millis(timeout_ms))
        {
            Ok(n) => n,
            Err(rusb::Error::Timeout) => 0,
            Err(e) => return Err(e),
        };
        strip_status(&raw[..n], out);
        Ok(())
    }
}

/// Append the payload of `raw`, a run of USB packets, to `out`. Each packet is
/// up to 64 bytes and starts with two modem-status bytes, which are dropped.
fn strip_status(raw: &[u8], out: &mut Vec<u8>) {
    let n = raw.len();
    let mut i = 0usize;
    while i + 2 <= n {
        let chunk = (n - i).min(PACKET);
        if chunk > 2 {
            out.extend_from_slice(&raw[i + 2..i + chunk]);
        }
        i += chunk;
    }
}

impl Drop for Ftdi {
    fn drop(&mut self) {
        let _ = self.handle.release_interface(0);
    }
}

#[cfg(test)]
pub(crate) mod fake {
    //! A scripted RadarBox behind [`Port`], for tests.
    //!
    //! It answers the way the box does: QUESTION after UNLOCK, UNLOCKED after
    //! the right ANSWER, the firmware string after VERSION, PONG after PING,
    //! and silence after LOCK. Each reply can be changed or removed. Every
    //! control request and every write is recorded.
    //!
    //! Time is virtual and belongs to the test thread. Sleeping moves it on at
    //! once, and so does a read that finds nothing: it takes its whole
    //! timeout, as a USB read does. A read that returns data takes 1 ms.

    use super::*;
    use crate::protocol;
    use std::cell::{Cell, RefCell};
    use std::collections::VecDeque;
    use std::rc::Rc;

    thread_local! {
        static BASE: Instant = Instant::now();
        static OFFSET: Cell<Duration> = const { Cell::new(Duration::ZERO) };
        static OPENS: RefCell<VecDeque<rusb::Result<FakePort>>> =
            const { RefCell::new(VecDeque::new()) };
    }

    /// The virtual time on this thread.
    pub fn now() -> Instant {
        BASE.with(|b| *b) + OFFSET.with(Cell::get)
    }

    /// Move the virtual time on by `d`.
    pub fn advance(d: Duration) {
        OFFSET.with(|o| o.set(o.get() + d))
    }

    /// Queue the result of the next [`Port::open`]. Once the queue is empty,
    /// opening fails with `NoDevice`.
    pub fn expect_open(r: rusb::Result<FakePort>) {
        OPENS.with(|q| q.borrow_mut().push_back(r))
    }

    /// One thing the host did to the device.
    #[derive(Clone, Debug, PartialEq, Eq)]
    pub enum Ev {
        Ctrl(Ctrl),
        Write(String),
        ResetDevice,
    }

    /// What the device does, and what was done to it.
    pub struct Script {
        /// Every control request, write and port reset, in order.
        pub log: Vec<Ev>,
        /// The virtual time of each entry in `log`.
        pub at: Vec<Instant>,
        /// Called after each write has been answered, with the text written.
        /// It can change the script from then on.
        #[allow(clippy::type_complexity)]
        pub hook: Option<Box<dyn FnMut(&str, &mut Script)>>,
        /// Data waiting to be read, one entry per read.
        pub rx: VecDeque<Vec<u8>>,
        /// Returned by every read that finds `rx` empty, until LOCK.
        pub stream: Option<Vec<u8>>,
        /// Keep sending `stream` after LOCK.
        pub stream_after_lock: bool,
        /// The challenge, which decides the ANSWER the device accepts.
        pub question: u32,
        /// The reply to UNLOCK. `None` is silence.
        pub question_reply: Option<Vec<u8>>,
        /// UNLOCKs ignored before the device starts answering, as by a part
        /// that is wedged. A USB port reset sets it back to 0.
        pub ignore_unlocks: u32,
        /// Answer the right ANSWER with UNLOCKED. When false, or when the
        /// ANSWER is wrong, the device says nothing.
        pub accept: bool,
        /// The reply to VERSION. `None` is silence.
        pub version_reply: Option<Vec<u8>>,
        /// The reply to PING. `None` is silence.
        pub pong_reply: Option<Vec<u8>>,
        /// The session is open.
        pub unlocked: bool,
        /// The control request with this index, counted from 0, fails.
        pub fail_ctrl_at: Option<usize>,
        /// Every write fails.
        pub fail_write: bool,
        /// Every read fails.
        pub fail_read: bool,
        data_cfg: u16,
        ctrls: usize,
    }

    impl Script {
        /// Queue `data` as the result of one read.
        pub fn push(&mut self, data: &[u8]) {
            self.rx.push_back(data.to_vec())
        }

        /// The session drops on the device's side, and it says LOCKED.
        pub fn drop_lock(&mut self) {
            self.unlocked = false;
            self.push(b"LOCKED\r\n");
        }

        /// The control requests, in order.
        pub fn ctrls(&self) -> Vec<Ctrl> {
            self.log
                .iter()
                .filter_map(|e| match e {
                    Ev::Ctrl(c) => Some(*c),
                    _ => None,
                })
                .collect()
        }

        /// The writes, in order.
        pub fn writes(&self) -> Vec<String> {
            self.log
                .iter()
                .filter_map(|e| match e {
                    Ev::Write(w) => Some(w.clone()),
                    _ => None,
                })
                .collect()
        }

        /// How many writes were exactly `cmd`.
        pub fn count(&self, cmd: &str) -> usize {
            self.writes().iter().filter(|w| *w == cmd).count()
        }

        fn ctrl(&mut self, c: Ctrl) -> rusb::Result<()> {
            let n = self.ctrls;
            self.ctrls += 1;
            self.record(Ev::Ctrl(c));
            if self.fail_ctrl_at == Some(n) {
                Err(rusb::Error::Pipe)
            } else {
                Ok(())
            }
        }

        fn record(&mut self, e: Ev) {
            self.log.push(e);
            self.at.push(now());
        }

        fn command(&mut self, cmd: &str) {
            let cmd = cmd.trim_end();
            if cmd.starts_with("UNLOCK ") {
                if self.ignore_unlocks > 0 {
                    self.ignore_unlocks -= 1;
                } else if let Some(r) = self.question_reply.clone() {
                    self.rx.push_back(r);
                }
            } else if let Some(a) = cmd.strip_prefix("ANSWER ") {
                let right = format!("{:08X}", protocol::hs_answer(self.question));
                if self.accept && a == right {
                    self.unlocked = true;
                    self.push(b"UNLOCKED\r\n");
                }
            } else if cmd == protocol::VERSION {
                if let Some(r) = self.version_reply.clone() {
                    self.rx.push_back(r);
                }
            } else if cmd == protocol::PING {
                if let Some(r) = self.pong_reply.clone() {
                    self.rx.push_back(r);
                }
            } else if cmd == protocol::LOCK {
                self.unlocked = false;
                if !self.stream_after_lock {
                    self.stream = None;
                }
            }
        }
    }

    /// The fake device's end of the link. Tests keep a second handle on the
    /// [`Script`] to steer it and to read what happened.
    pub struct FakePort(Rc<RefCell<Script>>);

    impl FakePort {
        /// A device that answers everything correctly, with challenge
        /// 0xAA006688 and firmware "Fw: 02.03.1".
        pub fn new() -> (FakePort, Rc<RefCell<Script>>) {
            let q = 0xAA00_6688;
            let s = Rc::new(RefCell::new(Script {
                log: Vec::new(),
                at: Vec::new(),
                hook: None,
                rx: VecDeque::new(),
                stream: None,
                stream_after_lock: false,
                question: q,
                question_reply: Some(format!("QUESTION {q:08X}\r\n").into_bytes()),
                ignore_unlocks: 0,
                accept: true,
                version_reply: Some(b"Fw: 02.03.1\r\n".to_vec()),
                pong_reply: Some(b"PONG\r\n".to_vec()),
                unlocked: false,
                fail_ctrl_at: None,
                fail_write: false,
                fail_read: false,
                data_cfg: LINE_8N1,
                ctrls: 0,
            }));
            (FakePort(s.clone()), s)
        }

        /// A second port on the same device, as a second open finds it.
        pub fn again(s: &Rc<RefCell<Script>>) -> FakePort {
            FakePort(s.clone())
        }
    }

    impl Port for FakePort {
        fn open(_vid: u16, _pid: u16) -> rusb::Result<Self> {
            OPENS
                .with(|q| q.borrow_mut().pop_front())
                .unwrap_or(Err(rusb::Error::NoDevice))
        }
        fn reset_device(&mut self) -> rusb::Result<()> {
            // The MCU loses power: the session, the stream and anything
            // waiting to be read are gone, and it boots ready to answer.
            let mut s = self.0.borrow_mut();
            s.record(Ev::ResetDevice);
            s.unlocked = false;
            s.stream = None;
            s.rx.clear();
            s.ignore_unlocks = 0;
            Ok(())
        }
        fn reset(&self) -> rusb::Result<()> {
            self.0.borrow_mut().ctrl(Ctrl::reset())
        }
        fn purge(&self) -> rusb::Result<()> {
            let mut s = self.0.borrow_mut();
            s.ctrl(Ctrl::purge_rx())?;
            s.rx.clear();
            s.ctrl(Ctrl::purge_tx())
        }
        fn set_baudrate(&self, baud: u32) -> rusb::Result<u32> {
            self.0.borrow_mut().ctrl(Ctrl::baudrate(baud))?;
            Ok(encode_baud(baud).2)
        }
        fn set_line_8n1(&mut self) -> rusb::Result<()> {
            let mut s = self.0.borrow_mut();
            s.data_cfg = LINE_8N1;
            let c = Ctrl::line(s.data_cfg, false);
            s.ctrl(c)
        }
        fn set_break(&self, on: bool) -> rusb::Result<()> {
            let mut s = self.0.borrow_mut();
            let c = Ctrl::line(s.data_cfg, on);
            s.ctrl(c)
        }
        fn set_latency(&self, ms: u8) -> rusb::Result<()> {
            self.0.borrow_mut().ctrl(Ctrl::latency(ms))
        }
        fn set_flowctrl(&self, mode: u16) -> rusb::Result<()> {
            self.0.borrow_mut().ctrl(Ctrl::flowctrl(mode))
        }
        fn set_dtr(&self, on: bool) -> rusb::Result<()> {
            self.0.borrow_mut().ctrl(Ctrl::dtr(on))
        }
        fn set_rts(&self, on: bool) -> rusb::Result<()> {
            self.0.borrow_mut().ctrl(Ctrl::rts(on))
        }

        fn write(&self, data: &[u8]) -> rusb::Result<usize> {
            let mut s = self.0.borrow_mut();
            let text = String::from_utf8_lossy(data).into_owned();
            s.record(Ev::Write(text.clone()));
            if s.fail_write {
                return Err(rusb::Error::Io);
            }
            s.command(&text);
            if let Some(mut hook) = s.hook.take() {
                hook(&text, &mut s);
                s.hook.get_or_insert(hook);
            }
            Ok(data.len())
        }

        fn read(&self, out: &mut Vec<u8>, timeout_ms: u64) -> rusb::Result<()> {
            let mut s = self.0.borrow_mut();
            if s.fail_read {
                return Err(rusb::Error::Io);
            }
            if let Some(d) = s.rx.pop_front().or_else(|| s.stream.clone()) {
                out.extend_from_slice(&d);
                advance(Duration::from_millis(1));
            } else {
                advance(Duration::from_millis(timeout_ms));
            }
            Ok(())
        }

        fn now() -> Instant {
            now()
        }
        fn sleep(d: Duration) {
            advance(d)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn the_receiver_rate_encodes_as_the_part_expects() {
        // 1250000 is divisor 2 + 3/8: integer 2, and 3/8 is code 4 in the
        // fraction table, in bits 14 to 16.
        assert_eq!(encode_baud(1_250_000), (0x0002, 0x0001, 1_263_158));
    }

    #[test]
    fn baud_rates_at_and_above_two_megabaud_are_special_cases() {
        assert_eq!(encode_baud(3_000_000), (0, 0, 3_000_000));
        assert_eq!(encode_baud(4_000_000), (0, 0, 3_000_000));
        assert_eq!(encode_baud(2_000_000), (1, 0, 2_000_000));
        assert_eq!(encode_baud(2_999_999), (1, 0, 2_000_000));
        assert_eq!(encode_baud(0), (0, 0, 0));
    }

    #[test]
    fn ordinary_baud_rates_round_to_the_nearest_eighth() {
        // 24 MHz / 115200 = 208.33 eighths, so 208: divisor 26, no fraction.
        assert_eq!(encode_baud(115_200), (26, 0, 115_385));
        // 9600 is exactly divisor 312.5: 2500 eighths, fraction 4/8 is code 1.
        assert_eq!(encode_baud(9_600), (312 | 1 << 14, 0, 9_600));
        // Every fraction code lands where the table puts it.
        for (eighths, code) in [0u32, 3, 2, 4, 1, 5, 6, 7].iter().enumerate() {
            let div8 = 8 * 20 + eighths as u32;
            let baud = 24_000_000 / div8;
            let (value, index, _) = encode_baud(baud);
            let enc = u32::from(value) | u32::from(index) << 16;
            assert_eq!(enc, 20 | code << 14, "{eighths}/8");
        }
    }

    #[test]
    fn control_requests_carry_the_vendor_values() {
        let c = |request, value, index| Ctrl {
            request,
            value,
            index,
        };
        assert_eq!(Ctrl::reset(), c(0x00, 0, 1));
        assert_eq!(Ctrl::purge_rx(), c(0x00, 1, 1));
        assert_eq!(Ctrl::purge_tx(), c(0x00, 2, 1));
        assert_eq!(Ctrl::baudrate(1_250_000), c(0x03, 0x0002, 0x0001));
        assert_eq!(Ctrl::line_8n1(false), c(0x04, 0x0008, 1));
        assert_eq!(Ctrl::line_8n1(true), c(0x04, 0x4008, 1));
        assert_eq!(Ctrl::line(0x0107, true), c(0x04, 0x4107, 1));
        assert_eq!(Ctrl::latency(2), c(0x09, 2, 1));
        assert_eq!(Ctrl::flowctrl(DISABLE_FLOW_CTRL), c(0x02, 0, 1));
        assert_eq!(Ctrl::flowctrl(0x0100), c(0x02, 0, 0x0101));
        assert_eq!(Ctrl::dtr(true), c(0x01, 0x0101, 1));
        assert_eq!(Ctrl::dtr(false), c(0x01, 0x0100, 1));
        assert_eq!(Ctrl::rts(true), c(0x01, 0x0202, 1));
        assert_eq!(Ctrl::rts(false), c(0x01, 0x0200, 1));
    }

    #[test]
    fn status_bytes_are_dropped_from_every_packet() {
        let raw: Vec<u8> = (0..150u32).map(|i| i as u8).collect();
        let mut out = vec![0xEE];
        strip_status(&raw, &mut out);
        let mut want = vec![0xEE];
        want.extend(2..64u8);
        want.extend(66..128u8);
        want.extend(130..150u8);
        assert_eq!(out, want);
    }

    #[test]
    fn packets_with_no_payload_add_nothing() {
        let mut out = Vec::new();
        strip_status(&[], &mut out);
        strip_status(&[0x01], &mut out);
        strip_status(&[0x01, 0x60], &mut out);
        assert!(out.is_empty());
        // A full packet followed by a status-only one.
        let mut raw = vec![0u8; 66];
        raw[2] = 0xAB;
        strip_status(&raw, &mut out);
        assert_eq!(out.len(), 62);
        assert_eq!(out[0], 0xAB);
    }

    #[test]
    fn the_real_link_uses_the_system_clock() {
        let before = Instant::now();
        <Ftdi as Port>::sleep(Duration::from_millis(1));
        assert!(<Ftdi as Port>::now() >= before + Duration::from_millis(1));
    }

    #[test]
    fn the_access_hint_names_a_remedy() {
        assert!(Ftdi::access_hint().starts_with("permission denied claiming the interface"));
    }
}
