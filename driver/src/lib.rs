//! AirNav RadarBox driver and Mode-S/ADS-B decoder.
//!
//! The decoding half works with no device attached, so a recorded capture
//! runs through the same code as the live receiver:
//!
//! * [`device`], [`ftdi`] - the driver: USB bring-up, the three-way handshake,
//!   the keepalive, and reassembly of the gap-delimited burst stream.
//! * [`protocol`] - what the box says besides Mode-S samples: the handshake,
//!   its text replies, and the framing of its binary stream.
//! * [`decode`], [`stages`], [`crc`], [`simd`] - Mode-S decoding: samples in,
//!   frames out, with no knowledge of the device and no dependency on USB.
//! * [`tracker`] - turning frames into aircraft: positions, altitudes,
//!   velocities, callsigns.
//! * [`sbs`], [`beast`], [`feed`], [`tui`] - what the receiver does with
//!   them: BaseStation and Beast feeds over TCP, and a terminal dashboard.
//! * [`corpus`] - the recorded-capture formats, and a fingerprint over the
//!   frames decoded from them.
//! * [`clock`] - local wall-clock time for the handshake, SBS and the log.
//!
//! The output is a stream of [`Frame`]s:
//!
//! ```no_run
//! # #[cfg(feature = "usb")]
//! # fn main() -> std::io::Result<()> {
//! use anrb::RadarBox;
//! for frame in RadarBox::open()?.frames() {
//!     let frame = frame?;
//!     println!("{} DF{} {:06X}", frame.hex(), frame.df(), frame.icao());
//! }
//! # Ok(()) }
//! # #[cfg(not(feature = "usb"))]
//! # fn main() {}
//! ```
//!
//! The same decoder runs over recorded bytes with no hardware:
//!
//! ```
//! use anrb::Decoder;
//! let mut dec = Decoder::new();
//! let frames = dec.feed(&[0u8; 128], 0);
//! assert!(frames.is_empty());
//! ```

//! # The live receiver
//!
//! `anrb-rx` shows a dashboard when stdout is a terminal and prints one line
//! per frame when it is not, so its output can be piped or redirected.
//!
//! ```text
//! anrb-rx --seconds 900 --sbs 30003     # dashboard, and a BaseStation feed
//! anrb-rx --plain > frames.log          # line output, forced
//! ```
//!
//! The dashboard carries the device's own state - locked or unlocked, the
//! firmware string, how long since it last answered a keepalive - alongside the
//! decode rates, how many frames needed one or two bits corrected, the aircraft
//! being tracked, and the readers attached to the SBS port.

// Every unsafe block says why it is sound, next to it. What remains is the
// SIMD dispatch (the CPU check is the proof), the kernels' unaligned loads and
// stores (each through a window exactly the width it touches) and rdtsc in
// profiling builds.
#![deny(clippy::undocumented_unsafe_blocks)]

pub mod beast;
pub mod clock;
pub mod corpus;
pub mod crc;
pub mod decode;
#[cfg(feature = "usb")]
pub mod device;
pub mod feed;
#[cfg(feature = "usb")]
pub mod ftdi;
pub(crate) mod profile;
pub mod protocol;
#[cfg(feature = "usb")]
pub mod sbs;
pub mod simd;
pub mod stages;
pub mod tracker;
#[cfg(feature = "usb")]
pub mod tui;

pub use decode::{Decoder, Frame, Options, Stats};
#[cfg(feature = "usb")]
pub use device::{Device, Locked};
pub use simd::Backend;
pub use tracker::{Aircraft, Tracker, Update};

// The one place the device's framing meets the Mode-S decoder: `protocol`
// knows how the box delimits bursts, `decode` knows what a burst of samples
// holds, and neither knows about the other.
impl Decoder {
    /// Decode every burst in one buffer of device output, appending recovered
    /// frames. A keepalive reply decodes to nothing.
    pub fn feed_into(&mut self, buf: &[u8], ms: u32, out: &mut Vec<Frame>) -> usize {
        let before = out.len();
        self.decode_buffer(buf, ms, |f| out.push(f));
        out.len() - before
    }

    /// Decode every burst in one buffer of device output, handing each
    /// recovered frame to `each`.
    fn decode_buffer(&mut self, buf: &[u8], ms: u32, mut each: impl FnMut(Frame)) {
        if protocol::is_pong(buf) {
            return;
        }
        for (off, len) in protocol::segments(buf) {
            if let Some(f) = self.decode_burst(&buf[off..off + len], ms) {
                each(f);
            }
        }
    }

    /// Convenience form that allocates.
    pub fn feed(&mut self, buf: &[u8], ms: u32) -> Vec<Frame> {
        let mut v = Vec::new();
        self.feed_into(buf, ms, &mut v);
        v
    }
}

#[cfg(feature = "usb")]
mod live {
    use super::*;
    use crate::ftdi::{Ftdi, Port};
    use std::collections::VecDeque;
    use std::io;
    use std::time::{Duration, Instant};

    /// A connected receiver, already authenticated.
    ///
    /// `P` is the USB link, [`Ftdi`] unless a test puts something else there.
    pub struct RadarBox<P: Port = Ftdi> {
        dev: Device<P>,
        dec: Decoder,
        raw_log: Option<std::fs::File>,
        /// The first error writing `raw_log`, after which recording stopped.
        raw_log_error: Option<io::Error>,
    }

    impl RadarBox {
        /// Open the device, bring up the link and authenticate.
        pub fn open() -> io::Result<Self> {
            Self::open_with(&mut |_| {})
        }

        /// As [`RadarBox::open`], reporting each phase of the bring-up.
        ///
        /// Opening takes about two seconds - a BREAK, 1.5 s of settling, then the
        /// challenge - and about five more in the rare case the device has to be
        /// power-cycled, so anything with a display should show what is happening
        /// rather than leave the screen empty until it finishes.
        pub fn open_with(progress: &mut dyn FnMut(&str)) -> io::Result<Self> {
            Self::open_port(progress)
        }
    }

    impl<P: Port> RadarBox<P> {
        pub(crate) fn open_port(progress: &mut dyn FnMut(&str)) -> io::Result<Self> {
            progress("opening the USB device");
            // Attach to the box as it is. This answers in under two seconds
            // whether the last session ended cleanly, was killed mid-stream, or
            // never happened, so it is worth two attempts before reaching for the
            // reset that costs about five extra seconds of MCU boot (4.8 measured).
            let mut dev = Device::<P>::open_inner(false)?;
            if dev.authenticate_tries(progress, 2).is_err() {
                progress("no answer; power-cycling the device");
                drop(dev);
                dev = Device::<P>::open_inner(true)?;
                dev.authenticate_tries(progress, 6)?;
            }
            Ok(RadarBox {
                dev,
                dec: Decoder::new(),
                raw_log: None,
                raw_log_error: None,
            })
        }

        /// Choose which decoder passes run.
        pub fn with_options(mut self, opts: Options) -> Self {
            self.dec.opts = opts;
            self
        }

        /// Record every burst to a file that `anrb-replay` and
        /// [`corpus::raw_log`] read back.
        /// Header [`corpus::RAW_MAGIC`], then per burst [u32 ms][u16 len][len bytes].
        /// If a write fails, recording stops and [`Frames::record_error`] returns
        /// the error.
        pub fn record_to(mut self, path: &str) -> io::Result<Self> {
            use std::io::Write;
            let mut f = std::fs::File::create(path)?;
            f.write_all(corpus::RAW_MAGIC)?;
            self.raw_log = Some(f);
            Ok(self)
        }

        /// Consume the receiver and yield frames as they are decoded.
        ///
        /// The iterator blocks waiting for the device and never ends on its own;
        /// use [`Frames::until`] to bound it, or stop iterating.
        pub fn frames(self) -> Frames<P> {
            Frames {
                rb: self,
                queue: VecDeque::new(),
                deadline: None,
                flash: true,
            }
        }
    }

    /// A stream of decoded ADS-B/Mode-S frames.
    pub struct Frames<P: Port = Ftdi> {
        rb: RadarBox<P>,
        queue: VecDeque<Frame>,
        deadline: Option<Instant>,
        flash: bool,
    }

    impl<P: Port> Frames<P> {
        /// Stop yielding after `d` has elapsed.
        pub fn until(mut self, d: Duration) -> Self {
            self.deadline = Some(P::now() + d);
            self
        }
        /// Whether to pulse the front-panel LED on each decoded frame (default on).
        pub fn flash_led(mut self, yes: bool) -> Self {
            self.flash = yes;
            self
        }
        /// The decoder's counters so far.
        pub fn stats(&self) -> Stats {
            self.rb.dec.stats
        }
        /// The decoder itself, for the counters that are not in [`Stats`] - how
        /// many addresses it trusts, and how many it has had to drop.
        pub fn decoder(&self) -> &Decoder {
            &self.rb.dec
        }
        /// The error that stopped recording, if one did.
        pub fn record_error(&self) -> Option<&io::Error> {
            self.rb.raw_log_error.as_ref()
        }
    }

    /// One step of the receive loop.
    pub enum Tick {
        /// A frame came out of the decoder.
        Frame(Frame),
        /// Nothing decoded this time round. Returned often - it is the caller's
        /// chance to redraw, service a listener, or do anything else periodic.
        Idle,
        /// The run is over.
        Done,
    }

    impl<P: Port> Frames<P> {
        /// Advance once without blocking on traffic.
        ///
        /// [`Iterator::next`] spins here until something decodes, which is what a
        /// plain printer wants and is useless to a display that has to stay
        /// responsive when the sky is quiet.
        pub fn poll(&mut self) -> io::Result<Tick> {
            if let Some(f) = self.queue.pop_front() {
                if self.flash {
                    let _ = self.rb.dev.signal();
                }
                return Ok(Tick::Frame(f));
            }
            if let Some(d) = self.deadline {
                if P::now() >= d {
                    return Ok(Tick::Done);
                }
            }
            self.rb.dev.keepalive()?;
            let Some(ms) = self.rb.dev.poll_burst()? else {
                return Ok(Tick::Idle);
            };

            let RadarBox {
                dev,
                dec,
                raw_log,
                raw_log_error,
            } = &mut self.rb;
            let data = dev.burst();
            if let Some(f) = raw_log {
                use std::io::Write;
                let written = f
                    .write_all(&ms.to_le_bytes())
                    .and_then(|()| f.write_all(&(data.len() as u16).to_le_bytes()))
                    .and_then(|()| f.write_all(data));
                if let Err(e) = written {
                    *raw_log = None;
                    *raw_log_error = Some(e);
                }
            }
            dec.decode_buffer(data, ms, |f| self.queue.push_back(f));
            Ok(Tick::Idle)
        }

        /// The device, for status while a run is in progress.
        pub fn device(&mut self) -> &mut Device<P> {
            &mut self.rb.dev
        }
    }

    impl<P: Port> Iterator for Frames<P> {
        type Item = io::Result<Frame>;

        /// Blocks until a frame decodes or the run ends: [`Frames::poll`] in a loop.
        fn next(&mut self) -> Option<Self::Item> {
            loop {
                match self.poll() {
                    Ok(Tick::Frame(f)) => return Some(Ok(f)),
                    Ok(Tick::Idle) => continue,
                    Ok(Tick::Done) => return None,
                    Err(e) => return Some(Err(e)),
                }
            }
        }
    }

    #[cfg(test)]
    mod tests {
        use super::*;
        use crate::ftdi::fake::{self, Ev, FakePort, Script};
        use crate::tests::{burst, hex, EVEN, ODD};
        use std::cell::RefCell;
        use std::path::PathBuf;
        use std::rc::Rc;

        type Rb = RadarBox<FakePort>;
        type Shared = Rc<RefCell<Script>>;

        fn open() -> (Rb, Shared) {
            let (p, s) = FakePort::new();
            fake::expect_open(Ok(p));
            (Rb::open_port(&mut |_| {}).expect("open"), s)
        }

        fn unlocks(s: &Shared) -> usize {
            s.borrow()
                .writes()
                .iter()
                .filter(|w| w.starts_with("UNLOCK "))
                .count()
        }

        /// A file name in the temporary directory that no other test uses.
        fn temp(name: &str) -> PathBuf {
            std::env::temp_dir().join(format!("anrb-{}-{name}.raw", std::process::id()))
        }

        // ---- opening --------------------------------------------------------

        #[test]
        fn open_brings_the_device_up_and_unlocks_it() {
            let (p, s) = FakePort::new();
            fake::expect_open(Ok(p));
            let mut said = Vec::<String>::new();
            let rb = Rb::open_port(&mut |m| {
                if said.last().map(String::as_str) != Some(m) {
                    said.push(m.to_string());
                }
            })
            .expect("open");
            assert_eq!(
                said,
                [
                    "opening the USB device",
                    "waking the device",
                    "waiting for the challenge",
                    "answering the challenge",
                    "unlocked, asking for the version",
                ]
            );
            let mut frames = rb.frames();
            assert!(frames.device().is_unlocked());
            assert_eq!(frames.device().firmware(), Some("Fw: 02.03.1"));
            assert_eq!(unlocks(&s), 1);
        }

        #[test]
        fn open_power_cycles_a_device_that_does_not_answer() {
            // The part is wedged until its power is cut: UNLOCK draws nothing
            // on the two tries before the reset.
            let (p, s) = FakePort::new();
            s.borrow_mut().ignore_unlocks = 2;
            fake::expect_open(Ok(p));
            fake::expect_open(Ok(FakePort::again(&s)));
            fake::expect_open(Ok(FakePort::again(&s)));
            let mut said = Vec::<String>::new();
            let rb = Rb::open_port(&mut |m| said.push(m.to_string())).expect("open");
            assert!(said
                .iter()
                .any(|m| m == "no answer; power-cycling the device"));
            assert_eq!(
                s.borrow()
                    .log
                    .iter()
                    .filter(|e| **e == Ev::ResetDevice)
                    .count(),
                1
            );
            assert_eq!(unlocks(&s), 3);
            assert!(rb.frames().device().is_unlocked());
        }

        #[test]
        fn open_fails_when_power_cycling_does_not_help() {
            let (p, s) = FakePort::new();
            s.borrow_mut().question_reply = None;
            fake::expect_open(Ok(p));
            fake::expect_open(Ok(FakePort::again(&s)));
            fake::expect_open(Ok(FakePort::again(&s)));
            let e = Rb::open_port(&mut |_| {}).err().expect("no answer");
            assert_eq!(e.kind(), io::ErrorKind::TimedOut);
            assert_eq!(unlocks(&s), 2 + 6);
        }

        #[test]
        fn open_fails_when_the_device_does_not_come_back() {
            let (p, s) = FakePort::new();
            s.borrow_mut().question_reply = None;
            fake::expect_open(Ok(p));
            let e = Rb::open_port(&mut |_| {}).err().expect("gone");
            assert_eq!(e.kind(), io::ErrorKind::NotFound);
        }

        #[test]
        fn open_fails_with_no_device() {
            let e = Rb::open_port(&mut |_| {}).err().expect("no device");
            assert_eq!(e.kind(), io::ErrorKind::NotFound);
        }

        // ---- the receive loop -----------------------------------------------

        #[test]
        fn poll_turns_sample_bursts_into_frames() {
            let (rb, s) = open();
            let mut frames = rb.frames();
            s.borrow_mut().push(&burst(&hex(EVEN)));
            assert!(
                matches!(frames.poll().unwrap(), Tick::Idle),
                "burst read, PING sent"
            );
            assert!(
                matches!(frames.poll().unwrap(), Tick::Idle),
                "PONG read into the same burst"
            );
            assert!(
                matches!(frames.poll().unwrap(), Tick::Idle),
                "the gap closes it"
            );
            let Tick::Frame(f) = frames.poll().unwrap() else {
                panic!("no frame")
            };
            assert_eq!(f.as_bytes(), hex(EVEN));
            assert!(matches!(frames.poll().unwrap(), Tick::Idle));
            let st = frames.stats();
            assert_eq!((st.bursts, st.frames, st.clean), (1, 1, 1));
            assert_eq!(frames.decoder().stats.frames, 1);
            assert_eq!(s.borrow().count("PING\r\n"), 1);
            assert_eq!(
                s.borrow().count("SIGNAL\n"),
                1,
                "the LED is pulsed for the frame"
            );
        }

        #[test]
        fn frames_iterate_until_the_deadline() {
            let (rb, s) = open();
            s.borrow_mut().push(&burst(&hex(EVEN)));
            s.borrow_mut().push(&burst(&hex(ODD)));
            let start = fake::now();
            let got: Vec<Frame> = rb
                .frames()
                .until(Duration::from_secs(5))
                .collect::<io::Result<_>>()
                .unwrap();
            assert_eq!(got.iter().map(|f| f.hex()).collect::<Vec<_>>(), [EVEN, ODD]);
            assert!(fake::now() - start >= Duration::from_secs(5));
            // A PING every two seconds, the first one at once.
            assert_eq!(s.borrow().count("PING\r\n"), 3);
        }

        #[test]
        fn a_frame_decoded_past_the_deadline_is_still_delivered() {
            let (rb, s) = open();
            s.borrow_mut().push(&burst(&hex(EVEN)));
            // The burst is read at 1 ms, the PONG at 2 ms, and the empty read
            // that closes the burst ends at 7 ms, past the deadline.
            let start = fake::now();
            let mut frames = rb.frames().until(Duration::from_millis(5));
            let got: Vec<_> = frames.by_ref().collect();
            assert_eq!(fake::now() - start, Duration::from_millis(7));
            assert_eq!(got.len(), 1);
        }

        #[test]
        fn frames_stop_at_once_when_the_deadline_has_passed() {
            let (rb, s) = open();
            s.borrow_mut().push(&burst(&hex(EVEN)));
            let mut frames = rb.frames().until(Duration::ZERO);
            assert!(frames.next().is_none());
            assert_eq!(s.borrow().count("PING\r\n"), 0);
        }

        #[test]
        fn flash_led_off_sends_no_signal() {
            let (rb, s) = open();
            s.borrow_mut().push(&burst(&hex(EVEN)));
            let mut frames = rb.frames().flash_led(false);
            assert!(frames.next().unwrap().is_ok());
            assert_eq!(s.borrow().count("SIGNAL\n"), 0);
        }

        #[test]
        fn a_failed_signal_does_not_stop_the_frames() {
            let (rb, s) = open();
            s.borrow_mut().push(&burst(&hex(EVEN)));
            s.borrow_mut().hook = Some(Box::new(|cmd, s| {
                if cmd.starts_with("PING") {
                    s.fail_write = true;
                }
            }));
            let mut frames = rb.frames();
            assert!(frames.next().unwrap().is_ok());
        }

        #[test]
        fn options_reach_the_decoder() {
            let (rb, _s) = open();
            let opts = Options {
                soft: false,
                blind2: true,
                overlaid: true,
            };
            let frames = rb.with_options(opts).frames();
            let o = frames.decoder().opts;
            assert_eq!((o.soft, o.blind2, o.overlaid), (false, true, true));
        }

        #[test]
        fn link_errors_end_up_in_the_iterator() {
            let (rb, s) = open();
            let mut frames = rb.frames();
            s.borrow_mut().fail_read = true;
            assert!(frames.next().unwrap().is_err(), "a failed read");

            let (rb, s) = open();
            let mut frames = rb.frames();
            s.borrow_mut().fail_write = true;
            assert!(frames.next().unwrap().is_err(), "a failed PING");
        }

        #[test]
        fn dropping_the_frames_locks_the_device() {
            let (rb, s) = open();
            drop(rb.frames());
            assert_eq!(s.borrow().count("LOCK\n"), 1);
        }

        // ---- recording ------------------------------------------------------

        #[test]
        fn record_to_writes_every_burst_to_a_raw_log() {
            let path = temp("record");
            let (rb, s) = open();
            let rb = rb.record_to(path.to_str().unwrap()).expect("create");
            let mut frames = rb.frames();
            s.borrow_mut().push(&burst(&hex(EVEN)));
            let f = frames.next().unwrap().unwrap();
            fake::advance(Duration::from_millis(100));
            s.borrow_mut().push(&burst(&hex(ODD)));
            let g = frames.next().unwrap().unwrap();
            assert!(frames.record_error().is_none());
            drop(frames);

            let data = std::fs::read(&path).unwrap();
            std::fs::remove_file(&path).unwrap();
            assert!(data.starts_with(corpus::RAW_MAGIC));
            let log: Vec<(u32, &[u8])> = corpus::raw_log(&data).unwrap().collect();
            assert_eq!(log.len(), 2);
            let mut first = burst(&hex(EVEN));
            first.extend_from_slice(b"PONG\r\n");
            assert_eq!(log[0], (f.ms, &first[..]));
            assert_eq!(log[1], (g.ms, &burst(&hex(ODD))[..]));
            assert!(g.ms >= f.ms + 100);
        }

        #[test]
        fn record_to_reports_a_file_it_cannot_create() {
            let (rb, _s) = open();
            let path = std::env::temp_dir()
                .join("anrb-no-such-dir")
                .join("x")
                .join("log.raw");
            assert!(rb.record_to(path.to_str().unwrap()).is_err());
        }

        #[test]
        fn a_failed_write_stops_recording_and_is_kept() {
            let path = temp("readonly");
            std::fs::write(&path, b"").unwrap();
            let (mut rb, s) = open();
            // A handle opened for reading only: every write to it fails.
            rb.raw_log = Some(std::fs::File::open(&path).unwrap());
            let mut frames = rb.frames();
            s.borrow_mut().push(&burst(&hex(EVEN)));
            assert!(frames.next().unwrap().is_ok(), "frames go on");
            std::fs::remove_file(&path).unwrap();
            assert!(frames.record_error().is_some());
            assert!(frames.rb.raw_log.is_none(), "recording stopped");
        }
    }
}

#[cfg(feature = "usb")]
pub use live::{Frames, RadarBox, Tick};

#[cfg(test)]
mod tests {
    use super::*;

    /// Two real DF17 frames with valid parity.
    pub(super) const EVEN: &str = "8d4009da5833318e2bd82af8c6f5";
    pub(super) const ODD: &str = "8d4009da583324fef1cbc5c7449d";

    pub(super) fn hex(s: &str) -> Vec<u8> {
        (0..s.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
            .collect()
    }

    /// The device's samples for one frame, as it sends them: 8 samples per
    /// bit, one per bit of a byte, most significant first. A 1 bit is high
    /// then low (`F0`), a 0 bit low then high (`0F`). The `00 0a` that ends
    /// a burst follows.
    pub(super) fn burst(frame: &[u8]) -> Vec<u8> {
        let mut v: Vec<u8> = frame
            .iter()
            .flat_map(|b| {
                (0..8)
                    .rev()
                    .map(move |i| if b >> i & 1 == 1 { 0xF0 } else { 0x0F })
            })
            .collect();
        v.extend_from_slice(&[0x00, 0x0a]);
        v
    }

    #[test]
    fn a_burst_of_samples_decodes_to_its_frame() {
        let mut dec = Decoder::new();
        let f = dec.feed(&burst(&hex(EVEN)), 7);
        assert_eq!(f.len(), 1);
        assert_eq!(f[0].as_bytes(), hex(EVEN));
        assert_eq!(f[0].ms, 7);
    }

    #[test]
    fn every_burst_in_a_buffer_decodes() {
        let mut dec = Decoder::new();
        let mut buf = burst(&hex(EVEN));
        buf.extend(burst(&hex(ODD)));
        let mut out = vec![Frame::new(&hex(ODD), 0).unwrap()];
        assert_eq!(dec.feed_into(&buf, 3, &mut out), 2);
        assert_eq!(out.len(), 3, "appended to what was there");
        assert_eq!(out[1].as_bytes(), hex(EVEN));
        assert_eq!(out[2].as_bytes(), hex(ODD));
        assert_eq!(dec.stats.bursts, 2);
    }

    #[test]
    fn a_keepalive_reply_decodes_to_nothing() {
        let mut dec = Decoder::new();
        assert!(dec.feed(b"PONG\r\n", 0).is_empty());
        assert_eq!(dec.stats.bursts, 0, "not counted as a burst");
    }
}
