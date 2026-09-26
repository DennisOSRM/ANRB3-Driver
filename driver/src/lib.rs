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
//! use anrb::RadarBox;
//! # fn main() -> std::io::Result<()> {
//! for frame in RadarBox::open()?.frames() {
//!     let frame = frame?;
//!     println!("{} DF{} {:06X}", frame.hex(), frame.df(), frame.icao());
//! }
//! # Ok(()) }
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
pub use tracker::{Aircraft, Tracker, Update};
#[cfg(feature = "usb")]
pub use device::{Device, Locked};
pub use simd::Backend;

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
        if protocol::is_pong(buf) { return; }
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
    use std::collections::VecDeque;
    use std::io;
    use std::time::{Duration, Instant};

    /// A connected receiver, already authenticated.
    pub struct RadarBox {
        dev: Device,
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
            progress("opening the USB device");
            // Attach to the box as it is. This answers in under two seconds
            // whether the last session ended cleanly, was killed mid-stream, or
            // never happened, so it is worth two attempts before reaching for the
            // reset that costs about five extra seconds of MCU boot (4.8 measured).
            let mut dev = Device::open()?;
            if dev.authenticate_tries(progress, 2).is_err() {
                progress("no answer; power-cycling the device");
                drop(dev);
                dev = Device::power_cycle_open()?;
                dev.authenticate_tries(progress, 6)?;
            }
            Ok(RadarBox { dev, dec: Decoder::new(), raw_log: None, raw_log_error: None })
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
        pub fn frames(self) -> Frames {
            Frames {
                rb: self,
                queue: VecDeque::new(),
                deadline: None,
                flash: true,
            }
        }
    }

    /// A stream of decoded ADS-B/Mode-S frames.
    pub struct Frames {
        rb: RadarBox,
        queue: VecDeque<Frame>,
        deadline: Option<Instant>,
        flash: bool,
    }

    impl Frames {
        /// Stop yielding after `d` has elapsed.
        pub fn until(mut self, d: Duration) -> Self {
            self.deadline = Some(Instant::now() + d);
            self
        }
        /// Whether to pulse the front-panel LED on each decoded frame (default on).
        pub fn flash_led(mut self, yes: bool) -> Self {
            self.flash = yes;
            self
        }
        /// The decoder's counters so far.
        pub fn stats(&self) -> Stats { self.rb.dec.stats }
        /// The decoder itself, for the counters that are not in [`Stats`] - how
        /// many addresses it trusts, and how many it has had to drop.
        pub fn decoder(&self) -> &Decoder { &self.rb.dec }
        /// The error that stopped recording, if one did.
        pub fn record_error(&self) -> Option<&io::Error> { self.rb.raw_log_error.as_ref() }
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

    impl Frames {
        /// Advance once without blocking on traffic.
        ///
        /// [`Iterator::next`] spins here until something decodes, which is what a
        /// plain printer wants and is useless to a display that has to stay
        /// responsive when the sky is quiet.
        pub fn poll(&mut self) -> io::Result<Tick> {
            if let Some(f) = self.queue.pop_front() {
                if self.flash { let _ = self.rb.dev.signal(); }
                return Ok(Tick::Frame(f));
            }
            if let Some(d) = self.deadline {
                if Instant::now() >= d { return Ok(Tick::Done); }
            }
            self.rb.dev.keepalive()?;
            let Some(ms) = self.rb.dev.poll_burst()? else { return Ok(Tick::Idle) };

            let RadarBox { dev, dec, raw_log, raw_log_error } = &mut self.rb;
            let data = dev.burst();
            if let Some(f) = raw_log {
                use std::io::Write;
                let written = f.write_all(&ms.to_le_bytes())
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
        pub fn device(&mut self) -> &mut Device { &mut self.rb.dev }
    }

    impl Iterator for Frames {
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
}

#[cfg(feature = "usb")]
pub use live::{Frames, RadarBox, Tick};
