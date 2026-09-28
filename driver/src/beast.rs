//! Mode-S Beast binary output, the format dump1090, readsb, tar1090 and the
//! feeder clients read on port 30005.
//!
//! Each frame goes out as `0x1a`, a type byte - `'2'` for a 56-bit frame, `'3'`
//! for 112 bits - six timestamp bytes, one signal byte, then the frame. Any
//! `0x1a` after the type byte is sent twice, so a reader can always find the
//! start of the next frame.
//! <https://wiki.jetvision.de/wiki/Mode-S_Beast:Data_Output_Formats>
//!
//! Two fields are sent as zero, deliberately:
//!
//! * **Timestamp.** Beast carries a 12 MHz receive clock for multilateration.
//!   This receiver knows time only to the millisecond, and filling the field
//!   from that would give MLAT networks timestamps with 300 km of uncertainty.
//!   Zero is the value that says "none": mlat-client drops zero-timestamped
//!   messages unless asked for them, while decoders use them as normal. (The
//!   other reserved value, 0xFF004D4C4154, means "synthetic, made by MLAT",
//!   which these frames are not.)
//! * **Signal level.** The RadarBox reports no amplitude at all. readsb encodes
//!   any real signal as at least 1, so 0 reads as "no information".

use crate::decode::Frame;
use crate::feed::{Event, Feed, ReaderInfo};

const ESC: u8 = 0x1a;

/// Bytes of a short (56-bit) and a long (112-bit) Mode-S frame.
const SHORT: usize = 7;
const LONG: usize = 14;
/// Bytes before the frame: six of timestamp, one of signal level.
const HEADER: usize = 7;
/// Bytes of a Mode A/C payload.
const MODE_AC: usize = 2;

/// Append one frame in Beast form. False, with nothing written, for a length
/// Beast has no type for.
pub fn encode(frame: &[u8], out: &mut Vec<u8>) -> bool {
    let kind = match frame.len() {
        SHORT => b'2',
        LONG => b'3',
        _ => return false,
    };
    out.extend_from_slice(&[ESC, kind]);
    out.extend_from_slice(&[0; 6]); // timestamp: none
    out.push(0); // signal level: none
    for &b in frame {
        out.push(b);
        if b == ESC {
            out.push(ESC);
        }
    }
    true
}

/// A Beast feed on a TCP port.
pub struct BeastServer {
    feed: Feed,
    buf: Vec<u8>,
}

impl BeastServer {
    /// Listen on `port` on every interface; 0 lets the system choose.
    pub fn bind(port: u16) -> std::io::Result<Self> {
        Ok(BeastServer {
            feed: Feed::bind(port)?,
            buf: Vec::with_capacity(64),
        })
    }
    /// The port listened on.
    pub fn port(&self) -> u16 {
        self.feed.port()
    }
    /// Readers currently attached.
    pub fn clients(&self) -> usize {
        self.feed.clients()
    }
    /// Frames sent while at least one reader was attached.
    pub fn frames(&self) -> u64 {
        self.feed.sent()
    }
    /// Readers cut off for falling behind.
    pub fn dropped(&self) -> u64 {
        self.feed.dropped()
    }
    /// Accept readers and flush queued bytes; call every time round the loop.
    pub fn poll(&mut self) {
        self.feed.poll();
    }
    /// Who is attached.
    pub fn readers(&self) -> Vec<ReaderInfo> {
        self.feed.readers()
    }
    /// Joins and departures since the last call.
    pub fn take_events(&mut self) -> Vec<Event> {
        self.feed.take_events()
    }

    /// Every decoded frame, as it decodes.
    pub fn emit(&mut self, f: &Frame) {
        self.buf.clear();
        if encode(f.as_bytes(), &mut self.buf) {
            self.feed.send(&self.buf);
        }
    }
}

/// One message read from a Beast stream.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Message {
    /// 12 MHz receive clock; 0 when the sender has none.
    pub timestamp: u64,
    /// Signal level; 0 when the sender has none.
    pub signal: u8,
    bytes: [u8; LONG],
    len: u8,
}

impl Message {
    /// The Mode-S frame, 7 or 14 bytes.
    pub fn frame(&self) -> &[u8] {
        &self.bytes[..self.len as usize]
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum State {
    /// Looking for 0x1a.
    #[default]
    Hunt,
    /// Just read 0x1a; the type byte comes next.
    Kind,
    /// Inside a message, with this many bytes still to come.
    Body(usize),
}

/// Reads Beast as it arrives, in whatever pieces the socket hands over.
///
/// A 0x1a inside a message must be doubled; a single one followed by anything
/// else is the start of the next message, and the one before it was cut
/// short. That is how a reader that joins mid-stream, or loses bytes,
/// resynchronises. Such truncations are counted in `resyncs`, not treated as
/// errors.
#[derive(Default)]
pub struct Reader {
    state: State,
    kind: u8,
    body: [u8; HEADER + LONG],
    got: usize,
    escaped: bool,
    /// Messages cut short by the start of another.
    pub resyncs: u64,
}

impl Reader {
    /// A reader hunting for the start of the first message.
    pub fn new() -> Self {
        Self::default()
    }

    /// Consume `data`, handing every complete Mode-S message to `each`.
    /// Mode A/C ('1') and status messages are read past and not reported.
    pub fn feed(&mut self, data: &[u8], mut each: impl FnMut(&Message)) {
        for &b in data {
            if let State::Body(left) = self.state {
                if self.escaped {
                    self.escaped = false;
                    if b != ESC {
                        // A lone 0x1a: a new message began here.
                        self.resyncs += 1;
                        self.start(b);
                        continue;
                    }
                } else if b == ESC {
                    self.escaped = true;
                    continue;
                }
                self.body[self.got] = b;
                self.got += 1;
                if left == 1 {
                    self.state = State::Hunt;
                    self.finish(&mut each);
                } else {
                    self.state = State::Body(left - 1);
                }
                continue;
            }
            match self.state {
                State::Kind => self.start(b),
                _ if b == ESC => self.state = State::Kind,
                _ => {}
            }
        }
    }

    /// The type byte: how much follows, or back to hunting if it is not one.
    fn start(&mut self, kind: u8) {
        let payload = match kind {
            b'1' => MODE_AC,
            b'2' => SHORT,
            b'3' | b'4' => LONG,
            _ => 0,
        };
        self.got = 0;
        self.kind = kind;
        self.state = if payload == 0 {
            State::Hunt
        } else {
            State::Body(HEADER + payload)
        };
    }

    fn finish(&self, each: &mut impl FnMut(&Message)) {
        let n = match self.kind {
            b'2' => SHORT,
            b'3' => LONG,
            _ => return,
        };
        let mut m = Message {
            timestamp: 0,
            signal: self.body[HEADER - 1],
            bytes: [0; LONG],
            len: n as u8,
        };
        for &t in &self.body[..HEADER - 1] {
            m.timestamp = m.timestamp << 8 | t as u64;
        }
        m.bytes[..n].copy_from_slice(&self.body[HEADER..HEADER + n]);
        each(&m);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Undo `encode`, the way a reader does: find 0x1a and a type byte, then
    /// read a fixed number of payload bytes, collapsing doubled 0x1a.
    fn decode(mut s: &[u8]) -> Vec<(u8, Vec<u8>)> {
        let mut out = Vec::new();
        while let [ESC, kind, rest @ ..] = s {
            let n = match kind {
                b'2' => 7,
                b'3' => 14,
                _ => panic!("type {kind}"),
            } + 7;
            let (mut body, mut i) = (Vec::new(), 0);
            while body.len() < n {
                body.push(rest[i]);
                i += if rest[i] == ESC {
                    assert_eq!(rest[i + 1], ESC, "a lone 0x1a in a payload");
                    2
                } else {
                    1
                };
            }
            out.push((*kind, body));
            s = &rest[i..];
        }
        assert!(s.is_empty(), "trailing bytes");
        out
    }

    #[test]
    fn short_and_long_frames_get_their_types() {
        let df11 = [0x5d, 0x3c, 0x65, 0x51, 0x12, 0x34, 0x56];
        let df17 = [
            0x8d, 0x40, 0x09, 0xda, 0x58, 0x33, 0x31, 0x8e, 0x2b, 0xd8, 0x2a, 0xf8, 0xc6, 0xf5,
        ];
        let mut out = Vec::new();
        assert!(encode(&df11, &mut out));
        assert!(encode(&df17, &mut out));
        assert_eq!(
            &out[..9],
            &[ESC, b'2', 0, 0, 0, 0, 0, 0, 0],
            "header: type, no timestamp, no signal"
        );
        let got = decode(&out);
        assert_eq!(got[0], (b'2', [&[0u8; 7][..], &df11].concat()));
        assert_eq!(got[1], (b'3', [&[0u8; 7][..], &df17].concat()));
    }

    #[test]
    fn a_0x1a_in_the_frame_is_sent_twice() {
        let f = [0x5d, ESC, 0x65, ESC, ESC, 0x34, 0x56];
        let mut out = Vec::new();
        encode(&f, &mut out);
        assert_eq!(out.len(), 2 + 7 + 7 + 3, "three extra bytes, one per 0x1a");
        assert_eq!(decode(&out), vec![(b'2', [&[0u8; 7][..], &f].concat())]);
    }

    fn read_all(r: &mut Reader, chunks: &[&[u8]]) -> Vec<Vec<u8>> {
        let mut got = Vec::new();
        for c in chunks {
            r.feed(c, |m| got.push(m.frame().to_vec()));
        }
        got
    }

    /// Whatever the socket's piece boundaries, the same frames come out,
    /// including one full of bytes that had to be doubled.
    #[test]
    fn the_reader_undoes_encode_across_any_split() {
        let frames: [&[u8]; 3] = [
            &[0x5d, ESC, 0x65, ESC, ESC, 0x34, 0x56],
            &[
                0x8d, 0x40, 0x09, 0xda, 0x58, 0x33, 0x31, 0x8e, 0x2b, 0xd8, 0x2a, 0xf8, 0xc6, 0xf5,
            ],
            &[ESC; 7],
        ];
        let mut wire = Vec::new();
        for f in frames {
            encode(f, &mut wire);
        }
        for cut in 1..wire.len() {
            let mut r = Reader::new();
            let got = read_all(&mut r, &[&wire[..cut], &wire[cut..]]);
            assert_eq!(got, frames.map(<[u8]>::to_vec), "split at {cut}");
            assert_eq!(r.resyncs, 0);
        }
        let mut r = Reader::new();
        let bytes: Vec<&[u8]> = wire.chunks(1).collect();
        assert_eq!(read_all(&mut r, &bytes).len(), 3, "one byte at a time");
    }

    /// Joining mid-message, the reader skips to the next 0x1a; a message cut
    /// short by a new one is abandoned for it.
    #[test]
    fn the_reader_resynchronises() {
        let f = [0x5d, 0x3c, 0x65, 0x51, 0x12, 0x34, 0x56];
        let mut one = Vec::new();
        encode(&f, &mut one);
        let mut wire = one[5..].to_vec(); // the tail of a message
        wire.extend_from_slice(&one[..12]); // then one cut short
        wire.extend_from_slice(&one); // then a whole one
        let mut r = Reader::new();
        assert_eq!(read_all(&mut r, &[&wire]), vec![f.to_vec()]);
        assert_eq!(r.resyncs, 1);
    }

    #[test]
    fn mode_ac_and_timestamps() {
        let mut r = Reader::new();
        let mut wire = vec![ESC, b'1', 0, 0, 0, 0, 0, 1, 0x20, 0x12, 0x34];
        wire.extend_from_slice(&[ESC, b'2', 0, 0, 0, 0, 1, 2, 0x55]);
        wire.extend_from_slice(&[0x5d, 0x3c, 0x65, 0x51, 0x12, 0x34, 0x56]);
        let mut got = Vec::new();
        r.feed(&wire, |m| got.push(*m));
        assert_eq!(got.len(), 1, "Mode A/C is read past");
        assert_eq!((got[0].timestamp, got[0].signal), (0x0102, 0x55));
    }

    #[test]
    fn other_lengths_are_refused() {
        let mut out = Vec::new();
        assert!(!encode(&[0; 11], &mut out));
        assert!(out.is_empty());
    }

    /// Type '4' is a long frame of another kind, and a type byte that is not
    /// one sends the reader back to hunting; neither is reported.
    #[test]
    fn other_message_types_are_read_past() {
        let long = [
            0x8d, 0x40, 0x09, 0xda, 0x58, 0x33, 0x31, 0x8e, 0x2b, 0xd8, 0x2a, 0xf8, 0xc6, 0xf5,
        ];
        let mut wire = vec![ESC, b'4'];
        wire.extend_from_slice(&[0; HEADER]);
        wire.extend_from_slice(&long);
        wire.extend_from_slice(&[ESC, b'9', 1, 2, 3]);
        encode(&long, &mut wire);
        let mut r = Reader::new();
        assert_eq!(
            read_all(&mut r, &[&wire]),
            vec![long.to_vec()],
            "only the '3' frame"
        );
        assert_eq!(r.resyncs, 0);
    }

    /// Connect a reader and wait until the server has accepted it.
    fn attach(s: &mut BeastServer, n: usize) -> std::net::TcpStream {
        let c = std::net::TcpStream::connect(("127.0.0.1", s.port())).unwrap();
        c.set_read_timeout(Some(std::time::Duration::from_secs(5)))
            .unwrap();
        for _ in 0..400 {
            s.poll();
            if s.clients() == n {
                break;
            }
            std::thread::sleep(std::time::Duration::from_millis(5));
        }
        assert_eq!(s.clients(), n, "the reader was accepted");
        c
    }

    /// Every reader gets every frame, in Beast form.
    #[test]
    fn the_server_sends_each_frame_to_every_reader() {
        use std::io::Read;
        let short = [0x5d, 0x3c, 0x65, 0x51, 0x12, 0x34, ESC];
        let long = [
            0x8d, 0x40, 0x09, 0xda, 0x58, 0x33, 0x31, 0x8e, 0x2b, 0xd8, 0x2a, 0xf8, 0xc6, 0xf5,
        ];
        let mut s = BeastServer::bind(0).unwrap();
        assert_ne!(s.port(), 0);
        s.emit(&Frame::new(&short, 0).unwrap());
        assert_eq!(s.frames(), 0, "no reader, nothing counted");

        let mut readers = [attach(&mut s, 1), attach(&mut s, 2)];
        assert_eq!(s.readers().len(), 2);
        assert_eq!(s.take_events().len(), 2, "two joined");

        s.emit(&Frame::new(&short, 1).unwrap());
        s.emit(&Frame::new(&long, 2).unwrap());
        assert_eq!((s.frames(), s.dropped()), (2, 0));
        let mut want = Vec::new();
        encode(&short, &mut want);
        encode(&long, &mut want);
        for c in &mut readers {
            let mut got = vec![0u8; want.len()];
            c.read_exact(&mut got).unwrap();
            assert_eq!(got, want);
            let mut r = Reader::new();
            assert_eq!(
                read_all(&mut r, &[&got]),
                vec![short.to_vec(), long.to_vec()]
            );
        }
    }
}
