//! A TCP feed: a listener that takes any number of readers and sends each of
//! them every message, whole. The SBS and Beast outputs are both one of these.
//!
//! Sockets are non-blocking, so a slow reader can never stall the receiver.
//! What a reader cannot take at once is queued for it and sent on a later
//! [`Feed::poll`] - and messages are only ever queued whole, so a reader never
//! sees half an SBS line or half a Beast frame. `write_all` is not used: on a
//! non-blocking socket it can fail with WouldBlock part-way through a message
//! and lose the remainder. A reader whose queue passes [`BACKLOG`] has stopped
//! draining and is dropped.

use std::io::{ErrorKind, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::time::{Duration, Instant};

/// Bytes a reader may fall behind before it is dropped: about a minute of SBS
/// at a busy receiver's rate, far more of Beast.
pub const BACKLOG: usize = 256 * 1024;

struct Reader {
    stream: TcpStream,
    queue: Vec<u8>,
    peer: SocketAddr,
    since: Instant,
}

/// One attached reader, for showing who is connected.
#[derive(Clone, Debug)]
pub struct ReaderInfo {
    pub peer: SocketAddr,
    /// How long it has been connected.
    pub connected: Duration,
    /// Bytes waiting for it to read: zero for one that keeps up.
    pub behind: usize,
}

/// Something that happened to the set of readers.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Event {
    /// A reader connected.
    Joined(SocketAddr),
    /// A reader went away, and why.
    Left(SocketAddr, LeaveReason),
}

/// Why a reader went away.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LeaveReason {
    /// It closed the connection.
    Closed,
    /// It stopped reading and fell BACKLOG behind.
    Stalled,
}

impl Reader {
    /// Push what the socket will take. False when the reader is gone.
    fn flush(&mut self) -> bool {
        while !self.queue.is_empty() {
            match self.stream.write(&self.queue) {
                Ok(0) => return false,
                Ok(n) => {
                    self.queue.drain(..n);
                }
                Err(e) if e.kind() == ErrorKind::WouldBlock => return true,
                Err(e) if e.kind() == ErrorKind::Interrupted => continue,
                Err(_) => return false,
            }
        }
        true
    }
}

/// A listening socket and the readers attached to it.
pub struct Feed {
    listener: TcpListener,
    readers: Vec<Reader>,
    events: Vec<Event>,
    port: u16,
    sent: u64,
    dropped: u64,
}

impl Feed {
    /// Listen on every interface.
    pub fn bind(port: u16) -> std::io::Result<Self> {
        let listener = TcpListener::bind(("0.0.0.0", port))?;
        listener.set_nonblocking(true)?;
        let port = listener.local_addr()?.port(); // the real one, if 0 was asked for
        Ok(Feed {
            listener,
            readers: Vec::new(),
            events: Vec::new(),
            port,
            sent: 0,
            dropped: 0,
        })
    }

    /// The port listened on: the one asked for, or the one the system chose
    /// if 0 was asked for.
    pub fn port(&self) -> u16 {
        self.port
    }

    /// Readers currently attached.
    pub fn clients(&self) -> usize {
        self.readers.len()
    }

    /// Messages sent while at least one reader was attached.
    pub fn sent(&self) -> u64 {
        self.sent
    }

    /// Readers cut off for falling BACKLOG behind. One that hangs up is not
    /// counted: that is a reader leaving, not the feed dropping it.
    pub fn dropped(&self) -> u64 {
        self.dropped
    }

    /// Who is attached, oldest first.
    pub fn readers(&self) -> Vec<ReaderInfo> {
        self.readers
            .iter()
            .map(|r| ReaderInfo {
                peer: r.peer,
                connected: r.since.elapsed(),
                behind: r.queue.len(),
            })
            .collect()
    }

    /// Joins and departures since the last call.
    pub fn take_events(&mut self) -> Vec<Event> {
        std::mem::take(&mut self.events)
    }

    /// Take any pending connections and push queued bytes to slow readers.
    /// Cheap; call it every time round the loop.
    pub fn poll(&mut self) {
        while let Ok((s, peer)) = self.listener.accept() {
            let _ = s.set_nodelay(true);
            if s.set_nonblocking(true).is_ok() {
                self.readers.push(Reader {
                    stream: s,
                    queue: Vec::new(),
                    peer,
                    since: Instant::now(),
                });
                self.events.push(Event::Joined(peer));
            }
        }
        self.retain(|r| {
            if r.queue.is_empty() || r.flush() {
                None
            } else {
                Some(LeaveReason::Closed)
            }
        });
    }

    /// Send one whole message to every reader.
    pub fn send(&mut self, msg: &[u8]) {
        if self.readers.is_empty() {
            return;
        }
        self.retain(|r| {
            r.queue.extend_from_slice(msg);
            if !r.flush() {
                Some(LeaveReason::Closed)
            } else if r.queue.len() > BACKLOG {
                Some(LeaveReason::Stalled)
            } else {
                None
            }
        });
        self.sent += 1;
    }

    /// Drop every reader `gone` gives a reason for, and say so.
    fn retain(&mut self, mut gone: impl FnMut(&mut Reader) -> Option<LeaveReason>) {
        let (events, dropped) = (&mut self.events, &mut self.dropped);
        self.readers.retain_mut(|r| match gone(r) {
            Some(reason) => {
                if reason == LeaveReason::Stalled {
                    *dropped += 1;
                }
                events.push(Event::Left(r.peer, reason));
                false
            }
            None => true,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Read;

    fn feed() -> Feed {
        Feed::bind(0).unwrap()
    }

    fn attach(f: &mut Feed) -> TcpStream {
        let c = TcpStream::connect(("127.0.0.1", f.port())).unwrap();
        for _ in 0..200 {
            f.poll();
            if f.clients() > 0 {
                break;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        assert_eq!(f.clients(), 1, "the reader was accepted");
        c
    }

    /// A reader that falls behind gets every message, in order and whole,
    /// once it starts reading again. Sends until the kernel's buffers are full
    /// and the queue is in use - otherwise the path this is about never
    /// runs - then some more, then reads it all back.
    ///
    /// The messages are about 1 KiB. With messages of a few bytes, macOS
    /// accepts tens of megabytes on a loopback socket without pushing back
    /// and then resets the connection.
    #[test]
    fn a_slow_reader_gets_whole_messages_in_order() {
        let mut f = feed();
        let mut c = attach(&mut f);
        let mut want = Vec::new();
        let mut i = 0u32;
        let mut send = |f: &mut Feed, want: &mut Vec<u8>| {
            let m = format!("message {i:07} {:>1000}\r\n", "").into_bytes();
            i += 1;
            f.send(&m);
            want.extend_from_slice(&m);
        };
        while f.readers.first().is_some_and(|r| r.queue.is_empty()) {
            send(&mut f, &mut want);
            assert!(want.len() < 64 << 20, "the kernel never pushed back");
        }
        assert_eq!(
            f.clients(),
            1,
            "the reader was dropped before the kernel pushed back"
        );
        // Well short of BACKLOG, so the reader stays attached.
        for _ in 0..100 {
            send(&mut f, &mut want);
        }
        assert!(
            !f.readers[0].queue.is_empty() && f.clients() == 1,
            "queued, still attached"
        );
        let info = f.readers();
        assert_eq!(info.len(), 1);
        assert!(
            info[0].behind > 0 && info[0].peer.ip().is_loopback(),
            "{info:?}"
        );

        let mut got = Vec::new();
        c.set_read_timeout(Some(Duration::from_millis(200)))
            .unwrap();
        let mut buf = [0u8; 65536];
        while got.len() < want.len() {
            f.poll();
            match c.read(&mut buf) {
                Ok(0) => break,
                Ok(n) => got.extend_from_slice(&buf[..n]),
                Err(_) => {}
            }
        }
        assert!(
            got == want,
            "received {} of {} bytes, and they must match",
            got.len(),
            want.len()
        );
    }

    /// A reader that hangs up is noticed on the next write, and said to have
    /// closed rather than stalled.
    #[test]
    fn a_reader_that_hangs_up_is_reported_closed() {
        let mut f = feed();
        drop(attach(&mut f));
        for _ in 0..200 {
            f.send(b"x\r\n");
            f.poll();
            if f.clients() == 0 {
                break;
            }
            std::thread::sleep(Duration::from_millis(5));
        }
        assert_eq!(f.clients(), 0);
        let ev = f.take_events();
        assert!(
            matches!(
                ev.as_slice(),
                [Event::Joined(_), Event::Left(_, LeaveReason::Closed)]
            ),
            "{ev:?}"
        );
        assert_eq!(f.dropped(), 0, "hanging up is leaving, not being dropped");
    }

    /// One that never reads is dropped once it is BACKLOG behind, rather than
    /// holding memory for ever.
    #[test]
    fn a_reader_that_never_drains_is_dropped() {
        let mut f = feed();
        let _c = attach(&mut f);
        let m = [b'x'; 1024];
        let mut n = 0;
        while f.clients() == 1 && n < 100_000 {
            f.send(&m);
            n += 1;
        }
        assert_eq!(f.clients(), 0);
        assert_eq!(f.dropped(), 1);
        let ev = f.take_events();
        assert!(
            matches!(
                ev.as_slice(),
                [Event::Joined(_), Event::Left(_, LeaveReason::Stalled)]
            ),
            "{ev:?}"
        );
        assert!(
            n * m.len() > BACKLOG,
            "dropped only after passing the backlog"
        );
    }
}
