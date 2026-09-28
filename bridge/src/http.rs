//! The map bridge's transport: an HTTP/1.1 and WebSocket server, optionally
//! inside TLS, with a thread per connection and no async runtime.
//!
//! It knows nothing about aircraft: it takes a [`Handler`], hands it a path
//! and a query string for every GET, and hands it a [`WebSocket`] for every
//! accepted upgrade. What goes down the socket is the caller's business.
//!
//! Connections are one per thread and blocking throughout, because there are
//! only ever a handful of browsers on a home receiver. An idle blocking thread
//! costs only its stack. The handshake runs on the connection thread; see
//! [`Server::serve`].
//!
//! The WebSocket side is implemented here rather than taken as a dependency:
//! the handshake is a SHA-1 and a base64, and the framing this needs is a
//! length and a mask. The map bridge depends only on flate2 and rustls (with
//! rustls-pemfile and webpki-roots).

use std::io::{self, ErrorKind, Read, Write};
use std::net::{SocketAddr, TcpListener, TcpStream};
use std::panic::AssertUnwindSafe;
use std::path::Path;
use std::sync::Arc;
use std::time::{Duration, Instant};

/// The most a request head may be. Everything this serves is a GET with a
/// short query, so anything longer is rejected with 431.
const HEAD_LIMIT: usize = 8 * 1024;

/// How long a kept-alive connection may sit silent before it is closed, and
/// how long a request head may take to arrive.
const IDLE: Duration = Duration::from_secs(30);

/// The most a client may send in one WebSocket frame. A browser has no reason
/// to send anything at all here beyond a ping or a close.
const MAX_FRAME: usize = 1 << 20;

const WS_GUID: &str = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";

// ---- what a handler returns ------------------------------------------------

/// One HTTP response, complete.
pub struct Response {
    pub status: u16,
    pub content_type: String,
    pub body: Vec<u8>,
    /// Whether `body` is already deflated and wants `Content-Encoding: gzip`.
    pub gzip: bool,
    /// Seconds a client may keep it. Zero is `no-store`, which is right for
    /// anything that tracks the feed; an airline logo never changes.
    pub max_age: u32,
}

impl Response {
    pub fn ok(content_type: &str, body: Vec<u8>) -> Response {
        Response {
            status: 200,
            content_type: content_type.to_string(),
            body,
            gzip: false,
            max_age: 0,
        }
    }

    /// As [`Response::ok`], for a body the caller has already run through
    /// [`gzip`]. The server does not compress anything itself: only the caller
    /// knows whether a payload is worth it, and the JSON updates are built
    /// once and sent to several pages.
    pub fn gzipped(content_type: &str, body: Vec<u8>) -> Response {
        Response {
            status: 200,
            content_type: content_type.to_string(),
            body,
            gzip: true,
            max_age: 0,
        }
    }

    /// Something that does not change: served once and kept by the browser.
    pub fn cached(content_type: &str, body: Vec<u8>, max_age: u32) -> Response {
        Response {
            status: 200,
            content_type: content_type.to_string(),
            body,
            gzip: false,
            max_age,
        }
    }

    /// An error, as plain text for whoever is reading the raw response.
    pub fn status(code: u16, text: &str) -> Response {
        Response {
            status: code,
            content_type: "text/plain; charset=utf-8".to_string(),
            body: format!("{text}\n").into_bytes(),
            gzip: false,
            max_age: 0,
        }
    }
}

/// What a client sent down an open WebSocket.
pub enum Message {
    Text(String),
    /// Already answered with a pong by [`WebSocket::poll`]; reported so the
    /// caller can treat it as a sign of life.
    Ping(Vec<u8>),
    Pong,
    /// The peer is closing. A close has been sent back; the handler should
    /// return.
    Close,
}

/// Application callbacks for GET requests and WebSocket upgrades.
pub trait Handler: Send + Sync + 'static {
    /// A GET. `path` has no query string; `query` is what followed '?', or "".
    fn get(&self, path: &str, query: &str) -> Response;

    /// An accepted WebSocket upgrade, on its own thread. Runs until it
    /// returns, at which point the connection is closed.
    ///
    /// Every path is an upgrade endpoint as far as this module is concerned -
    /// the map has exactly one - so the handler is given the query string
    /// alone. A GET to that path without an `Upgrade` header goes to
    /// [`Handler::get`] like any other.
    fn websocket(&self, query: &str, ws: WebSocket);
}

// ---- the server ------------------------------------------------------------

pub struct Server {
    listener: TcpListener,
    addr: SocketAddr,
    tls: Option<Arc<rustls::ServerConfig>>,
}

impl Server {
    /// Bind, optionally with TLS from a PEM certificate chain and private key.
    ///
    /// Browsers only give a page the viewer's location when it is served
    /// securely, so the dot for "you" needs this whenever the map is opened
    /// from another machine.
    pub fn bind(addr: SocketAddr, tls: Option<(&Path, &Path)>) -> io::Result<Server> {
        let tls = match tls {
            Some((cert, key)) => Some(Arc::new(tls_config(cert, key)?)),
            None => None,
        };
        let listener = TcpListener::bind(addr)?;
        let addr = listener.local_addr()?; // the real one, if port 0 was asked for
        Ok(Server {
            listener,
            addr,
            tls,
        })
    }

    pub fn local_addr(&self) -> SocketAddr {
        self.addr
    }

    pub fn is_tls(&self) -> bool {
        self.tls.is_some()
    }

    /// Serve until the process ends, a thread per connection.
    ///
    /// Nothing but accepting happens here. The TLS handshake runs on the
    /// connection's own thread at its first read, so a client that connects
    /// and sends nothing cannot stall the accept loop.
    ///
    /// A connection thread that panics takes its connection down and nothing
    /// else - it is caught here so the panic cannot escape into the runtime,
    /// and the accept loop carries on either way.
    pub fn serve<H: Handler>(self, handler: Arc<H>) -> io::Result<()> {
        loop {
            let (stream, _) = match self.listener.accept() {
                Ok(pair) => pair,
                // One client going away between the SYN and the accept is not
                // a reason to stop serving the others.
                Err(e)
                    if matches!(
                        e.kind(),
                        ErrorKind::ConnectionAborted | ErrorKind::Interrupted
                    ) =>
                {
                    continue
                }
                Err(e) => return Err(e),
            };
            let handler = Arc::clone(&handler);
            let tls = self.tls.clone();
            let spawned = std::thread::Builder::new()
                .name("anrb-http".to_string())
                .spawn(move || {
                    let _ = std::panic::catch_unwind(AssertUnwindSafe(|| {
                        serve_connection(stream, tls, &*handler);
                    }));
                });
            // Out of threads: drop the connection rather than the process.
            if spawned.is_err() {
                continue;
            }
        }
    }
}

fn tls_config(cert: &Path, key: &Path) -> io::Result<rustls::ServerConfig> {
    let mut rd = io::BufReader::new(std::fs::File::open(cert)?);
    let chain = rustls_pemfile::certs(&mut rd).collect::<Result<Vec<_>, _>>()?;
    if chain.is_empty() {
        return Err(io::Error::new(
            ErrorKind::InvalidData,
            "no certificate in the PEM file",
        ));
    }
    let mut rd = io::BufReader::new(std::fs::File::open(key)?);
    let key = rustls_pemfile::private_key(&mut rd)?
        .ok_or_else(|| io::Error::new(ErrorKind::InvalidData, "no private key in the PEM file"))?;
    // The provider is named rather than left to rustls' default so that the
    // build stays on ring: aws-lc-rs wants cmake, which this tree does not.
    let provider = Arc::new(rustls::crypto::ring::default_provider());
    rustls::ServerConfig::builder_with_provider(provider)
        .with_safe_default_protocol_versions()
        .and_then(|b| b.with_no_client_auth().with_single_cert(chain, key))
        .map_err(|e| io::Error::new(ErrorKind::InvalidData, e))
}

// ---- one connection --------------------------------------------------------

/// A connection with or without TLS around it. Both halves are blocking, and
/// the timeout that bounds a read is set on the socket underneath.
enum Io {
    Plain(TcpStream),
    Tls(Box<rustls::StreamOwned<rustls::ServerConnection, TcpStream>>),
}

impl Read for Io {
    fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
        match self {
            Io::Plain(s) => s.read(buf),
            Io::Tls(s) => s.read(buf),
        }
    }
}

impl Write for Io {
    fn write(&mut self, buf: &[u8]) -> io::Result<usize> {
        match self {
            Io::Plain(s) => s.write(buf),
            Io::Tls(s) => s.write(buf),
        }
    }
    fn flush(&mut self) -> io::Result<()> {
        match self {
            Io::Plain(s) => s.flush(),
            Io::Tls(s) => s.flush(),
        }
    }
}

/// The socket, whatever is wrapped around it, and whatever has been read but
/// not yet consumed. The leftovers matter: the bytes after a request head can
/// be the next request, and after a handshake they can be the first frame.
struct Conn {
    io: Io,
    /// The same socket again, kept for timeouts. Socket options live on the
    /// socket rather than the descriptor, so setting one here sets it for the
    /// copy inside `io` too.
    sock: TcpStream,
    buf: Vec<u8>,
}

/// Why reading a request head stopped.
enum Head {
    Request(String),
    /// Longer than [`HEAD_LIMIT`].
    TooLong,
    /// The peer hung up, or said nothing at all within [`IDLE`].
    Gone,
}

impl Conn {
    /// One read, appending to the buffer. Ok(false) at end of stream.
    fn fill(&mut self) -> io::Result<bool> {
        let mut chunk = [0u8; 16 * 1024];
        loop {
            match self.io.read(&mut chunk) {
                Ok(0) => return Ok(false),
                Ok(n) => {
                    self.buf.extend_from_slice(&chunk[..n]);
                    return Ok(true);
                }
                Err(e) if e.kind() == ErrorKind::Interrupted => continue,
                Err(e) => return Err(e),
            }
        }
    }

    /// Everything up to the blank line, consumed from the buffer.
    fn read_head(&mut self) -> Head {
        let mut from = 0;
        loop {
            if let Some(at) = find(&self.buf[from..], b"\r\n\r\n") {
                let end = from + at;
                let head = String::from_utf8_lossy(&self.buf[..end]).into_owned();
                self.buf.drain(..end + 4);
                return Head::Request(head);
            }
            if self.buf.len() > HEAD_LIMIT {
                return Head::TooLong;
            }
            // Only the last three bytes of what is here can start a match.
            from = self.buf.len().saturating_sub(3);
            match self.fill() {
                Ok(true) => {}
                Ok(false) | Err(_) => return Head::Gone,
            }
        }
    }

    fn send(&mut self, bytes: &[u8]) -> io::Result<()> {
        self.io.write_all(bytes)?;
        self.io.flush()
    }

    fn timeout(&self, d: Option<Duration>) {
        let _ = self.sock.set_read_timeout(d);
    }
}

/// Where `needle` first occurs in `hay`.
pub(crate) fn find(hay: &[u8], needle: &[u8]) -> Option<usize> {
    hay.windows(needle.len()).position(|w| w == needle)
}

/// A parsed request line and its headers. Header names are matched without
/// regard to case, as the field names of a request are.
struct Request {
    method: String,
    path: String,
    query: String,
    headers: Vec<(String, String)>,
}

impl Request {
    fn parse(head: &str) -> Option<Request> {
        let mut lines = head.split("\r\n");
        let mut first = lines.next()?.split(' ');
        let method = first.next()?.to_string();
        let target = first.next()?;
        // The version is not checked: HTTP/1.0 differs here only in whether
        // the connection is kept, which the Connection header settles below.
        first.next()?;
        let (path, query) = match target.split_once('?') {
            Some((p, q)) => (p.to_string(), q.to_string()),
            None => (target.to_string(), String::new()),
        };
        let headers = lines
            .filter_map(|l| l.split_once(':'))
            .map(|(k, v)| (k.trim().to_ascii_lowercase(), v.trim().to_string()))
            .collect();
        Some(Request {
            method,
            path,
            query,
            headers,
        })
    }

    fn header(&self, name: &str) -> Option<&str> {
        self.headers
            .iter()
            .find(|(k, _)| k == name)
            .map(|(_, v)| v.as_str())
    }
}

fn serve_connection<H: Handler>(
    stream: TcpStream,
    tls: Option<Arc<rustls::ServerConfig>>,
    handler: &H,
) {
    let _ = stream.set_nodelay(true);
    let Ok(sock) = stream.try_clone() else { return };
    let io = match tls {
        // Nothing has been read yet, so this is where the handshake happens:
        // on this thread, at the first read below.
        Some(cfg) => match rustls::ServerConnection::new(cfg) {
            Ok(c) => Io::Tls(Box::new(rustls::StreamOwned::new(c, stream))),
            Err(_) => return,
        },
        None => Io::Plain(stream),
    };
    let mut conn = Conn {
        io,
        sock,
        buf: Vec::new(),
    };

    loop {
        conn.timeout(Some(IDLE));
        let head = match conn.read_head() {
            Head::Request(h) => h,
            Head::Gone => return,
            Head::TooLong => {
                let r = Response::status(431, "request head too long");
                let _ = write_response(&mut conn, &r, false);
                return;
            }
        };
        let Some(req) = Request::parse(&head) else {
            let r = Response::status(400, "malformed request");
            let _ = write_response(&mut conn, &r, false);
            return;
        };

        // GET is all there is, an upgrade included: nothing here is written to
        // and a body would have to be read before the connection could be used
        // again.
        if req.method != "GET" {
            let r = Response::status(405, "only GET is supported");
            let _ = write_response(&mut conn, &r, false);
            return;
        }

        if req
            .header("upgrade")
            .is_some_and(|u| u.eq_ignore_ascii_case("websocket"))
        {
            let Some(key) = req.header("sec-websocket-key").map(str::to_string) else {
                let r = Response::status(400, "missing Sec-WebSocket-Key");
                let _ = write_response(&mut conn, &r, false);
                return;
            };
            let reply = format!(
                "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n\
                 Connection: Upgrade\r\nSec-WebSocket-Accept: {}\r\n\r\n",
                accept_key(&key)
            );
            if conn.send(reply.as_bytes()).is_err() {
                return;
            }
            let query = req.query.clone();
            conn.timeout(None);
            handler.websocket(
                &query,
                WebSocket {
                    conn,
                    closed: false,
                },
            );
            return;
        }

        // HTTP/1.1 keeps the connection unless the client says otherwise.
        let keep = !req
            .header("connection")
            .is_some_and(|c| c.eq_ignore_ascii_case("close"));
        let resp = handler.get(&req.path, &req.query);
        if write_response(&mut conn, &resp, keep).is_err() || !keep {
            return;
        }
    }
}

fn write_response(conn: &mut Conn, r: &Response, keep: bool) -> io::Result<()> {
    let mut head = format!(
        "HTTP/1.1 {} {}\r\nContent-Type: {}\r\nContent-Length: {}\r\nCache-Control: {}\r\n",
        r.status,
        reason(r.status),
        r.content_type,
        r.body.len(),
        if r.max_age == 0 {
            "no-store".to_string()
        } else {
            format!("public, max-age={}", r.max_age)
        }
    );
    if r.gzip {
        head.push_str("Content-Encoding: gzip\r\n");
    }
    if r.status == 405 {
        head.push_str("Allow: GET\r\n");
    }
    head.push_str(if keep {
        "Connection: keep-alive\r\n\r\n"
    } else {
        "Connection: close\r\n\r\n"
    });
    // One write for the head and the body together, so a small response is
    // one segment rather than two and Nagle has nothing to hold on to.
    let mut out = head.into_bytes();
    out.extend_from_slice(&r.body);
    conn.send(&out)
}

fn reason(code: u16) -> &'static str {
    match code {
        200 => "OK",
        400 => "Bad Request",
        404 => "Not Found",
        405 => "Method Not Allowed",
        431 => "Request Header Fields Too Large",
        503 => "Service Unavailable",
        _ => "Status",
    }
}

// ---- WebSocket (RFC 6455), server side only --------------------------------
//
// Enough of the protocol to push JSON down one connection: the handshake, text
// frames out, and enough frame parsing to notice a close or a ping. Server
// frames are never masked; client frames always are.

/// An open WebSocket, handed to [`Handler::websocket`] on its own thread.
pub struct WebSocket {
    conn: Conn,
    closed: bool,
}

impl WebSocket {
    /// One text frame, unfragmented and unmasked.
    pub fn send_text(&mut self, s: &str) -> io::Result<()> {
        let f = frame(0x1, s.as_bytes());
        self.conn.send(&f)
    }

    /// The next message, waiting at most `timeout`.
    ///
    /// `Ok(None)` means nothing arrived in time, which is the normal answer on
    /// a connection where the traffic is all outbound. An error of kind
    /// `UnexpectedEof` means the peer has gone.
    ///
    /// A ping is answered with a pong here rather than left to the caller,
    /// since the caller has no way to send one and there is only one right
    /// reply; a close is answered with a close for the same reason. Both are
    /// still reported.
    pub fn poll(&mut self, timeout: Duration) -> io::Result<Option<Message>> {
        let deadline = Instant::now() + timeout;
        loop {
            if let Some(msg) = self.take_frame()? {
                match &msg {
                    Message::Ping(payload) => {
                        let f = frame(0xA, payload);
                        self.conn.send(&f)?;
                    }
                    Message::Close => self.close(),
                    _ => {}
                }
                return Ok(Some(msg));
            }
            let left = deadline.saturating_duration_since(Instant::now());
            if left.is_zero() {
                return Ok(None);
            }
            // A zero timeout means "block for ever" to the kernel, so never
            // ask for less than a tick.
            self.conn.timeout(Some(left.max(Duration::from_millis(1))));
            match self.conn.fill() {
                Ok(true) => {}
                Ok(false) => {
                    return Err(io::Error::new(
                        ErrorKind::UnexpectedEof,
                        "the peer closed the connection",
                    ))
                }
                // Linux reports a read timeout as WouldBlock, Windows as
                // TimedOut; either way the time is up and nothing came.
                Err(e) if matches!(e.kind(), ErrorKind::WouldBlock | ErrorKind::TimedOut) => {
                    return Ok(None)
                }
                Err(e) => return Err(e),
            }
        }
    }

    /// A close frame, best effort. Doing it twice sends one frame.
    pub fn close(&mut self) {
        if !self.closed {
            self.closed = true;
            let _ = self.conn.send(&frame(0x8, b""));
        }
    }

    /// One whole frame out of the buffer, or None when more bytes are needed.
    ///
    /// Fragments are not reassembled and binary frames are dropped: a browser
    /// sends its close and its pong in one frame each, and nothing here has a
    /// use for anything else.
    fn take_frame(&mut self) -> io::Result<Option<Message>> {
        loop {
            let buf = &self.conn.buf;
            if buf.len() < 2 {
                return Ok(None);
            }
            let opcode = buf[0] & 0x0F;
            let masked = buf[1] & 0x80 != 0;
            let short = usize::from(buf[1] & 0x7F);
            let (len, mut at) = match short {
                126 => {
                    if buf.len() < 4 {
                        return Ok(None);
                    }
                    (usize::from(u16::from_be_bytes([buf[2], buf[3]])), 4)
                }
                127 => {
                    if buf.len() < 10 {
                        return Ok(None);
                    }
                    let n = u64::from_be_bytes(buf[2..10].try_into().unwrap());
                    (usize::try_from(n).unwrap_or(usize::MAX), 10)
                }
                n => (n, 2),
            };
            if len > MAX_FRAME {
                return Err(io::Error::new(
                    ErrorKind::InvalidData,
                    "frame exceeds MAX_FRAME",
                ));
            }
            let mask = if masked {
                if buf.len() < at + 4 {
                    return Ok(None);
                }
                let m = [buf[at], buf[at + 1], buf[at + 2], buf[at + 3]];
                at += 4;
                m
            } else {
                [0; 4]
            };
            if buf.len() < at + len {
                return Ok(None);
            }
            let mut body: Vec<u8> = buf[at..at + len].to_vec();
            if masked {
                for (i, b) in body.iter_mut().enumerate() {
                    *b ^= mask[i % 4];
                }
            }
            self.conn.buf.drain(..at + len);
            return Ok(Some(match opcode {
                // Invalid UTF-8 is replaced, not treated as fatal.
                0x1 => Message::Text(String::from_utf8_lossy(&body).into_owned()),
                0x8 => Message::Close,
                0x9 => Message::Ping(body),
                0xA => Message::Pong,
                _ => continue,
            }));
        }
    }
}

impl Drop for WebSocket {
    fn drop(&mut self) {
        self.close();
    }
}

/// One unfragmented frame, unmasked, as a server sends them.
fn frame(opcode: u8, payload: &[u8]) -> Vec<u8> {
    let mut out = Vec::with_capacity(payload.len() + 10);
    out.push(0x80 | opcode);
    let n = payload.len();
    if n < 126 {
        out.push(n as u8);
    } else if n < 65536 {
        out.push(126);
        out.extend_from_slice(&(n as u16).to_be_bytes());
    } else {
        out.push(127);
        out.extend_from_slice(&(n as u64).to_be_bytes());
    }
    out.extend_from_slice(payload);
    out
}

/// The handshake's answer: the client's key and a fixed GUID, hashed and
/// base64'd, which proves to the browser that this is a WebSocket server and
/// not a cache that happened to return a 101.
fn accept_key(key: &str) -> String {
    let mut buf = key.trim().to_string();
    buf.push_str(WS_GUID);
    b64_encode(&sha1(buf.as_bytes()))
}

// ---- SHA-1 and base64, by hand ---------------------------------------------
//
// Both are here only to turn one short header into another, which is not worth
// a dependency and its supply chain. SHA-1 is broken for signatures and
// irrelevant to security here: the handshake uses it as a checksum that a
// proxy cannot fake by accident.

fn sha1(data: &[u8]) -> [u8; 20] {
    let mut h: [u32; 5] = [
        0x6745_2301,
        0xEFCD_AB89,
        0x98BA_DCFE,
        0x1032_5476,
        0xC3D2_E1F0,
    ];
    let mut msg = data.to_vec();
    let bits = (data.len() as u64) * 8;
    msg.push(0x80);
    while msg.len() % 64 != 56 {
        msg.push(0);
    }
    msg.extend_from_slice(&bits.to_be_bytes());

    for block in msg.chunks_exact(64) {
        let mut w = [0u32; 80];
        for (word, bytes) in w.iter_mut().zip(block.chunks_exact(4)) {
            *word = u32::from_be_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        }
        for i in 16..80 {
            w[i] = (w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16]).rotate_left(1);
        }
        let (mut a, mut b, mut c, mut d, mut e) = (h[0], h[1], h[2], h[3], h[4]);
        for (i, &word) in w.iter().enumerate() {
            let (f, k) = match i {
                0..=19 => ((b & c) | (!b & d), 0x5A82_7999),
                20..=39 => (b ^ c ^ d, 0x6ED9_EBA1),
                40..=59 => ((b & c) | (b & d) | (c & d), 0x8F1B_BCDC),
                _ => (b ^ c ^ d, 0xCA62_C1D6),
            };
            let t = a
                .rotate_left(5)
                .wrapping_add(f)
                .wrapping_add(e)
                .wrapping_add(k)
                .wrapping_add(word);
            e = d;
            d = c;
            c = b.rotate_left(30);
            b = a;
            a = t;
        }
        for (slot, add) in h.iter_mut().zip([a, b, c, d, e]) {
            *slot = slot.wrapping_add(add);
        }
    }

    let mut out = [0u8; 20];
    for (chunk, word) in out.chunks_exact_mut(4).zip(h) {
        chunk.copy_from_slice(&word.to_be_bytes());
    }
    out
}

const B64: &[u8; 64] = b"ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

fn b64_encode(data: &[u8]) -> String {
    let mut out = String::with_capacity(data.len().div_ceil(3) * 4);
    for c in data.chunks(3) {
        let n = (u32::from(c[0]) << 16)
            | (u32::from(c.get(1).copied().unwrap_or(0)) << 8)
            | u32::from(c.get(2).copied().unwrap_or(0));
        let sym = |shift: u32| char::from(B64[((n >> shift) & 63) as usize]);
        out.push(sym(18));
        out.push(sym(12));
        out.push(if c.len() > 1 { sym(6) } else { '=' });
        out.push(if c.len() > 2 { sym(0) } else { '=' });
    }
    out
}

// ---- gzip ------------------------------------------------------------------

/// Deflate with a gzip wrapper, which is what `Content-Encoding: gzip` means.
///
/// Public because the aircraft side compresses its own JSON: a poll and a
/// WebSocket push carry the same bytes, and there is no sense in building them
/// twice. Level 6: level 9 measured under a percent smaller on these updates
/// for several times the work.
pub fn gzip(data: &[u8]) -> Vec<u8> {
    let mut enc = flate2::write::GzEncoder::new(Vec::new(), flate2::Compression::new(6));
    // Writing to a Vec cannot fail, and neither can finishing one.
    let _ = enc.write_all(data);
    enc.finish().unwrap_or_default()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::BufRead;

    // ---- the handler the socket tests drive --------------------------------

    struct Demo;

    impl Handler for Demo {
        fn get(&self, path: &str, query: &str) -> Response {
            match path {
                "/hi" => Response::ok("text/plain", format!("hello [{query}]").into_bytes()),
                "/zip" => Response::gzipped("application/json", gzip(br#"{"a":1}"#)),
                "/logo" => Response::cached("image/png", b"png".to_vec(), 86400),
                _ => Response::status(404, "no such thing"),
            }
        }

        /// `?quiet` waits for a message that never comes and reports what
        /// poll() said; anything else greets the client and then answers its
        /// control frames until it goes away.
        fn websocket(&self, query: &str, mut ws: WebSocket) {
            if query == "quiet" {
                ws.send_text("ready").unwrap();
                let t0 = Instant::now();
                let got = ws.poll(Duration::from_millis(250));
                let waited = t0.elapsed().as_millis();
                let word = if matches!(got, Ok(None)) {
                    "quiet"
                } else {
                    "not quiet"
                };
                let _ = ws.send_text(&format!("{word} {waited}"));
                return;
            }
            ws.send_text(&format!("hello [{query}]")).unwrap();
            loop {
                match ws.poll(Duration::from_millis(500)) {
                    Ok(Some(Message::Close)) | Err(_) => return,
                    Ok(Some(Message::Text(t))) => {
                        if ws.send_text(&format!("echo {t}")).is_err() {
                            return;
                        }
                    }
                    Ok(Some(_)) | Ok(None) => {}
                }
            }
        }
    }

    fn serving() -> SocketAddr {
        let s = Server::bind("127.0.0.1:0".parse().unwrap(), None).unwrap();
        assert!(!s.is_tls());
        let addr = s.local_addr();
        std::thread::spawn(move || {
            let _ = s.serve(Arc::new(Demo));
        });
        addr
    }

    /// Status code, headers and body of one response, read from the socket.
    fn read_response(rd: &mut impl BufRead) -> (u16, Vec<(String, String)>, Vec<u8>) {
        let mut line = String::new();
        rd.read_line(&mut line).unwrap();
        let code: u16 = line.split(' ').nth(1).unwrap_or("0").parse().unwrap();
        let mut headers = Vec::new();
        loop {
            let mut h = String::new();
            rd.read_line(&mut h).unwrap();
            let h = h.trim_end();
            if h.is_empty() {
                break;
            }
            let (k, v) = h.split_once(':').unwrap();
            headers.push((k.trim().to_ascii_lowercase(), v.trim().to_string()));
        }
        let len: usize = headers
            .iter()
            .find(|(k, _)| k == "content-length")
            .map_or("0", |(_, v)| v.as_str())
            .parse()
            .unwrap();
        let mut body = vec![0u8; len];
        rd.read_exact(&mut body).unwrap();
        (code, headers, body)
    }

    fn header<'a>(hs: &'a [(String, String)], name: &str) -> Option<&'a str> {
        hs.iter().find(|(k, _)| k == name).map(|(_, v)| v.as_str())
    }

    fn connect(addr: SocketAddr) -> TcpStream {
        let s = TcpStream::connect(addr).unwrap();
        s.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
        s
    }

    // ---- the pieces --------------------------------------------------------

    /// The known vectors, including the one RFC 6455 works through, so a
    /// browser's handshake is checked against the document rather than against
    /// this implementation of it.
    #[test]
    fn sha1_matches_the_published_vectors() {
        let hex = |d: &[u8]| {
            sha1(d)
                .iter()
                .map(|b| format!("{b:02x}"))
                .collect::<String>()
        };
        assert_eq!(hex(b""), "da39a3ee5e6b4b0d3255bfef95601890afd80709");
        assert_eq!(hex(b"abc"), "a9993e364706816aba3e25717850c26c9cd0d89d");
        // Two blocks, and padding that spills into a third.
        assert_eq!(
            hex(b"abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq"),
            "84983e441c3bd26ebaae4aa1f95129e5e54670f1"
        );
        assert_eq!(
            hex(&[b'a'; 1000]),
            "291e9a6c66994949b57ba5e650361e98fc36b1ba"
        );

        assert_eq!(
            accept_key("dGhlIHNhbXBsZSBub25jZQ=="),
            "s3pPLMBiTxaQ9kYGzzhZRbK+xOo="
        );
    }

    /// Decoded only here: the server has no reason to read base64, but a
    /// round trip is a better test of the encoder than a table of six.
    fn b64_decode(s: &str) -> Vec<u8> {
        let mut out = Vec::new();
        let mut acc = 0u32;
        let mut bits = 0;
        for ch in s.bytes().take_while(|&b| b != b'=') {
            let v = B64.iter().position(|&c| c == ch).expect("base64 alphabet") as u32;
            acc = (acc << 6) | v;
            bits += 6;
            if bits >= 8 {
                bits -= 8;
                out.push((acc >> bits) as u8);
            }
        }
        out
    }

    #[test]
    fn base64_round_trips() {
        assert_eq!(b64_encode(b""), "");
        assert_eq!(b64_encode(b"f"), "Zg==");
        assert_eq!(b64_encode(b"fo"), "Zm8=");
        assert_eq!(b64_encode(b"foo"), "Zm9v");
        assert_eq!(b64_encode(b"foobar"), "Zm9vYmFy");
        assert_eq!(b64_encode(&[0xFB, 0xFF, 0xFE]), "+//+");

        for n in 0..=64usize {
            let bytes: Vec<u8> = (0..n).map(|i| (i * 37 + 11) as u8).collect();
            let text = b64_encode(&bytes);
            assert_eq!(
                text.len(),
                n.div_ceil(3) * 4,
                "padded to a multiple of four"
            );
            assert_eq!(b64_decode(&text), bytes, "round trip of {n} bytes");
        }
    }

    #[test]
    fn gzip_is_gzip_and_comes_back() {
        let body = b"{\"ac\":{}}".repeat(200);
        let z = gzip(&body);
        assert_eq!(&z[..2], &[0x1f, 0x8b], "the gzip magic, not raw deflate");
        assert!(
            z.len() < body.len() / 4,
            "and it actually compressed: {} bytes",
            z.len()
        );
        let mut back = Vec::new();
        flate2::read::GzDecoder::new(&z[..])
            .read_to_end(&mut back)
            .unwrap();
        assert_eq!(back, body);
        assert_eq!(
            &gzip(b"")[..2],
            &[0x1f, 0x8b],
            "an empty body is still a gzip member"
        );
    }

    // ---- over a real socket ------------------------------------------------

    /// A body, a 404, and the headers the map depends on. Both requests go
    /// down one connection, which is the keep-alive path.
    #[test]
    fn a_get_is_answered_and_an_unknown_path_is_404() {
        let addr = serving();
        let s = connect(addr);
        let mut rd = io::BufReader::new(s.try_clone().unwrap());
        let mut wr = s;

        wr.write_all(b"GET /hi?since=7 HTTP/1.1\r\nHost: x\r\n\r\n")
            .unwrap();
        let (code, hs, body) = read_response(&mut rd);
        assert_eq!(code, 200);
        assert_eq!(body, b"hello [since=7]");
        assert_eq!(header(&hs, "content-length"), Some("15"));
        assert_eq!(header(&hs, "cache-control"), Some("no-store"));
        assert_eq!(header(&hs, "content-encoding"), None);

        // The same connection again: the server kept it.
        wr.write_all(b"GET /zip HTTP/1.1\r\nHost: x\r\n\r\n")
            .unwrap();
        let (code, hs, body) = read_response(&mut rd);
        assert_eq!(code, 200);
        assert_eq!(header(&hs, "content-encoding"), Some("gzip"));
        let mut back = Vec::new();
        flate2::read::GzDecoder::new(&body[..])
            .read_to_end(&mut back)
            .unwrap();
        assert_eq!(back, br#"{"a":1}"#);

        wr.write_all(b"GET /nowhere HTTP/1.1\r\nHost: x\r\nConnection: close\r\n\r\n")
            .unwrap();
        let (code, hs, body) = read_response(&mut rd);
        assert_eq!(code, 404);
        assert_eq!(header(&hs, "connection"), Some("close"));
        assert!(String::from_utf8_lossy(&body).contains("no such thing"));
    }

    /// Anything but a GET is refused, and so is a head that will not stop.
    #[test]
    fn only_get_is_served_and_a_huge_head_is_refused() {
        let addr = serving();
        let s = connect(addr);
        let mut rd = io::BufReader::new(s.try_clone().unwrap());
        (&s).write_all(b"POST /hi HTTP/1.1\r\nHost: x\r\n\r\n")
            .unwrap();
        let (code, hs, _) = read_response(&mut rd);
        assert_eq!(code, 405);
        assert_eq!(header(&hs, "allow"), Some("GET"));

        let s = connect(addr);
        let mut rd = io::BufReader::new(s.try_clone().unwrap());
        let mut wr = &s;
        wr.write_all(b"GET /hi HTTP/1.1\r\n").unwrap();
        let junk = format!("X-Pad: {}\r\n", "y".repeat(1000));
        // The write may fail part-way once the server has answered and closed,
        // which is the point of the test rather than a problem.
        for _ in 0..20 {
            if wr.write_all(junk.as_bytes()).is_err() {
                break;
            }
        }
        let _ = wr.flush();
        let (code, _, _) = read_response(&mut rd);
        assert_eq!(code, 431);
    }

    /// Everything a browser does: the handshake, a frame from the server, a
    /// masked frame each way, and a close that is answered.
    #[test]
    fn a_websocket_carries_frames_both_ways() {
        let addr = serving();
        let s = connect(addr);
        let mut rd = io::BufReader::new(s.try_clone().unwrap());
        let mut wr = &s;
        wr.write_all(
            b"GET /ws?since=3 HTTP/1.1\r\nHost: x\r\nUpgrade: WebSocket\r\n\
              Connection: Upgrade\r\nSec-WebSocket-Version: 13\r\n\
              Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n\r\n",
        )
        .unwrap();

        let mut line = String::new();
        rd.read_line(&mut line).unwrap();
        assert!(line.starts_with("HTTP/1.1 101 "), "{line:?}");
        let mut accept = None;
        loop {
            let mut h = String::new();
            rd.read_line(&mut h).unwrap();
            let h = h.trim_end().to_string();
            if h.is_empty() {
                break;
            }
            if let Some((k, v)) = h.split_once(':') {
                if k.eq_ignore_ascii_case("sec-websocket-accept") {
                    accept = Some(v.trim().to_string());
                }
            }
        }
        assert_eq!(accept.as_deref(), Some("s3pPLMBiTxaQ9kYGzzhZRbK+xOo="));

        let (op, body) = recv_frame(&mut rd);
        assert_eq!((op, body.as_slice()), (0x1, &b"hello [since=3]"[..]));

        // A masked text frame, echoed back unmasked.
        send_frame(&mut wr, 0x1, b"ping me");
        let (op, body) = recv_frame(&mut rd);
        assert_eq!(
            (op, String::from_utf8_lossy(&body).to_string()),
            (0x1, "echo ping me".to_string())
        );

        // A ping, answered with a pong carrying the same payload.
        send_frame(&mut wr, 0x9, b"are you there");
        let (op, body) = recv_frame(&mut rd);
        assert_eq!((op, body.as_slice()), (0xA, &b"are you there"[..]));

        send_frame(&mut wr, 0x8, b"");
        let (op, _) = recv_frame(&mut rd);
        assert_eq!(op, 0x8, "the close is answered with a close");
    }

    /// A connection where the client says nothing: poll() gives up after its
    /// timeout and says so, rather than blocking the push loop for ever.
    #[test]
    fn poll_gives_up_when_nothing_arrives() {
        let addr = serving();
        let s = connect(addr);
        let mut rd = io::BufReader::new(s.try_clone().unwrap());
        (&s).write_all(
            b"GET /ws?quiet HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\n\
              Connection: Upgrade\r\nSec-WebSocket-Key: AAAAAAAAAAAAAAAAAAAAAA==\r\n\r\n",
        )
        .unwrap();
        let mut line = String::new();
        rd.read_line(&mut line).unwrap();
        assert!(line.starts_with("HTTP/1.1 101 "), "{line:?}");
        loop {
            let mut h = String::new();
            rd.read_line(&mut h).unwrap();
            if h.trim_end().is_empty() {
                break;
            }
        }
        let (_, ready) = recv_frame(&mut rd);
        assert_eq!(ready, b"ready");
        let (_, verdict) = recv_frame(&mut rd);
        let verdict = String::from_utf8_lossy(&verdict).to_string();
        let (word, waited) = verdict.split_once(' ').unwrap();
        assert_eq!(word, "quiet", "{verdict}");
        let waited: u128 = waited.parse().unwrap();
        assert!(
            (240..2000).contains(&waited),
            "waited about the timeout, not {waited} ms"
        );
    }

    /// A certificate that is not there is an error from bind, not a panic in
    /// the accept loop later on.
    #[test]
    fn tls_needs_a_certificate_that_exists() {
        let missing = Path::new("/nonexistent/anrb-map-test.pem");
        let e = Server::bind("127.0.0.1:0".parse().unwrap(), Some((missing, missing))).err();
        assert_eq!(
            e.expect("bound with a certificate that does not exist")
                .kind(),
            ErrorKind::NotFound
        );
    }

    /// A directory of this test's own, gone by the time the test returns.
    struct Temp(std::path::PathBuf);

    impl Temp {
        fn new(what: &str) -> Temp {
            let dir = std::env::temp_dir().join(format!("anrb-http-{what}-{}", std::process::id()));
            let _ = std::fs::remove_dir_all(&dir);
            std::fs::create_dir_all(&dir).expect("a temporary directory");
            Temp(dir)
        }

        /// Write a file in the directory and give its path.
        fn file(&self, name: &str, text: &str) -> std::path::PathBuf {
            let path = self.0.join(name);
            std::fs::write(&path, text).expect("write a file");
            path
        }
    }

    impl Drop for Temp {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    /// With a certificate and key, the server speaks TLS, and a client that
    /// trusts the certificate gets the same answers as over plain TCP.
    #[test]
    fn a_get_over_tls_is_answered() {
        let tmp = Temp::new("tls");
        let cert = tmp.file("cert.pem", crate::test_tls::CERT);
        let key = tmp.file("key.pem", crate::test_tls::KEY);
        let s = Server::bind("127.0.0.1:0".parse().unwrap(), Some((&cert, &key)))
            .expect("bind with TLS");
        assert!(s.is_tls());
        let addr = s.local_addr();
        std::thread::spawn(move || {
            let _ = s.serve(Arc::new(Demo));
        });

        let name = rustls::pki_types::ServerName::try_from("127.0.0.1").unwrap();
        let client =
            rustls::ClientConnection::new(Arc::new(crate::test_tls::client()), name).unwrap();
        let mut tls = io::BufReader::new(rustls::StreamOwned::new(client, connect(addr)));
        tls.get_mut()
            .write_all(b"GET /hi?over=tls HTTP/1.1\r\nHost: x\r\n\r\n")
            .unwrap();
        let (code, _, body) = read_response(&mut tls);
        assert_eq!((code, body.as_slice()), (200, &b"hello [over=tls]"[..]));
        // A second request on the same TLS connection.
        tls.get_mut()
            .write_all(b"GET /nowhere HTTP/1.1\r\nHost: x\r\n\r\n")
            .unwrap();
        assert_eq!(read_response(&mut tls).0, 404);
    }

    /// PEM files with no certificate, with no key, or with a key that is not
    /// one are refused by bind.
    #[test]
    fn tls_needs_a_certificate_and_a_key() {
        let tmp = Temp::new("pem");
        let cert = tmp.file("cert.pem", crate::test_tls::CERT);
        let key = tmp.file("key.pem", crate::test_tls::KEY);
        let empty = tmp.file("empty.pem", "nothing in PEM form\n");
        let broken = tmp.file(
            "broken.pem",
            "-----BEGIN PRIVATE KEY-----\nAAAA\n-----END PRIVATE KEY-----\n",
        );
        let addr: SocketAddr = "127.0.0.1:0".parse().unwrap();
        let refused = |c: &Path, k: &Path| Server::bind(addr, Some((c, k))).err().expect("refused");
        assert!(refused(&empty, &key).to_string().contains("no certificate"));
        assert!(refused(&cert, &empty)
            .to_string()
            .contains("no private key"));
        assert_eq!(refused(&cert, &broken).kind(), ErrorKind::InvalidData);
    }

    /// A head that is not a request line, and an upgrade with no key, are
    /// answered with 400 and the connection is closed.
    #[test]
    fn a_malformed_request_is_400() {
        let addr = serving();
        for req in [
            &b"NONSENSE\r\n\r\n"[..],
            b"GET /ws HTTP/1.1\r\nUpgrade: websocket\r\n\r\n",
        ] {
            let s = connect(addr);
            (&s).write_all(req).unwrap();
            let mut rd = io::BufReader::new(s);
            let (code, hs, body) = read_response(&mut rd);
            assert_eq!(code, 400);
            assert_eq!(header(&hs, "connection"), Some("close"));
            assert!(!body.is_empty());
            let mut rest = Vec::new();
            assert_eq!(rd.read_to_end(&mut rest).unwrap(), 0, "and nothing follows");
        }
    }

    /// Something that does not change is sent with a max-age.
    #[test]
    fn a_cached_response_says_how_long_to_keep_it() {
        let addr = serving();
        let s = connect(addr);
        (&s).write_all(b"GET /logo HTTP/1.1\r\n\r\n").unwrap();
        let (code, hs, body) = read_response(&mut io::BufReader::new(s));
        assert_eq!((code, body.as_slice()), (200, &b"png"[..]));
        assert_eq!(header(&hs, "cache-control"), Some("public, max-age=86400"));
        assert_eq!(header(&hs, "content-type"), Some("image/png"));
    }

    #[test]
    fn every_status_has_a_reason() {
        assert_eq!(reason(200), "OK");
        assert_eq!(reason(400), "Bad Request");
        assert_eq!(reason(404), "Not Found");
        assert_eq!(reason(405), "Method Not Allowed");
        assert_eq!(reason(431), "Request Header Fields Too Large");
        assert_eq!(reason(503), "Service Unavailable");
        assert_eq!(reason(418), "Status");
    }

    /// A client that connects and hangs up without a word is let go.
    #[test]
    fn a_client_that_says_nothing_is_let_go() {
        let addr = serving();
        let s = connect(addr);
        s.shutdown(std::net::Shutdown::Write).unwrap();
        let mut rest = Vec::new();
        assert_eq!((&s).read_to_end(&mut rest).unwrap(), 0);
    }

    // ---- frames, without the handshake -------------------------------------

    /// A WebSocket on one end of a local connection, and the other end.
    fn pair() -> (WebSocket, TcpStream) {
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let client = connect(listener.local_addr().unwrap());
        let (server, _) = listener.accept().unwrap();
        let conn = Conn {
            sock: server.try_clone().unwrap(),
            io: Io::Plain(server),
            buf: Vec::new(),
        };
        (
            WebSocket {
                conn,
                closed: false,
            },
            client,
        )
    }

    /// A client frame with the given opcode and, if given, mask. The length
    /// takes the shortest form that holds it.
    fn client_frame(opcode: u8, payload: &[u8], mask: Option<[u8; 4]>) -> Vec<u8> {
        let flag = if mask.is_some() { 0x80 } else { 0 };
        let mut out = vec![0x80 | opcode];
        let n = payload.len();
        if n < 126 {
            out.push(flag | n as u8);
        } else if n < 65536 {
            out.push(flag | 126);
            out.extend_from_slice(&(n as u16).to_be_bytes());
        } else {
            out.push(flag | 127);
            out.extend_from_slice(&(n as u64).to_be_bytes());
        }
        let m = mask.unwrap_or([0; 4]);
        if mask.is_some() {
            out.extend_from_slice(&m);
        }
        out.extend(payload.iter().enumerate().map(|(i, b)| b ^ m[i % 4]));
        out
    }

    fn text(m: Option<Message>) -> String {
        let Some(Message::Text(t)) = m else {
            panic!("not a text message")
        };
        t
    }

    /// Frames with a 16-bit and a 64-bit length, masked or not, and the
    /// control frames. A binary frame is skipped.
    #[test]
    fn frames_of_every_length_are_read() {
        let (mut ws, _client) = pair();
        let mid = "m".repeat(300);
        let long = "l".repeat(70_000);
        let mut bytes = client_frame(0x1, mid.as_bytes(), Some([1, 2, 3, 4]));
        bytes.extend(client_frame(0x1, long.as_bytes(), None));
        bytes.extend(client_frame(0xA, b"", Some([9, 9, 9, 9])));
        bytes.extend(client_frame(0x2, b"binary", None));
        bytes.extend(client_frame(0x1, b"after", Some([5, 6, 7, 8])));
        ws.conn.buf = bytes;
        assert_eq!(text(ws.take_frame().unwrap()), mid);
        assert_eq!(text(ws.take_frame().unwrap()), long);
        assert!(matches!(ws.take_frame().unwrap(), Some(Message::Pong)));
        assert_eq!(
            text(ws.take_frame().unwrap()),
            "after",
            "the binary frame is passed over"
        );
        assert!(ws.take_frame().unwrap().is_none());
    }

    /// A frame that has only partly arrived is waited for, whichever part of
    /// it is missing.
    #[test]
    fn a_partial_frame_waits_for_the_rest() {
        let (mut ws, _client) = pair();
        let whole = client_frame(0x1, "p".repeat(300).as_bytes(), Some([1, 2, 3, 4]));
        for at in [1, 3, 5, 7, 100] {
            ws.conn.buf = whole[..at].to_vec();
            assert!(ws.take_frame().unwrap().is_none(), "{at} bytes");
            assert_eq!(ws.conn.buf.len(), at, "and nothing is consumed");
        }
        let whole = client_frame(0x1, "q".repeat(70_000).as_bytes(), None);
        ws.conn.buf = whole[..9].to_vec();
        assert!(ws.take_frame().unwrap().is_none());
        ws.conn.buf = whole;
        assert_eq!(text(ws.take_frame().unwrap()).len(), 70_000);
    }

    /// A frame that says it is longer than MAX_FRAME is an error before any of
    /// it is read.
    #[test]
    fn a_frame_over_the_limit_is_refused() {
        let (mut ws, _client) = pair();
        let mut head = vec![0x81, 127];
        head.extend_from_slice(&(MAX_FRAME as u64 + 1).to_be_bytes());
        ws.conn.buf = head;
        let e = ws.take_frame().err().expect("an error");
        assert_eq!(e.kind(), ErrorKind::InvalidData);
    }

    /// The server's own frames use the longer length forms when they must,
    /// and a client can read them.
    #[test]
    fn long_server_frames_use_the_extended_lengths() {
        assert_eq!(frame(0x1, &[0; 125])[..2], [0x81, 125]);
        assert_eq!(frame(0x1, &[0; 126])[..4], [0x81, 126, 0, 126]);
        assert_eq!(frame(0x1, &[0; 65535])[..4], [0x81, 126, 0xFF, 0xFF]);
        assert_eq!(
            frame(0x2, &[0; 65536])[..10],
            [0x82, 127, 0, 0, 0, 0, 0, 1, 0, 0]
        );

        let (mut ws, client) = pair();
        let mid = "a".repeat(1000);
        let long = "b".repeat(100_000);
        let reader = std::thread::spawn(move || {
            let mut rd = io::BufReader::new(client);
            (recv_frame(&mut rd), recv_frame(&mut rd))
        });
        ws.send_text(&mid).unwrap();
        ws.send_text(&long).unwrap();
        let (a, b) = reader.join().unwrap();
        assert_eq!(a, (0x1, mid.into_bytes()));
        assert_eq!(b, (0x1, long.into_bytes()));
    }

    /// A peer that hangs up is an end of file from poll, not a quiet spell.
    #[test]
    fn poll_reports_a_peer_that_hung_up() {
        let (mut ws, client) = pair();
        drop(client);
        let e = ws.poll(Duration::from_secs(5)).err().expect("an error");
        assert_eq!(e.kind(), ErrorKind::UnexpectedEof);
    }

    /// A frame that arrives in pieces is put together across reads.
    #[test]
    fn poll_reads_a_frame_that_arrives_in_pieces() {
        let (mut ws, mut client) = pair();
        let whole = client_frame(0x1, "z".repeat(200).as_bytes(), Some([1, 2, 3, 4]));
        client.write_all(&whole[..3]).unwrap();
        assert!(ws.poll(Duration::from_millis(50)).unwrap().is_none());
        client.write_all(&whole[3..]).unwrap();
        assert_eq!(
            text(ws.poll(Duration::from_secs(5)).unwrap()),
            "z".repeat(200)
        );
    }

    /// Closing twice, and dropping after that, sends exactly one close frame.
    #[test]
    fn a_close_is_sent_once() {
        let (mut ws, client) = pair();
        ws.close();
        ws.close();
        drop(ws);
        let mut rest = Vec::new();
        (&client).read_to_end(&mut rest).unwrap();
        assert_eq!(rest, [0x88, 0x00]);
    }

    // ---- a client's side of the framing, for the tests ---------------------

    fn send_frame(wr: &mut impl Write, opcode: u8, payload: &[u8]) {
        let mask = [0x37, 0xfa, 0x21, 0x3d];
        let mut out = vec![0x80 | opcode, 0x80 | payload.len() as u8];
        out.extend_from_slice(&mask);
        out.extend(payload.iter().enumerate().map(|(i, b)| b ^ mask[i % 4]));
        wr.write_all(&out).unwrap();
        wr.flush().unwrap();
    }

    fn recv_frame(rd: &mut impl Read) -> (u8, Vec<u8>) {
        let mut h = [0u8; 2];
        rd.read_exact(&mut h).unwrap();
        assert_eq!(h[0] & 0x80, 0x80, "the server sends whole frames");
        assert_eq!(h[1] & 0x80, 0, "a server frame is never masked");
        let len = match h[1] & 0x7F {
            126 => {
                let mut e = [0u8; 2];
                rd.read_exact(&mut e).unwrap();
                usize::from(u16::from_be_bytes(e))
            }
            127 => {
                let mut e = [0u8; 8];
                rd.read_exact(&mut e).unwrap();
                u64::from_be_bytes(e) as usize
            }
            n => usize::from(n),
        };
        let mut body = vec![0u8; len];
        rd.read_exact(&mut body).unwrap();
        (h[0] & 0x0F, body)
    }
}
