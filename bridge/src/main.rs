//! The map bridge: read the receiver's feed, keep a quarter of an hour of it,
//! and serve it to a browser.
//!
//!     anrb-map
//!     anrb-map --feed sbs --sbs-port 30003     # a BaseStation feed
//!     anrb-map --cert cert.pem --key key.pem   # HTTPS, which the location dot needs
//!
//! The tracker is the driver crate's, linked in rather than run as a separate
//! process: the bridge decodes the feed itself and serves the result.

use std::collections::HashMap;
use std::io::{ErrorKind, Read};
use std::net::{SocketAddr, TcpStream, ToSocketAddrs};
use std::path::{Path, PathBuf};
use std::sync::Arc;
use std::time::{Duration, Instant};

use anrb::beast;
use anrb::tracker::{Aircraft, Tracker, Update};

use anrb_map::hexdb::{Hexdb, Lookup};
use anrb_map::http::{gzip, Handler, Message, Response, Server, WebSocket};
use anrb_map::map::{self, State};

const USAGE: &str = "\
anrb-map - the live map's bridge

  --feed beast|sbs    which of the receiver's feeds to read (default beast)
  --host HOST         the receiver (default 127.0.0.1)
  --beast-port PORT   default 30005
  --sbs-port PORT     default 30003
  --http-port PORT    default 8080
  --bind ADDR         address to listen on (default 0.0.0.0, every interface)
  --cert FILE         serve HTTPS with this certificate chain (PEM)
  --key FILE          the certificate's private key (PEM)
  --root DIR          where index.html and its files live (default web)
  --logos DIR         directory of <ICAO code>.bmp airline logos (e.g. the
                      vendor's Data/Logos), served as /logo/<CODE>; off by
                      default
  --cache DIR         where to keep what hexdb.io answers (default
                      ~/.cache/anrb-map); made if it is not there
  --no-hexdb          answer no lookups and make no outbound request at all;
                      nothing is read from or written to the cache, and the
                      page is told not to ask
  -h, --help          show this help
";

/// Print `msg` and the usage text, and exit with status 2.
fn usage_error(msg: &str) -> ! {
    eprint!("{msg}\n{USAGE}");
    std::process::exit(2)
}

struct Args {
    beast: bool,
    host: String,
    feed_port: u16,
    http: SocketAddr,
    cert: Option<PathBuf>,
    key: Option<PathBuf>,
    root: PathBuf,
    logos: Option<PathBuf>,
    cache: PathBuf,
    hexdb: bool,
}

fn args() -> Args {
    let mut a = Args {
        beast: true,
        host: "127.0.0.1".into(),
        feed_port: 0,
        http: ([0, 0, 0, 0], 8080).into(),
        cert: None,
        key: None,
        root: "web".into(),
        logos: None,
        cache: cache_home(),
        hexdb: true,
    };
    let (mut beast_port, mut sbs_port, mut bind, mut port) = (30005u16, 30003u16, "0.0.0.0".to_string(), 8080u16);
    let argv: Vec<String> = std::env::args().collect();
    let mut i = 1;
    let need = |i: usize, argv: &[String]| -> String {
        argv.get(i + 1).cloned().unwrap_or_else(|| usage_error(&format!("{} needs a value", argv[i])))
    };
    let port_of = |i: usize, argv: &[String]| -> u16 {
        let v = need(i, argv);
        v.parse().unwrap_or_else(|_| usage_error(&format!("{} {v}: not a port number", argv[i])))
    };
    while i < argv.len() {
        match argv[i].as_str() {
            "--feed" => {
                a.beast = match need(i, &argv).as_str() {
                    "beast" => true,
                    "sbs" => false,
                    other => usage_error(&format!("--feed {other}: expected beast or sbs")),
                };
                i += 2;
            }
            "--host" => { a.host = need(i, &argv); i += 2; }
            "--beast-port" => { beast_port = port_of(i, &argv); i += 2; }
            "--sbs-port" => { sbs_port = port_of(i, &argv); i += 2; }
            "--http-port" => { port = port_of(i, &argv); i += 2; }
            "--bind" => { bind = need(i, &argv); i += 2; }
            "--cert" => { a.cert = Some(need(i, &argv).into()); i += 2; }
            "--key" => { a.key = Some(need(i, &argv).into()); i += 2; }
            "--root" => { a.root = need(i, &argv).into(); i += 2; }
            "--logos" => { a.logos = Some(need(i, &argv).into()); i += 2; }
            "--cache" => { a.cache = need(i, &argv).into(); i += 2; }
            "--no-hexdb" => { a.hexdb = false; i += 1; }
            "-h" | "--help" => { print!("{USAGE}"); std::process::exit(0); }
            other => usage_error(&format!("unknown argument {other}")),
        }
    }
    a.feed_port = if a.beast { beast_port } else { sbs_port };
    a.http = format!("{bind}:{port}").to_socket_addrs().ok()
        .and_then(|mut it| it.next())
        .unwrap_or_else(|| { eprintln!("cannot use {bind}:{port} as an address"); std::process::exit(2) });
    a
}

/// Where the cache belongs when nobody says otherwise.
fn cache_home() -> PathBuf {
    std::env::var_os("XDG_CACHE_HOME")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("HOME").map(|h| PathBuf::from(h).join(".cache")))
        .unwrap_or_else(|| PathBuf::from("."))
        .join("anrb-map")
}

/// The page, and the feed behind it.
struct Bridge {
    state: Arc<State>,
    root: PathBuf,
    logos: Option<PathBuf>,
    /// Registration, type, operator, route and photograph, fetched from
    /// hexdb.io by the bridge and cached, so viewers' browsers never contact
    /// hexdb.io.
    hexdb: Option<Hexdb>,
}

impl Handler for Bridge {
    fn get(&self, path: &str, query: &str) -> Response {
        if path == "/updates" {
            let json = self.state.updates(param(query, "since"));
            return Response::gzipped("application/json", gzip(json.as_bytes()));
        }
        if let Some(code) = path.strip_prefix("/logo/") {
            return self.logo(code);
        }
        if let Some(rest) = path.strip_prefix("/api/") {
            return self.api(rest);
        }
        if let Some(icao) = path.strip_prefix("/photo/") {
            return self.photo(icao);
        }
        // One directory, by file name only: a path from the network never
        // reaches anywhere but the directory the page was served from.
        let name = match path {
            "/" | "/index.html" => "index.html",
            p => match Path::new(p).file_name().and_then(|n| n.to_str()) {
                Some(n) => n,
                None => return Response::status(404, "not found"),
            },
        };
        match std::fs::read(self.root.join(name)) {
            Ok(body) => Response::ok(content_type(name), body),
            Err(_) => Response::status(404, "not found"),
        }
    }

    fn websocket(&self, query: &str, mut ws: WebSocket) {
        self.push(query, &mut ws);
    }
}

impl Bridge {
    /// A hexdb answer, from the cache or from hexdb.io. The argument is
    /// checked by [`Hexdb`] against the shape its kind takes rather than
    /// escaped: an address is six hex digits, a callsign up to 12 letters and
    /// digits, and an airport code three or four, so nothing else reaches the
    /// network or the cache. Any other argument is answered as not found.
    fn api(&self, rest: &str) -> Response {
        let db = match &self.hexdb {
            Some(db) => db,
            None => return Response::status(404, "lookups disabled"),
        };
        let (kind, arg) = match rest.split_once('/') {
            Some(p) => p,
            None => return Response::status(404, "not found"),
        };
        let found = match kind {
            "aircraft" => db.aircraft(arg),
            "route" => db.route(arg),
            "airport" => db.airport(arg),
            _ => return Response::status(404, "not found"),
        };
        match found {
            Lookup::Found(json) => Response::ok("application/json", json.into_bytes()),
            Lookup::Missing => Response::status(404, "not found"),
            Lookup::Unavailable => Response::status(503, "hexdb unavailable"),
        }
    }

    /// The photograph of an airframe. Kept on disk here, so a browser that
    /// asks twice costs nothing and hexdb is asked once.
    fn photo(&self, icao: &str) -> Response {
        let db = match &self.hexdb {
            Some(db) => db,
            None => return Response::status(404, "lookups disabled"),
        };
        match db.photo(icao) {
            Some(jpeg) => Response::cached("image/jpeg", jpeg, 86_400),
            None => Response::status(404, "not found"),
        }
    }

    /// An airline's logo, by its ICAO code. The code is checked rather than
    /// sanitised: one to five letters or digits name a file in the logo
    /// directory and nothing else can, whatever the request asks for. A logo
    /// never changes, so it is worth the browser keeping.
    fn logo(&self, code: &str) -> Response {
        let dir = match &self.logos {
            Some(d) => d,
            None => return Response::status(404, "logos disabled"),
        };
        if !is_word(code, 5) {
            return Response::status(404, "not found");
        }
        // The vendor's own directory is inconsistent about the extension -
        // DLH.bmp beside BAW.BMP - so both are tried.
        let code = code.to_ascii_uppercase();
        for ext in ["bmp", "BMP"] {
            if let Ok(body) = std::fs::read(dir.join(format!("{code}.{ext}"))) {
                return Response::cached("image/bmp", body, 86_400);
            }
        }
        Response::status(404, "not found")
    }

    fn push(&self, query: &str, ws: &mut WebSocket) {
        // A reconnecting page passes the last sequence number it received;
        // only newer points are sent.
        let mut since = param(query, "since");
        let mut next = Instant::now();
        loop {
            if Instant::now() >= next {
                let msg = self.state.updates(since);
                since = seq_of(&msg).unwrap_or(since);
                if ws.send_text(&msg).is_err() {
                    return;
                }
                next = Instant::now() + Duration::from_secs(1);
            }
            if matches!(ws.poll(Duration::from_millis(250)), Ok(Some(Message::Close)) | Err(_)) {
                return;                  // `WebSocket`'s Drop sends the close frame
            }
        }
    }
}

/// Letters and digits, up to a length: an airline code.
fn is_word(s: &str, max: usize) -> bool {
    !s.is_empty() && s.len() <= max && s.bytes().all(|b| b.is_ascii_alphanumeric())
}

fn content_type(name: &str) -> &'static str {
    if name.ends_with(".html") { "text/html; charset=utf-8" }
    else if name.ends_with(".js") { "text/javascript; charset=utf-8" }
    else if name.ends_with(".json") { "application/json" }
    else { "application/octet-stream" }
}

/// An unsigned query parameter, or 0 when it is absent or malformed.
fn param(query: &str, name: &str) -> u64 {
    query.split('&')
        .filter_map(|kv| kv.split_once('='))
        .find(|(k, _)| *k == name)
        .and_then(|(_, v)| v.parse().ok())
        .unwrap_or(0)
}

/// The sequence number the page should ask from next time, read back out of
/// the message rather than held separately.
fn seq_of(msg: &str) -> Option<u64> {
    let at = msg.find("\"seq\":")? + 6;
    msg[at..].split(|c: char| !c.is_ascii_digit()).next()?.parse().ok()
}

/// One kind of feed, as [`follow`] reads it.
trait Feed {
    /// A new connection to the receiver is open.
    fn connected(&mut self);

    /// Bytes from the feed. An empty slice means a read timed out with
    /// nothing, so periodic work still runs while the feed is quiet.
    fn read(&mut self, bytes: &[u8]);
}

/// Read the feed at `host:port`, reconnecting for as long as the process runs.
fn follow(state: &State, host: &str, port: u16, feed: &mut impl Feed) {
    let mut buf = [0u8; 65536];
    loop {
        if let Ok(mut sock) = TcpStream::connect((host, port)) {
            let _ = sock.set_read_timeout(Some(Duration::from_secs(1)));
            state.set_connected(true);
            feed.connected();
            loop {
                let n = match sock.read(&mut buf) {
                    Ok(0) => break,
                    Ok(n) => n,
                    Err(e) if e.kind() == ErrorKind::WouldBlock || e.kind() == ErrorKind::TimedOut => 0,
                    Err(_) => break,
                };
                feed.read(&buf[..n]);
            }
        }
        // Nothing listening, or the feed went away: the sleep below
        // spaces out the retries rather than spinning on connect.
        state.set_connected(false);
        std::thread::sleep(Duration::from_secs(2));
    }
}

/// The Beast feed, decoded by the driver's tracker. The tracker outlives a
/// reconnect, so aircraft keep their confirmed positions.
struct Beast {
    state: Arc<State>,
    tracker: Tracker,
    reader: beast::Reader,
    start: Instant,
    messages: u64,
    next_sync: Instant,
}

impl Beast {
    fn new(state: Arc<State>) -> Beast {
        Beast {
            state,
            tracker: Tracker::new(),
            reader: beast::Reader::new(),
            start: Instant::now(),
            messages: 0,
            next_sync: Instant::now(),
        }
    }
}

impl Feed for Beast {
    fn connected(&mut self) {
        self.next_sync = Instant::now();
    }

    fn read(&mut self, bytes: &[u8]) {
        let ms = self.start.elapsed().as_millis() as u32;
        let mut msgs = Vec::new();
        self.reader.feed(bytes, |m| msgs.push(*m));
        for m in &msgs {
            self.messages += 1;
            let icao = m.frame().get(1..4)
                .map_or(0, |b| u32::from_be_bytes([0, b[0], b[1], b[2]]));
            let before = self.tracker.table.get(&icao).map(|a| (a.lat, a.lon, a.t_pos));
            if let Some((icao, Update::AirbornePosition | Update::SurfacePosition)) =
                self.tracker.update_raw(m.frame(), ms)
            {
                let a = &self.tracker.table[&icao];
                if let (Some(lat), Some(lon)) = (a.lat, a.lon) {
                    if before != Some((a.lat, a.lon, a.t_pos)) {
                        self.state.point(icao, lat, lon, a.alt, map::now());
                    }
                }
            }
        }
        if Instant::now() >= self.next_sync {
            let now = map::now();
            self.tracker.expire(ms, 2 * map::INACTIVE as u32 * 1000);
            for a in self.tracker.table.values() {
                self.state.fields(a, ms.wrapping_sub(a.t_seen) as f64 / 1000.0, now);
            }
            self.state.counters(self.messages, self.tracker.bad_parity, self.tracker.implausible,
                                self.reader.resyncs);
            self.state.expire(now);
            self.next_sync = Instant::now() + Duration::from_millis(500);
        }
    }
}

/// The BaseStation feed. Its lines are already decoded, so the tracker has
/// nothing to do here.
struct Sbs {
    state: Arc<State>,
    seen: HashMap<u32, Aircraft>,
    messages: u64,
    /// The start of a line whose end has not arrived yet.
    pending: Vec<u8>,
}

impl Sbs {
    fn new(state: Arc<State>) -> Sbs {
        Sbs { state, seen: HashMap::new(), messages: 0, pending: Vec::new() }
    }
}

impl Feed for Sbs {
    fn connected(&mut self) {
        self.pending.clear();
    }

    fn read(&mut self, bytes: &[u8]) {
        self.pending.extend_from_slice(bytes);
        while let Some(at) = self.pending.iter().position(|&b| b == b'\n') {
            let line = String::from_utf8_lossy(&self.pending[..at]).trim().to_string();
            self.pending.drain(..=at);
            self.messages += 1;
            let now = map::now();
            if let Some((icao, pos)) = sbs_line(&line, &mut self.seen) {
                if let Some((lat, lon)) = pos {
                    self.state.point(icao, lat, lon, self.seen[&icao].alt, now);
                }
                self.state.fields(&self.seen[&icao], 0.0, now);
            }
            self.state.counters(self.messages, 0, 0, 0);
        }
        self.state.expire(map::now());
    }
}

/// Fold one BaseStation line into what is known, returning the address and a
/// position if the line carried one. The 22 fields are the format's own.
fn sbs_line(line: &str, seen: &mut HashMap<u32, Aircraft>) -> Option<(u32, Option<(f64, f64)>)> {
    let f: Vec<&str> = line.split(',').collect();
    if f.len() < 22 || f[0] != "MSG" {
        return None;
    }
    let icao = u32::from_str_radix(f[4].trim(), 16).ok()?;
    let a = seen.entry(icao).or_insert_with(|| Aircraft::new(icao));
    let text = |i: usize| -> Option<&str> { f.get(i).map(|s| s.trim()).filter(|s| !s.is_empty()) };
    if let Some(cs) = text(10) { a.callsign = Some(cs.to_string()); }
    if let Some(v) = text(11).and_then(|s| s.parse().ok()) { a.alt = Some(v); }
    if let Some(v) = text(12).and_then(|s| s.parse().ok()) { a.speed = Some(v); }
    if let Some(v) = text(13).and_then(|s| s.parse().ok()) { a.heading = Some(v); }
    if let Some(v) = text(16).and_then(|s| s.parse().ok()) { a.vrate = Some(v); }
    if let Some(v) = text(17).and_then(|s| s.parse().ok()) { a.squawk = Some(v); }
    if let Some(v) = text(21) { a.on_ground = v == "-1" || v == "1"; }
    let pos = match (text(14).and_then(|s| s.parse::<f64>().ok()),
                     text(15).and_then(|s| s.parse::<f64>().ok())) {
        (Some(lat), Some(lon)) if lat != 0.0 || lon != 0.0 => {
            a.lat = Some(lat);
            a.lon = Some(lon);
            Some((lat, lon))
        }
        _ => None,
    };
    Some((icao, pos))
}

fn main() -> std::io::Result<()> {
    let a = args();

    // With lookups off nothing is opened, so the cache directory is not even
    // created: the bridge talks to the receiver and to whoever asks for the
    // page, and to nothing else.
    let hexdb = if a.hexdb {
        match Hexdb::open(&a.cache, Duration::from_secs(10 * 24 * 3600)) {
            Ok(db) => { println!("hexdb cache in {}", a.cache.display()); Some(db) }
            Err(e) => {
                eprintln!("cannot open hexdb cache at {}: {e}; hexdb lookups disabled", a.cache.display());
                None
            }
        }
    } else {
        println!("hexdb lookups disabled");
        None
    };

    let mut state = State::new(if a.beast { "beast" } else { "sbs" });
    if a.logos.is_some() {
        state.serving_logos();
    }
    if hexdb.is_some() {
        state.serving_lookups();
    }
    let state = Arc::new(state);

    let (s, host, port, beast) = (Arc::clone(&state), a.host.clone(), a.feed_port, a.beast);
    std::thread::spawn(move || {
        if beast {
            follow(&s, &host, port, &mut Beast::new(Arc::clone(&s)));
        } else {
            follow(&s, &host, port, &mut Sbs::new(Arc::clone(&s)));
        }
    });

    let tls = match (&a.cert, &a.key) {
        (Some(c), Some(k)) => Some((c.as_path(), k.as_path())),
        (None, None) => None,
        _ => { eprintln!("--cert and --key go together"); std::process::exit(2); }
    };
    let server = Server::bind(a.http, tls)?;
    println!("{} {}:{}  ->  {}://{}   (poll /updates, stream /ws)",
             if a.beast { "Beast" } else { "SBS" }, a.host, a.feed_port,
             if server.is_tls() { "https" } else { "http" }, server.local_addr());
    server.serve(Arc::new(Bridge { state, root: a.root, logos: a.logos, hexdb }))
}
