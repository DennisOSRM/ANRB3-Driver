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
use std::ffi::OsString;
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
use anrb_map::stats;

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
  --site LAT,LON      the receiver's position in decimal degrees, which the
                      range statistics are measured from; ANRB_SITE if not
                      given
  --no-hexdb          answer no lookups and make no outbound request at all;
                      nothing is read from or written to the cache, and the
                      page is told not to ask
  -h, --help          show this help
";

/// How long hexdb answers are kept.
const HEXDB_TTL: Duration = Duration::from_secs(10 * 24 * 3600);

/// Print `msg` and the usage text, and exit with status 2.
fn usage_error(msg: &str) -> ! {
    eprint!("{msg}\n{USAGE}");
    std::process::exit(2)
}

#[derive(Debug)]
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
    site: Option<(f64, f64)>,
}

/// The outcome of reading the command line.
#[derive(Debug)]
enum Command {
    Run(Box<Args>),
    Help,
}

/// Why the command line was refused.
#[derive(Debug, PartialEq)]
enum ArgError {
    /// Shown above the usage text.
    Usage(String),
    /// The listening address does not resolve; shown on its own.
    Address(String),
}

/// Read the arguments that follow the program name. `cache` is the cache
/// directory to use when `--cache` is not given, and `env_site` the value of
/// `ANRB_SITE`, used when `--site` is not given.
fn parse_args(
    argv: impl IntoIterator<Item = String>,
    cache: PathBuf,
    env_site: Option<String>,
) -> Result<Command, ArgError> {
    let mut a = Args {
        beast: true,
        host: "127.0.0.1".into(),
        feed_port: 0,
        http: ([0, 0, 0, 0], 8080).into(),
        cert: None,
        key: None,
        root: "web".into(),
        logos: None,
        cache,
        hexdb: true,
        site: None,
    };
    let (mut beast_port, mut sbs_port, mut bind, mut port) =
        (30005u16, 30003u16, "0.0.0.0".to_string(), 8080u16);
    let mut argv = argv.into_iter();
    while let Some(flag) = argv.next() {
        let mut need = || {
            argv.next()
                .ok_or_else(|| ArgError::Usage(format!("{flag} needs a value")))
        };
        let port_of = |v: String| {
            v.parse()
                .map_err(|_| ArgError::Usage(format!("{flag} {v}: not a port number")))
        };
        match flag.as_str() {
            "--feed" => {
                a.beast = match need()?.as_str() {
                    "beast" => true,
                    "sbs" => false,
                    other => {
                        return Err(ArgError::Usage(format!(
                            "--feed {other}: expected beast or sbs"
                        )))
                    }
                };
            }
            "--host" => a.host = need()?,
            "--beast-port" => beast_port = port_of(need()?)?,
            "--sbs-port" => sbs_port = port_of(need()?)?,
            "--http-port" => port = port_of(need()?)?,
            "--bind" => bind = need()?,
            "--cert" => a.cert = Some(need()?.into()),
            "--key" => a.key = Some(need()?.into()),
            "--root" => a.root = need()?.into(),
            "--logos" => a.logos = Some(need()?.into()),
            "--cache" => a.cache = need()?.into(),
            "--no-hexdb" => a.hexdb = false,
            "--site" => {
                a.site = Some(
                    stats::parse_site(&need()?)
                        .map_err(|e| ArgError::Usage(format!("--site {e}")))?,
                )
            }
            "-h" | "--help" => return Ok(Command::Help),
            other => return Err(ArgError::Usage(format!("unknown argument {other}"))),
        }
    }
    // An empty ANRB_SITE is the same as none: Compose passes the variable
    // through whether or not it is set.
    if let Some(v) = env_site.filter(|v| !v.trim().is_empty()) {
        if a.site.is_none() {
            a.site =
                Some(stats::parse_site(&v).map_err(|e| ArgError::Usage(format!("ANRB_SITE {e}")))?);
        }
    }
    a.feed_port = if a.beast { beast_port } else { sbs_port };
    a.http = format!("{bind}:{port}")
        .to_socket_addrs()
        .ok()
        .and_then(|mut it| it.next())
        .ok_or_else(|| ArgError::Address(format!("cannot use {bind}:{port} as an address")))?;
    Ok(Command::Run(Box::new(a)))
}

/// Where the cache belongs when nobody says otherwise.
fn cache_home() -> PathBuf {
    cache_home_from(std::env::var_os("XDG_CACHE_HOME"), std::env::var_os("HOME"))
}

/// [`cache_home`], given `XDG_CACHE_HOME` and `HOME`.
fn cache_home_from(xdg: Option<OsString>, home: Option<OsString>) -> PathBuf {
    xdg.map(PathBuf::from)
        .or_else(|| home.map(|h| PathBuf::from(h).join(".cache")))
        .unwrap_or_else(|| PathBuf::from("."))
        .join("anrb-map")
}

/// The certificate and key to serve HTTPS with, or None for plain HTTP. One
/// without the other is an error.
fn tls_files(a: &Args) -> Result<Option<(&Path, &Path)>, &'static str> {
    match (&a.cert, &a.key) {
        (Some(c), Some(k)) => Ok(Some((c.as_path(), k.as_path()))),
        (None, None) => Ok(None),
        _ => Err("--cert and --key go together"),
    }
}

/// The lookup cache, or None when lookups are off or the cache cannot be
/// opened. Says which on stdout or stderr.
///
/// With lookups off nothing is opened, so the cache directory is not even
/// created: the bridge talks to the receiver and to whoever asks for the
/// page, and to nothing else.
fn open_hexdb(a: &Args) -> Option<Hexdb> {
    if !a.hexdb {
        println!("hexdb lookups disabled");
        return None;
    }
    match Hexdb::open(&a.cache, HEXDB_TTL) {
        Ok(db) => {
            println!("hexdb cache in {}", a.cache.display());
            Some(db)
        }
        Err(e) => {
            eprintln!(
                "cannot open hexdb cache at {}: {e}; hexdb lookups disabled",
                a.cache.display()
            );
            None
        }
    }
}

/// The map state for this feed, telling the page what the bridge serves.
fn new_state(a: &Args, lookups: bool) -> State {
    let mut state = State::new(if a.beast { "beast" } else { "sbs" });
    if a.logos.is_some() {
        state.serving_logos();
    }
    if lookups {
        state.serving_lookups();
    }
    if let Some(site) = a.site {
        state.set_site(site);
    }
    state
}

/// Keep the statistics in the cache directory, so that a restart does not
/// lose the last day: read them now, and write them every ten minutes. The
/// file is written beside its final name and renamed into place, so it is
/// never half written. Not used without the cache, as with `--no-hexdb`.
fn keep_stats(state: &Arc<State>, dir: &Path) {
    let file = dir.join("stats.txt");
    if let Ok(text) = std::fs::read_to_string(&file) {
        state.load_stats(&text);
    }
    let state = Arc::clone(state);
    std::thread::spawn(move || loop {
        std::thread::sleep(Duration::from_secs(600));
        let tmp = file.with_extension("txt.new");
        if let Err(e) =
            std::fs::write(&tmp, state.save_stats()).and_then(|()| std::fs::rename(&tmp, &file))
        {
            eprintln!("cannot save statistics to {}: {e}", file.display());
        }
    });
}

/// The line printed once the server is listening.
fn banner(a: &Args, server: &Server) -> String {
    format!(
        "{} {}:{}  ->  {}://{}   (poll /updates, stream /ws)",
        if a.beast { "Beast" } else { "SBS" },
        a.host,
        a.feed_port,
        if server.is_tls() { "https" } else { "http" },
        server.local_addr()
    )
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
    /// How often an open WebSocket is sent the updates.
    push_every: Duration,
}

impl Handler for Bridge {
    fn get(&self, path: &str, query: &str) -> Response {
        if path == "/updates" {
            let json = self.state.updates(param(query, "since"));
            return Response::gzipped("application/json", gzip(json.as_bytes()));
        }
        if path == "/stats" {
            let json = self.state.stats_json();
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
                next = Instant::now() + self.push_every;
            }
            if matches!(
                ws.poll(Duration::from_millis(250)),
                Ok(Some(Message::Close)) | Err(_)
            ) {
                return; // `WebSocket`'s Drop sends the close frame
            }
        }
    }
}

/// Letters and digits, up to a length: an airline code.
fn is_word(s: &str, max: usize) -> bool {
    !s.is_empty() && s.len() <= max && s.bytes().all(|b| b.is_ascii_alphanumeric())
}

fn content_type(name: &str) -> &'static str {
    if name.ends_with(".html") {
        "text/html; charset=utf-8"
    } else if name.ends_with(".js") {
        "text/javascript; charset=utf-8"
    } else if name.ends_with(".json") {
        "application/json"
    } else {
        "application/octet-stream"
    }
}

/// An unsigned query parameter, or 0 when it is absent or malformed.
fn param(query: &str, name: &str) -> u64 {
    query
        .split('&')
        .filter_map(|kv| kv.split_once('='))
        .find(|(k, _)| *k == name)
        .and_then(|(_, v)| v.parse().ok())
        .unwrap_or(0)
}

/// The sequence number the page should ask from next time, read back out of
/// the message rather than held separately.
fn seq_of(msg: &str) -> Option<u64> {
    let at = msg.find("\"seq\":")? + 6;
    msg[at..]
        .split(|c: char| !c.is_ascii_digit())
        .next()?
        .parse()
        .ok()
}

/// One kind of feed, as [`follow`] reads it.
trait Feed {
    /// A new connection to the receiver is open.
    fn connected(&mut self);

    /// Bytes from the feed. An empty slice means a read timed out with
    /// nothing, so periodic work still runs while the feed is quiet.
    fn read(&mut self, bytes: &[u8]);
}

/// How [`follow`] paces itself.
struct Timing {
    /// The longest a read waits before the feed is handed an empty slice.
    read: Duration,
    /// The pause after a connection fails or ends, before the next attempt.
    retry: Duration,
}

/// The pacing the bridge runs with.
const TIMING: Timing = Timing {
    read: Duration::from_secs(1),
    retry: Duration::from_secs(2),
};

/// Read the feed at `host:port`, reconnecting for as long as `go_on` returns
/// true. It is asked before each attempt to connect.
fn follow(
    state: &State,
    host: &str,
    port: u16,
    feed: &mut impl Feed,
    timing: &Timing,
    mut go_on: impl FnMut() -> bool,
) {
    let mut buf = [0u8; 65536];
    while go_on() {
        if let Ok(mut sock) = TcpStream::connect((host, port)) {
            let _ = sock.set_read_timeout(Some(timing.read));
            state.set_connected(true);
            feed.connected();
            loop {
                let n = match sock.read(&mut buf) {
                    Ok(0) => break,
                    Ok(n) => n,
                    Err(e)
                        if e.kind() == ErrorKind::WouldBlock || e.kind() == ErrorKind::TimedOut =>
                    {
                        0
                    }
                    Err(_) => break,
                };
                feed.read(&buf[..n]);
            }
        }
        // Nothing listening, or the feed went away: the sleep below
        // spaces out the retries rather than spinning on connect.
        state.set_connected(false);
        std::thread::sleep(timing.retry);
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
        // A message cut off when the last connection ended is lost. A new
        // reader keeps its bytes out of the first message on this one; the
        // count of resyncs carries on.
        let resyncs = self.reader.resyncs;
        self.reader = beast::Reader::new();
        self.reader.resyncs = resyncs;
        self.next_sync = Instant::now();
    }

    fn read(&mut self, bytes: &[u8]) {
        let ms = self.start.elapsed().as_millis() as u32;
        let mut msgs = Vec::new();
        self.reader.feed(bytes, |m| msgs.push(*m));
        for m in &msgs {
            self.messages += 1;
            let icao = m
                .frame()
                .get(1..4)
                .map_or(0, |b| u32::from_be_bytes([0, b[0], b[1], b[2]]));
            let before = self
                .tracker
                .table
                .get(&icao)
                .map(|a| (a.lat, a.lon, a.t_pos));
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
                self.state
                    .fields(a, ms.wrapping_sub(a.t_seen) as f64 / 1000.0, now);
            }
            self.state.counters(
                self.messages,
                self.tracker.bad_parity,
                self.tracker.implausible,
                self.reader.resyncs,
            );
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
        Sbs {
            state,
            seen: HashMap::new(),
            messages: 0,
            pending: Vec::new(),
        }
    }
}

impl Feed for Sbs {
    fn connected(&mut self) {
        self.pending.clear();
    }

    fn read(&mut self, bytes: &[u8]) {
        self.pending.extend_from_slice(bytes);
        while let Some(at) = self.pending.iter().position(|&b| b == b'\n') {
            let line = String::from_utf8_lossy(&self.pending[..at])
                .trim()
                .to_string();
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
    if let Some(cs) = text(10) {
        a.callsign = Some(cs.to_string());
    }
    if let Some(v) = text(11).and_then(|s| s.parse().ok()) {
        a.alt = Some(v);
    }
    if let Some(v) = text(12).and_then(|s| s.parse().ok()) {
        a.speed = Some(v);
    }
    if let Some(v) = text(13).and_then(|s| s.parse().ok()) {
        a.heading = Some(v);
    }
    if let Some(v) = text(16).and_then(|s| s.parse().ok()) {
        a.vrate = Some(v);
    }
    if let Some(v) = text(17).and_then(|s| s.parse().ok()) {
        a.squawk = Some(v);
    }
    if let Some(v) = text(21) {
        a.on_ground = v == "-1" || v == "1";
    }
    let pos = match (
        text(14).and_then(|s| s.parse::<f64>().ok()),
        text(15).and_then(|s| s.parse::<f64>().ok()),
    ) {
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
    let a = match parse_args(
        std::env::args().skip(1),
        cache_home(),
        std::env::var("ANRB_SITE").ok(),
    ) {
        Ok(Command::Run(a)) => *a,
        Ok(Command::Help) => {
            print!("{USAGE}");
            std::process::exit(0)
        }
        Err(ArgError::Usage(msg)) => usage_error(&msg),
        Err(ArgError::Address(msg)) => {
            eprintln!("{msg}");
            std::process::exit(2)
        }
    };

    let hexdb = open_hexdb(&a);
    let state = Arc::new(new_state(&a, hexdb.is_some()));
    if hexdb.is_some() {
        keep_stats(&state, &a.cache);
    }

    let (s, host, port, beast) = (Arc::clone(&state), a.host.clone(), a.feed_port, a.beast);
    std::thread::spawn(move || {
        if beast {
            follow(
                &s,
                &host,
                port,
                &mut Beast::new(Arc::clone(&s)),
                &TIMING,
                || true,
            );
        } else {
            follow(
                &s,
                &host,
                port,
                &mut Sbs::new(Arc::clone(&s)),
                &TIMING,
                || true,
            );
        }
    });

    let tls = tls_files(&a).unwrap_or_else(|msg| {
        eprintln!("{msg}");
        std::process::exit(2)
    });
    let server = Server::bind(a.http, tls)?;
    println!("{}", banner(&a, &server));
    server.serve(Arc::new(Bridge {
        state,
        root: a.root,
        logos: a.logos,
        hexdb,
        push_every: Duration::from_secs(1),
    }))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::{BufRead, BufReader, Write};
    use std::net::TcpListener;
    use std::sync::atomic::{AtomicUsize, Ordering};

    // An even and an odd airborne position of 4009DA, from the tracker's
    // tests. Even, odd and even again publish 50.33265 N, 8.73717 E.
    const EVEN: &str = "8d4009da5833318e2bd82af8c6f5";
    const ODD: &str = "8d4009da583324fef1cbc5c7449d";

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
            .collect()
    }

    /// Frames in Beast form, as the receiver sends them.
    fn beast_stream(frames: &[&str]) -> Vec<u8> {
        let mut out = Vec::new();
        for f in frames {
            assert!(beast::encode(&hex(f), &mut out));
        }
        out
    }

    /// A directory under the system's temporary directory, removed on drop.
    struct TempDir(PathBuf);

    impl TempDir {
        fn new() -> TempDir {
            static N: AtomicUsize = AtomicUsize::new(0);
            let n = N.fetch_add(1, Ordering::Relaxed);
            let p = std::env::temp_dir().join(format!("anrb-map-main-{}-{n}", std::process::id()));
            let _ = std::fs::remove_dir_all(&p);
            std::fs::create_dir_all(&p).unwrap();
            TempDir(p)
        }
    }

    impl Drop for TempDir {
        fn drop(&mut self) {
            let _ = std::fs::remove_dir_all(&self.0);
        }
    }

    fn gunzip(b: &[u8]) -> String {
        let mut s = String::new();
        flate2::read::GzDecoder::new(b)
            .read_to_string(&mut s)
            .unwrap();
        s
    }

    fn text(r: &Response) -> String {
        String::from_utf8_lossy(&r.body).into_owned()
    }

    // ---- the command line --------------------------------------------------

    fn args(a: &[&str]) -> Result<Command, ArgError> {
        parse_args(
            a.iter().map(|s| s.to_string()),
            PathBuf::from("/tmp/c"),
            None,
        )
    }

    fn run_args(a: &[&str]) -> Args {
        match args(a) {
            Ok(Command::Run(a)) => *a,
            other => panic!("{a:?}: {other:?}"),
        }
    }

    fn usage_err(a: &[&str]) -> String {
        match args(a) {
            Err(ArgError::Usage(m)) => m,
            other => panic!("{a:?}: {other:?}"),
        }
    }

    /// The site comes from --site, or else from ANRB_SITE; an empty
    /// ANRB_SITE is none, and a bad value of either is refused.
    #[test]
    fn the_site_comes_from_the_option_or_the_environment() {
        let with_env = |a: &[&str], env: &str| match parse_args(
            a.iter().map(|s| s.to_string()),
            PathBuf::from("/tmp/c"),
            Some(env.to_string()),
        ) {
            Ok(Command::Run(a)) => Ok(a.site),
            Err(ArgError::Usage(m)) => Err(m),
            other => panic!("{a:?} {env:?}: {other:?}"),
        };
        assert_eq!(run_args(&[]).site, None);
        assert_eq!(
            run_args(&["--site", "50.05,8.57"]).site,
            Some((50.05, 8.57))
        );
        assert_eq!(with_env(&[], "49.9, 8.6"), Ok(Some((49.9, 8.6))));
        assert_eq!(
            with_env(&["--site", "50,8"], "49,9"),
            Ok(Some((50.0, 8.0))),
            "the option wins"
        );
        assert_eq!(with_env(&[], " "), Ok(None), "empty is none");
        assert!(usage_err(&["--site", "north"]).starts_with("--site north: expected LAT,LON"));
        assert!(with_env(&[], "x")
            .unwrap_err()
            .starts_with("ANRB_SITE x: expected"));
    }

    #[test]
    fn no_arguments_give_the_defaults() {
        let a = run_args(&[]);
        assert!(a.beast);
        assert_eq!(a.host, "127.0.0.1");
        assert_eq!(a.feed_port, 30005);
        assert_eq!(a.http, "0.0.0.0:8080".parse().unwrap());
        assert_eq!((a.cert, a.key), (None, None));
        assert_eq!(a.root, PathBuf::from("web"));
        assert_eq!(a.logos, None);
        assert_eq!(a.cache, PathBuf::from("/tmp/c"));
        assert!(a.hexdb);
    }

    #[test]
    fn the_feed_picks_which_port_is_read() {
        assert_eq!(run_args(&["--feed", "sbs"]).feed_port, 30003);
        let a = run_args(&[
            "--sbs-port",
            "4003",
            "--beast-port",
            "4005",
            "--feed",
            "sbs",
        ]);
        assert!(!a.beast);
        assert_eq!(a.feed_port, 4003);
        let a = run_args(&[
            "--sbs-port",
            "4003",
            "--beast-port",
            "4005",
            "--feed",
            "beast",
        ]);
        assert!(a.beast);
        assert_eq!(a.feed_port, 4005);
    }

    #[test]
    fn every_flag_is_read() {
        let a = run_args(&[
            "--host",
            "radar.local",
            "--http-port",
            "0",
            "--bind",
            "127.0.0.1",
            "--cert",
            "c.pem",
            "--key",
            "k.pem",
            "--root",
            "/srv/web",
            "--logos",
            "/srv/logos",
            "--cache",
            "/var/cache/x",
            "--no-hexdb",
        ]);
        assert_eq!(a.host, "radar.local");
        assert_eq!(a.http, "127.0.0.1:0".parse().unwrap());
        assert_eq!(a.cert, Some(PathBuf::from("c.pem")));
        assert_eq!(a.key, Some(PathBuf::from("k.pem")));
        assert_eq!(a.root, PathBuf::from("/srv/web"));
        assert_eq!(a.logos, Some(PathBuf::from("/srv/logos")));
        assert_eq!(a.cache, PathBuf::from("/var/cache/x"));
        assert!(!a.hexdb);
    }

    #[test]
    fn help_is_asked_for_with_either_spelling() {
        assert!(matches!(args(&["-h"]), Ok(Command::Help)));
        assert!(matches!(
            args(&["--no-hexdb", "--help", "--bogus"]),
            Ok(Command::Help)
        ));
    }

    #[test]
    fn bad_arguments_say_what_is_wrong() {
        assert_eq!(usage_err(&["--feed"]), "--feed needs a value");
        assert_eq!(
            usage_err(&["--feed", "raw"]),
            "--feed raw: expected beast or sbs"
        );
        assert_eq!(usage_err(&["--host"]), "--host needs a value");
        assert_eq!(
            usage_err(&["--http-port", "http"]),
            "--http-port http: not a port number"
        );
        assert_eq!(
            usage_err(&["--sbs-port", "70000"]),
            "--sbs-port 70000: not a port number"
        );
        assert_eq!(
            usage_err(&["--beast-port", "-1"]),
            "--beast-port -1: not a port number"
        );
        assert_eq!(usage_err(&["--verbose"]), "unknown argument --verbose");
    }

    #[test]
    fn an_address_that_does_not_resolve_is_refused_without_the_usage_text() {
        match args(&["--bind", ""]) {
            Err(ArgError::Address(m)) => assert_eq!(m, "cannot use :8080 as an address"),
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn the_cache_goes_under_the_xdg_directory_then_home() {
        let s = |v: &str| Some(OsString::from(v));
        assert_eq!(
            cache_home_from(s("/xdg"), s("/home/u")),
            PathBuf::from("/xdg/anrb-map")
        );
        assert_eq!(
            cache_home_from(None, s("/home/u")),
            PathBuf::from("/home/u/.cache/anrb-map")
        );
        assert_eq!(cache_home_from(None, None), PathBuf::from("./anrb-map"));
        assert!(cache_home().ends_with("anrb-map"));
    }

    #[test]
    fn a_certificate_needs_its_key() {
        let a = run_args(&[]);
        assert_eq!(tls_files(&a), Ok(None));
        let a = run_args(&["--cert", "c.pem", "--key", "k.pem"]);
        assert_eq!(
            tls_files(&a),
            Ok(Some((Path::new("c.pem"), Path::new("k.pem"))))
        );
        let msg = Err("--cert and --key go together");
        assert_eq!(tls_files(&run_args(&["--cert", "c.pem"])), msg);
        assert_eq!(tls_files(&run_args(&["--key", "k.pem"])), msg);
    }

    #[test]
    fn the_cache_is_opened_only_when_lookups_are_on() {
        let dir = TempDir::new();
        let cache = dir.0.join("cache");
        let cache_arg = cache.to_str().unwrap();

        assert!(open_hexdb(&run_args(&["--cache", cache_arg, "--no-hexdb"])).is_none());
        assert!(
            !cache.exists(),
            "with lookups off the cache is not even created"
        );

        assert!(open_hexdb(&run_args(&["--cache", cache_arg])).is_some());
        assert!(cache.join("entries.log").exists());

        // A file where the directory should be cannot hold a cache.
        let file = dir.0.join("file");
        std::fs::write(&file, b"x").unwrap();
        assert!(open_hexdb(&run_args(&["--cache", file.to_str().unwrap()])).is_none());
    }

    #[test]
    fn the_state_tells_the_page_what_is_served() {
        let json = new_state(&run_args(&[]), false).updates(0);
        assert!(
            json.contains("\"source\":\"beast\",\"logos\":false,\"lookups\":false"),
            "{json}"
        );
        let a = run_args(&["--feed", "sbs", "--logos", "/l"]);
        let json = new_state(&a, true).updates(0);
        assert!(
            json.contains("\"source\":\"sbs\",\"logos\":true,\"lookups\":true"),
            "{json}"
        );
    }

    #[test]
    fn the_banner_names_the_feed_and_the_address() {
        let server = Server::bind("127.0.0.1:0".parse().unwrap(), None).unwrap();
        let port = server.local_addr().port();
        assert_eq!(
            banner(&run_args(&[]), &server),
            format!(
                "Beast 127.0.0.1:30005  ->  http://127.0.0.1:{port}   (poll /updates, stream /ws)"
            )
        );
        assert_eq!(
            banner(&run_args(&["--feed", "sbs", "--host", "rx"]), &server),
            format!("SBS rx:30003  ->  http://127.0.0.1:{port}   (poll /updates, stream /ws)")
        );
    }

    // ---- small helpers -----------------------------------------------------

    #[test]
    fn a_query_parameter_is_a_number_or_zero() {
        assert_eq!(param("since=42", "since"), 42);
        assert_eq!(param("a=1&since=7&b=2", "since"), 7);
        assert_eq!(param("", "since"), 0);
        assert_eq!(param("since", "since"), 0);
        assert_eq!(param("since=-3", "since"), 0);
        assert_eq!(param("since=x", "since"), 0);
        assert_eq!(param("sincere=5", "since"), 0);
    }

    #[test]
    fn the_sequence_number_is_read_back_out_of_the_message() {
        assert_eq!(
            seq_of("{\"now\":1.0,\"seq\":1234,\"connected\":true}"),
            Some(1234)
        );
        assert_eq!(seq_of("{\"seq\":0}"), Some(0));
        assert_eq!(seq_of("{\"now\":1.0}"), None);
        assert_eq!(seq_of("{\"seq\":null}"), None);
        let s = State::new("beast");
        s.point(1, 1.0, 2.0, None, map::now());
        assert_eq!(seq_of(&s.updates(0)), Some(1));
    }

    #[test]
    fn a_word_is_letters_and_digits_up_to_a_length() {
        assert!(is_word("DLH", 5));
        assert!(is_word("a1b2c", 5));
        assert!(!is_word("", 5));
        assert!(!is_word("ABCDEF", 5));
        assert!(!is_word("../x", 5));
        assert!(!is_word("A.B", 5));
    }

    #[test]
    fn content_types_follow_the_extension() {
        assert_eq!(content_type("index.html"), "text/html; charset=utf-8");
        assert_eq!(content_type("tracks.js"), "text/javascript; charset=utf-8");
        assert_eq!(content_type("airports.json"), "application/json");
        assert_eq!(content_type("icon.png"), "application/octet-stream");
    }

    // ---- the handler -------------------------------------------------------

    fn bridge(root: &Path, logos: Option<PathBuf>, hexdb: Option<Hexdb>) -> Bridge {
        Bridge {
            state: Arc::new(State::new("beast")),
            root: root.to_path_buf(),
            logos,
            hexdb,
            push_every: Duration::from_millis(20),
        }
    }

    fn not_found(r: &Response, why: &str) {
        assert_eq!(r.status, 404);
        assert_eq!(text(r), format!("{why}\n"));
        assert_eq!(r.content_type, "text/plain; charset=utf-8");
    }

    #[test]
    fn page_files_are_served_by_name_from_the_root_only() {
        let dir = TempDir::new();
        std::fs::write(dir.0.join("index.html"), b"<html>").unwrap();
        std::fs::write(dir.0.join("tracks.js"), b"let x;").unwrap();
        std::fs::write(dir.0.join("data.json"), b"{}").unwrap();
        let b = bridge(&dir.0, None, None);

        for path in ["/", "/index.html", "/deep/dir/index.html"] {
            let r = b.get(path, "");
            assert_eq!((r.status, text(&r).as_str()), (200, "<html>"), "{path}");
            assert_eq!(r.content_type, "text/html; charset=utf-8");
            assert_eq!(r.max_age, 0);
            assert!(!r.gzip);
        }
        let r = b.get("/tracks.js", "");
        assert_eq!(
            (r.status, r.content_type.as_str()),
            (200, "text/javascript; charset=utf-8")
        );
        let r = b.get("/data.json", "");
        assert_eq!(
            (r.status, r.content_type.as_str()),
            (200, "application/json")
        );

        not_found(&b.get("/missing.css", ""), "not found");
        not_found(&b.get("/..", ""), "not found");
        not_found(&b.get("/../../etc/passwd", ""), "not found");
    }

    /// The statistics are served, measured from the site.
    #[test]
    fn the_statistics_are_served() {
        let dir = TempDir::new();
        let mut b = bridge(&dir.0, None, None);
        Arc::get_mut(&mut b.state).unwrap().set_site((50.0, 8.0));
        b.state.point(0x4009DA, 51.0, 8.0, None, map::now()); // 60 NM north
        b.state.counters(10, 0, 0, 0);
        let r = b.get("/stats", "");
        assert_eq!(
            (r.status, r.content_type.as_str(), r.gzip),
            (200, "application/json", true)
        );
        let json = gunzip(&r.body);
        assert!(
            json.starts_with("{\"site\":[50.00000,8.00000],\"sector\":5,\"range\":[60,0,"),
            "{json}"
        );
        // Saved and read back into a fresh state, the range is still there.
        let saved = b.state.save_stats();
        let mut c = State::new("beast");
        c.set_site((50.0, 8.0));
        c.load_stats(&saved);
        assert_eq!(c.stats_json(), b.state.stats_json());
    }

    #[test]
    fn updates_are_gzipped_json_from_the_sequence_number_asked_for() {
        let dir = TempDir::new();
        let b = bridge(&dir.0, None, None);
        let now = map::now();
        b.state.point(0x4009DA, 50.5, 8.5, Some(37000), now - 5.0);
        b.state.point(0x4009DA, 50.6, 8.6, Some(37000), now);

        let r = b.get("/updates", "");
        assert_eq!(
            (r.status, r.content_type.as_str(), r.gzip),
            (200, "application/json", true)
        );
        let json = gunzip(&r.body);
        assert!(json.starts_with("{\"now\":"), "{json}");
        assert!(json.contains("\"seq\":2,"), "{json}");
        assert!(json.contains("\"4009DA\":{"), "{json}");
        assert_eq!(json.matches(",37000]").count(), 2, "{json}");

        let json = gunzip(&b.get("/updates", "since=1").body);
        assert_eq!(
            json.matches(",37000]").count(),
            1,
            "only the newer point: {json}"
        );
        assert!(json.contains(",8.6,50.6,37000]"), "{json}");
    }

    #[test]
    fn logos_are_off_unless_a_directory_is_given() {
        let dir = TempDir::new();
        not_found(
            &bridge(&dir.0, None, None).get("/logo/DLH", ""),
            "logos disabled",
        );
    }

    #[test]
    fn a_logo_is_found_with_either_extension_and_kept_by_the_browser() {
        let dir = TempDir::new();
        let logos = dir.0.join("logos");
        std::fs::create_dir(&logos).unwrap();
        std::fs::write(logos.join("DLH.bmp"), b"BM lower").unwrap();
        std::fs::write(logos.join("BAW.BMP"), b"BM upper").unwrap();
        std::fs::write(dir.0.join("SECRET.bmp"), b"outside").unwrap();
        let b = bridge(&dir.0, Some(logos), None);

        for (path, body) in [
            ("/logo/DLH", "BM lower"),
            ("/logo/dlh", "BM lower"),
            ("/logo/BAW", "BM upper"),
        ] {
            let r = b.get(path, "");
            assert_eq!((r.status, text(&r).as_str()), (200, body), "{path}");
            assert_eq!((r.content_type.as_str(), r.max_age), ("image/bmp", 86_400));
        }
        not_found(&b.get("/logo/AFR", ""), "not found");
        // Codes that are not one to five letters and digits never reach the
        // file system.
        for code in ["", "TOOLONG", "..", "..%2FSECRET", "A-B"] {
            not_found(&b.get(&format!("/logo/{code}"), ""), "not found");
        }
        not_found(&b.get("/logo/../SECRET", ""), "not found");
    }

    /// A cache already holding answers, so no lookup below reaches hexdb.io.
    /// Every key asked for in these tests is either here or has a shape that
    /// hexdb cannot have.
    fn filled_cache(dir: &Path) -> Hexdb {
        let now = map::now() as u64;
        std::fs::create_dir_all(dir.join("images")).unwrap();
        std::fs::write(dir.join("images").join("4009DA.jpg"), b"\xff\xd8 a photo").unwrap();
        let log = format!(
            "a 4009DA {now} + {{\"Registration\":\"D-AIXX\"}}\n\
             a 3C6DD0 {now} -\n\
             r DLH400 {now} + {{\"route\":\"EDDF-KJFK\"}}\n\
             r DLH401 {now} -\n\
             f EDDF {now} + {{\"airport\":\"Frankfurt\"}}\n\
             p 4009DA {now} + 4009DA.jpg\n\
             p 3C6DD0 {now} -\n"
        );
        std::fs::write(dir.join("entries.log"), log).unwrap();
        Hexdb::open(dir, HEXDB_TTL).unwrap()
    }

    #[test]
    fn lookups_are_answered_from_the_cache() {
        let dir = TempDir::new();
        let b = bridge(&dir.0, None, Some(filled_cache(&dir.0.join("cache"))));

        for (path, body) in [
            ("/api/aircraft/4009DA", "{\"Registration\":\"D-AIXX\"}"),
            ("/api/aircraft/4009da", "{\"Registration\":\"D-AIXX\"}"),
            ("/api/route/DLH400", "{\"route\":\"EDDF-KJFK\"}"),
            ("/api/airport/EDDF", "{\"airport\":\"Frankfurt\"}"),
        ] {
            let r = b.get(path, "");
            assert_eq!((r.status, text(&r).as_str()), (200, body), "{path}");
            assert_eq!(r.content_type, "application/json");
        }
        // Remembered misses, keys hexdb cannot have, and paths that name no
        // lookup.
        for path in [
            "/api/aircraft/3C6DD0",
            "/api/route/DLH401",
            "/api/aircraft/XYZ",
            "/api/route/DLH-400",
            "/api/airport/EDDFX",
            "/api/aircraft",
            "/api/ship/4009DA",
            "/api/",
        ] {
            not_found(&b.get(path, ""), "not found");
        }
    }

    #[test]
    fn photos_are_answered_from_the_cache_and_kept_by_the_browser() {
        let dir = TempDir::new();
        let b = bridge(&dir.0, None, Some(filled_cache(&dir.0.join("cache"))));
        let r = b.get("/photo/4009DA", "");
        assert_eq!(
            (r.status, r.content_type.as_str(), r.max_age),
            (200, "image/jpeg", 86_400)
        );
        assert_eq!(r.body, b"\xff\xd8 a photo");
        not_found(&b.get("/photo/3C6DD0", ""), "not found");
        not_found(&b.get("/photo/nothex", ""), "not found");
    }

    #[test]
    fn with_lookups_off_nothing_is_looked_up() {
        let dir = TempDir::new();
        let b = bridge(&dir.0, None, None);
        not_found(&b.get("/api/aircraft/4009DA", ""), "lookups disabled");
        not_found(&b.get("/api/nonsense", ""), "lookups disabled");
        not_found(&b.get("/photo/4009DA", ""), "lookups disabled");
    }

    // ---- the WebSocket -----------------------------------------------------

    fn send_frame(wr: &mut impl Write, opcode: u8, payload: &[u8]) {
        let mask = [0x12, 0x34, 0x56, 0x78];
        let mut out = vec![0x80 | opcode, 0x80 | payload.len() as u8];
        out.extend_from_slice(&mask);
        out.extend(payload.iter().enumerate().map(|(i, b)| b ^ mask[i % 4]));
        wr.write_all(&out).unwrap();
    }

    fn recv_frame(rd: &mut impl Read) -> (u8, Vec<u8>) {
        let mut h = [0u8; 2];
        rd.read_exact(&mut h).unwrap();
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

    fn recv_text(rd: &mut impl Read) -> String {
        let (op, body) = recv_frame(rd);
        assert_eq!(op, 0x1, "a text frame");
        String::from_utf8(body).unwrap()
    }

    /// Serve `handler` on a port of its own and open a WebSocket to it.
    fn open_ws(handler: Arc<Bridge>, query: &str) -> (BufReader<TcpStream>, TcpStream) {
        let server = Server::bind("127.0.0.1:0".parse().unwrap(), None).unwrap();
        let addr = server.local_addr();
        std::thread::spawn(move || server.serve(handler));
        let s = TcpStream::connect(addr).unwrap();
        s.set_read_timeout(Some(Duration::from_secs(5))).unwrap();
        let mut rd = BufReader::new(s.try_clone().unwrap());
        (&s).write_all(format!("GET /ws?{query} HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\n\
                                Connection: Upgrade\r\nSec-WebSocket-Key: AAAAAAAAAAAAAAAAAAAAAA==\r\n\r\n")
                           .as_bytes()).unwrap();
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
        (rd, s)
    }

    #[test]
    fn the_websocket_pushes_only_what_is_new_and_closes_when_asked() {
        let dir = TempDir::new();
        let b = bridge(&dir.0, None, None);
        let state = Arc::clone(&b.state);
        let now = map::now();
        state.point(0x4009DA, 50.5, 8.5, None, now - 10.0);
        state.point(0x4009DA, 50.6, 8.6, None, now - 5.0);

        // A page that has seen the first point is sent the second only.
        let (mut rd, s) = open_ws(Arc::new(b), "since=1");
        let first = recv_text(&mut rd);
        assert_eq!(seq_of(&first), Some(2));
        assert!(!first.contains(",8.5,50.5,"), "{first}");
        assert!(first.contains(",8.6,50.6,null]]"), "{first}");

        // Pushes carry on, each from where the one before ended.
        let next = recv_text(&mut rd);
        assert!(next.contains("\"new\":[]"), "{next}");
        state.point(0x4009DA, 50.7, 8.7, None, now);
        let deadline = Instant::now() + Duration::from_secs(5);
        let with_point = loop {
            let m = recv_text(&mut rd);
            if seq_of(&m) == Some(3) {
                break m;
            }
            assert!(Instant::now() < deadline, "the new point was never pushed");
        };
        assert!(with_point.contains(",8.7,50.7,null]]"), "{with_point}");

        send_frame(&mut &s, 0x8, b"");
        // Whatever was already on its way, then the close.
        loop {
            let (op, _) = recv_frame(&mut rd);
            if op == 0x8 {
                break;
            }
            assert_eq!(op, 0x1);
        }
    }

    #[test]
    fn the_websocket_push_ends_when_the_page_goes_away() {
        let dir = TempDir::new();
        let handler = Arc::new(bridge(&dir.0, None, None));
        let weak = Arc::downgrade(&handler);
        let (rd, s) = open_ws(handler, "");
        drop(rd);
        drop(s);
        // The accept loop holds one reference and the connection thread
        // another, which goes when the push ends.
        let deadline = Instant::now() + Duration::from_secs(5);
        while weak.strong_count() > 1 {
            assert!(Instant::now() < deadline, "the push did not end");
            std::thread::sleep(Duration::from_millis(5));
        }
    }

    // ---- BaseStation lines -------------------------------------------------

    /// A 22-field MSG line for 4009DA with the given fields filled in.
    fn sbs(fields: &[(usize, &str)]) -> String {
        let mut f = vec![""; 22];
        f[0] = "MSG";
        f[1] = "3";
        f[4] = "4009DA";
        for &(i, v) in fields {
            f[i] = v;
        }
        f.join(",")
    }

    #[test]
    fn a_basestation_line_fills_in_every_field_it_carries() {
        let mut seen = HashMap::new();
        let line = sbs(&[
            (10, "DLH400  "),
            (11, "37000"),
            (12, "451.5"),
            (13, "87.2"),
            (14, "50.1"),
            (15, "8.2"),
            (16, "-640"),
            (17, "7700"),
            (21, "-1"),
        ]);
        assert_eq!(
            sbs_line(&line, &mut seen),
            Some((0x4009DA, Some((50.1, 8.2))))
        );
        let a = &seen[&0x4009DA];
        assert_eq!(a.callsign.as_deref(), Some("DLH400"));
        assert_eq!(a.alt, Some(37000));
        assert_eq!(a.speed, Some(451.5));
        assert_eq!(a.heading, Some(87.2));
        assert_eq!(a.vrate, Some(-640));
        assert_eq!(a.squawk, Some(7700));
        assert!(a.on_ground);
        assert_eq!((a.lat, a.lon), (Some(50.1), Some(8.2)));
    }

    #[test]
    fn a_basestation_line_keeps_what_it_does_not_carry() {
        let mut seen = HashMap::new();
        sbs_line(&sbs(&[(10, "DLH400"), (11, "37000"), (21, "1")]), &mut seen).unwrap();
        // Empty, unparsable and zero fields change nothing, except the
        // ground flag, which any value sets.
        let line = sbs(&[(11, "high"), (14, "0"), (15, "0"), (21, "0")]);
        assert_eq!(sbs_line(&line, &mut seen), Some((0x4009DA, None)));
        let a = &seen[&0x4009DA];
        assert_eq!(a.callsign.as_deref(), Some("DLH400"));
        assert_eq!(a.alt, Some(37000));
        assert!(!a.on_ground);
        assert_eq!(a.lat, None);
        // A latitude without a longitude is no position.
        assert_eq!(
            sbs_line(&sbs(&[(14, "50.1")]), &mut seen),
            Some((0x4009DA, None))
        );
        // Zero on one axis only is a real place.
        assert_eq!(
            sbs_line(&sbs(&[(14, "0"), (15, "8.2")]), &mut seen),
            Some((0x4009DA, Some((0.0, 8.2))))
        );
    }

    #[test]
    fn lines_that_are_not_messages_are_skipped() {
        let mut seen = HashMap::new();
        let short = sbs(&[]).rsplit_once(',').unwrap().0.to_string();
        assert_eq!(short.split(',').count(), 21);
        assert_eq!(sbs_line(&short, &mut seen), None);
        assert_eq!(sbs_line(&sbs(&[(0, "SEL")]), &mut seen), None);
        assert_eq!(sbs_line(&sbs(&[(4, "XYZ123")]), &mut seen), None);
        assert_eq!(sbs_line("", &mut seen), None);
        assert!(seen.is_empty());
    }

    #[test]
    fn the_sbs_feed_joins_lines_split_across_reads() {
        let state = Arc::new(State::new("sbs"));
        let mut feed = Sbs::new(Arc::clone(&state));
        feed.connected();
        let line = format!("{}\r\n", sbs(&[(10, "DLH400"), (14, "50.1"), (15, "8.2")]));
        let (a, b) = line.split_at(20);
        feed.read(a.as_bytes());
        assert!(
            !state.updates(0).contains("4009DA"),
            "half a line is not a message"
        );
        feed.read(b.as_bytes());
        feed.read(b"a line that is not a message\n");
        let json = state.updates(0);
        assert!(json.contains("\"messages\":2,"), "{json}");
        assert!(json.contains("\"4009DA\":{\"cs\":\"DLH400\""), "{json}");
        assert!(json.contains(",8.2,50.1,null]]"), "{json}");
    }

    // ---- Beast -------------------------------------------------------------

    #[test]
    fn the_beast_feed_publishes_a_confirmed_position() {
        let state = Arc::new(State::new("beast"));
        let mut feed = Beast::new(Arc::clone(&state));
        feed.connected();
        feed.read(&beast_stream(&[EVEN, ODD]));
        let json = state.updates(0);
        assert!(
            json.contains("\"4009DA\":{"),
            "the aircraft is known: {json}"
        );
        assert!(
            json.contains("\"new\":[]"),
            "one pair is not yet a position: {json}"
        );
        assert!(json.contains("\"messages\":2,"), "{json}");

        // The counters and fields are synced at most twice a second; the
        // point goes out at once.
        feed.read(&beast_stream(&[EVEN]));
        let json = state.updates(0);
        assert!(json.contains(",8.73") && json.contains(",50.33"), "{json}");
        assert_eq!(seq_of(&json), Some(1));

        // The odd message decodes on its own grid, a few tens of metres
        // away, and is a new point.
        feed.read(&beast_stream(&[ODD]));
        assert_eq!(seq_of(&state.updates(0)), Some(2));
    }

    #[test]
    fn the_beast_feed_counts_bad_parity_and_resyncs() {
        let state = Arc::new(State::new("beast"));
        let mut feed = Beast::new(Arc::clone(&state));
        let mut bad = hex(EVEN);
        bad[5] ^= 0x01;
        let mut bytes = Vec::new();
        beast::encode(&bad, &mut bytes);
        // A message cut short by the start of the next one.
        let cut = beast_stream(&[ODD]);
        bytes.extend_from_slice(&cut[..10]);
        bytes.extend(beast_stream(&[ODD]));
        feed.connected();
        feed.read(&bytes);
        let json = state.updates(0);
        assert!(json.contains("\"messages\":2,"), "{json}");
        assert!(json.contains("\"bad_parity\":1,"), "{json}");
        assert!(json.contains("\"resyncs\":1}"), "{json}");

        // The counts outlive a reconnect.
        feed.connected();
        feed.read(&[]);
        let json = state.updates(0);
        assert!(json.contains("\"bad_parity\":1,"), "{json}");
        assert!(json.contains("\"resyncs\":1}"), "{json}");
    }

    #[test]
    fn a_message_cut_off_by_a_hang_up_does_not_spoil_the_next_connection() {
        let state = Arc::new(State::new("beast"));
        let mut feed = Beast::new(Arc::clone(&state));
        feed.connected();
        // The connection drops just after the first 0x1a of a doubled pair.
        feed.read(&[0x1a, b'3', 0, 0, 0, 0, 0, 0x1a]);
        feed.connected();
        feed.read(&beast_stream(&[EVEN]));
        let json = state.updates(0);
        assert!(
            json.contains("\"4009DA\""),
            "the first message on the new connection is read: {json}"
        );
        assert!(json.contains("\"messages\":1,"), "{json}");
        assert!(
            json.contains("\"resyncs\":0}"),
            "a hang-up is not a resync: {json}"
        );

        // Cut off anywhere else, the same holds.
        feed.read(&beast_stream(&[ODD])[..12]);
        feed.connected();
        feed.next_sync = Instant::now();
        feed.read(&beast_stream(&[EVEN]));
        let json = state.updates(0);
        assert!(json.contains("\"messages\":2,"), "{json}");
        assert!(json.contains("\"resyncs\":0}"), "{json}");
    }

    #[test]
    fn a_quiet_beast_feed_still_expires_what_it_holds() {
        let state = Arc::new(State::new("beast"));
        let mut feed = Beast::new(Arc::clone(&state));
        feed.connected();
        feed.read(&beast_stream(&[EVEN]));
        assert!(state.updates(0).contains("\"4009DA\""));
        // An aircraft last heard two inactive periods ago is dropped by the
        // tracker on the next sync, even with no bytes arriving.
        feed.start -= Duration::from_secs(2 * map::INACTIVE as u64 + 1);
        feed.next_sync = Instant::now();
        feed.read(&[]);
        assert!(feed.tracker.table.is_empty());
    }

    // ---- following the feed ------------------------------------------------

    /// A feed that keeps what [`follow`] hands it.
    struct Recorder {
        state: Arc<State>,
        connects: usize,
        bytes: Vec<u8>,
        empty_reads: usize,
        /// Whether the state said connected at each read.
        up: Vec<bool>,
    }

    impl Feed for Recorder {
        fn connected(&mut self) {
            self.connects += 1;
        }

        fn read(&mut self, bytes: &[u8]) {
            if bytes.is_empty() {
                self.empty_reads += 1;
            }
            self.bytes.extend_from_slice(bytes);
            self.up
                .push(self.state.updates(0).contains("\"connected\":true"));
        }
    }

    const FAST: Timing = Timing {
        read: Duration::from_millis(10),
        retry: Duration::from_millis(1),
    };

    /// A listener that accepts one connection per entry of `sessions`,
    /// writes each of its pieces with a pause between them, and hangs up.
    fn receiver(
        sessions: Vec<Vec<Vec<u8>>>,
        pause: Duration,
    ) -> (u16, std::thread::JoinHandle<()>) {
        let l = TcpListener::bind("127.0.0.1:0").unwrap();
        let port = l.local_addr().unwrap().port();
        let t = std::thread::spawn(move || {
            for pieces in sessions {
                let (mut s, _) = l.accept().unwrap();
                for (i, p) in pieces.iter().enumerate() {
                    if i > 0 {
                        std::thread::sleep(pause);
                    }
                    s.write_all(p).unwrap();
                }
            }
        });
        (port, t)
    }

    /// A `go_on` that allows `n` attempts to connect.
    fn attempts(mut n: usize) -> impl FnMut() -> bool {
        move || {
            let more = n > 0;
            n = n.saturating_sub(1);
            more
        }
    }

    #[test]
    fn follow_reads_everything_and_reconnects_after_a_hang_up() {
        let (port, t) = receiver(
            vec![
                vec![b"one".to_vec(), b"two".to_vec()],
                vec![b"three".to_vec()],
            ],
            Duration::from_millis(50),
        );
        let state = Arc::new(State::new("beast"));
        let mut rec = Recorder {
            state: Arc::clone(&state),
            connects: 0,
            bytes: Vec::new(),
            empty_reads: 0,
            up: Vec::new(),
        };
        follow(&state, "127.0.0.1", port, &mut rec, &FAST, attempts(2));
        t.join().unwrap();

        assert_eq!(rec.connects, 2);
        assert_eq!(rec.bytes, b"onetwothree");
        assert!(
            rec.empty_reads >= 1,
            "a pause longer than the read timeout is an empty read"
        );
        assert!(rec.up.iter().all(|&u| u), "connected while reading");
        assert!(
            state.updates(0).contains("\"connected\":false"),
            "not connected once it ends"
        );
    }

    #[test]
    fn follow_retries_when_nothing_is_listening() {
        let port = TcpListener::bind("127.0.0.1:0")
            .unwrap()
            .local_addr()
            .unwrap()
            .port();
        let state = Arc::new(State::new("beast"));
        state.set_connected(true);
        let mut rec = Recorder {
            state: Arc::clone(&state),
            connects: 0,
            bytes: Vec::new(),
            empty_reads: 0,
            up: Vec::new(),
        };
        let mut asked = 0;
        follow(&state, "127.0.0.1", port, &mut rec, &FAST, || {
            asked += 1;
            asked <= 3
        });
        assert_eq!(
            asked, 4,
            "asked before each of three attempts, then told to stop"
        );
        assert_eq!(rec.connects, 0);
        assert!(state.updates(0).contains("\"connected\":false"));
    }

    #[test]
    fn a_beast_feed_keeps_its_tracker_across_a_reconnect() {
        let (port, t) = receiver(
            vec![
                vec![beast_stream(&[EVEN, ODD])],
                vec![beast_stream(&[EVEN])],
            ],
            Duration::ZERO,
        );
        let state = Arc::new(State::new("beast"));
        let mut feed = Beast::new(Arc::clone(&state));
        follow(&state, "127.0.0.1", port, &mut feed, &FAST, attempts(2));
        t.join().unwrap();
        // The pair came on the first connection and the message that
        // confirms it on the second.
        let json = state.updates(0);
        assert_eq!(seq_of(&json), Some(1), "{json}");
        assert!(json.contains(",8.73") && json.contains(",50.33"), "{json}");
        assert!(json.contains("\"messages\":3,"), "{json}");
    }

    #[test]
    fn an_sbs_feed_drops_a_line_cut_off_by_a_reconnect() {
        let first = format!(
            "{}\n{}",
            sbs(&[(10, "DLH400")]),
            &sbs(&[(11, "12000")])[..20]
        );
        let second = format!("{}\n", sbs(&[(14, "50.1"), (15, "8.2")]));
        let (port, t) = receiver(
            vec![vec![first.into_bytes()], vec![second.into_bytes()]],
            Duration::ZERO,
        );
        let state = Arc::new(State::new("sbs"));
        let mut feed = Sbs::new(Arc::clone(&state));
        follow(&state, "127.0.0.1", port, &mut feed, &FAST, attempts(2));
        t.join().unwrap();
        let json = state.updates(0);
        assert!(
            json.contains("\"messages\":2,"),
            "the cut line is not counted: {json}"
        );
        assert!(json.contains("\"cs\":\"DLH400\",\"alt\":null"), "{json}");
        assert!(json.contains(",8.2,50.1,null]]"), "{json}");
    }
}
