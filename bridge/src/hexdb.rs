//! What the aircraft never transmits, from hexdb.io: the registration and
//! type behind an address, today's route behind a callsign, an airport's name
//! behind its code, and a photograph of the airframe.
//!
//! Lookups are made here rather than in the browser: one request per airframe
//! however many viewers there are, answers kept across page loads, and the
//! viewer's browser never contacts hexdb.io. hexdb.io asks not to be scraped,
//! so the request says who it is and the answers are kept for days rather than
//! minutes - an airframe's registration does not change, and a route changes
//! at most daily.
//!
//! Everything here is blocking and called from the connection threads in
//! [`crate::http`], the same as the rest of the bridge. Two threads that want
//! the same key at the same moment make one request between them: the second
//! waits on a condition variable until the first has filed its answer. That is
//! a map keyed by the entry, not one lock around the whole cache, because a
//! fetch can sit in a read for ten seconds and a thread asking for something
//! already cached must not wait behind it.
//!
//! The cache is an append-only log of one line per entry, compacted once it
//! has more than 64 lines and more than twice as many lines as entries, next
//! to a directory of JPEGs. A line is
//! appended whole and a compaction lands by rename, so the file on disk is
//! always a set of complete lines plus, after a power cut, at most one torn
//! one - which is skipped on load along with anything else that does not
//! parse. Misses are lines too: an address hexdb has never heard of must not
//! be asked for again every time someone opens its popup.

use std::collections::HashMap;
use std::fs::{self, File, OpenOptions};
use std::io::{self, ErrorKind, Read, Write};
use std::net::{TcpStream, ToSocketAddrs};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Condvar, Mutex};
use std::time::Duration;

use crate::http::find;
use crate::lock;

/// The host every lookup goes to. The photograph URL returned by hexdb may
/// point to another host; it is the only request not sent to HOST.
const HOST: &str = "hexdb.io";

/// Who is asking. hexdb asks people not to scrape, so this names the software
/// and what it is for rather than pretending to be a browser. There is no URL
/// in it because this driver has no published home to point at.
const USER_AGENT: &str = "anrb-map/0.1 (map bridge for one AirNav RadarBox receiver)";

/// Connecting, and each read once connected. hexdb answers in well under a
/// second; ten seconds is for a link that has gone away without saying so.
const TIMEOUT: Duration = Duration::from_secs(10);

/// The most of a response body that is kept. The JSON answers are a few
/// hundred bytes and the thumbnails tens of kilobytes, so this only ever
/// catches something that has gone wrong.
const MAX_BODY: usize = 4 * 1024 * 1024;

/// The most a response head may be before it is treated as nonsense.
const HEAD_LIMIT: usize = 16 * 1024;

/// The log is rewritten once it holds this many lines and more than twice as
/// many as there are entries. Below that the wasted bytes are not worth the
/// rewrite.
const COMPACT_FLOOR: u64 = 64;

// ---- what a lookup says ----------------------------------------------------

/// The answer to one lookup: the JSON body, a 404 from hexdb (or a key hexdb
/// cannot have), or no answer at all because the request could not be made or
/// was refused.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Lookup {
    Found(String),
    Missing,
    Unavailable,
}

/// Which endpoint an entry came from. The letter goes in the cache key and in
/// the store line, so an airport and a callsign that happen to read the same
/// cannot overwrite one another.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    Aircraft,
    Route,
    Airport,
    Photo,
}

impl Kind {
    fn letter(self) -> char {
        match self {
            Kind::Aircraft => 'a',
            Kind::Route => 'r',
            Kind::Airport => 'f',
            Kind::Photo => 'p',
        }
    }

    fn from_letter(c: &str) -> Option<Kind> {
        match c {
            "a" => Some(Kind::Aircraft),
            "r" => Some(Kind::Route),
            "f" => Some(Kind::Airport),
            "p" => Some(Kind::Photo),
            _ => None,
        }
    }
}

/// One answer, kept. `body` is the JSON for a lookup and the image's file name
/// for a photograph; `None` is a miss, which is remembered exactly as a hit is.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Entry {
    /// When it was fetched, in seconds since the epoch.
    fetched: u64,
    body: Option<String>,
}

impl Entry {
    /// Whether it is old enough to be fetched again. A clock that has gone
    /// backwards leaves the entry fresh rather than expiring everything at
    /// once, which is what `saturating_sub` buys here.
    fn stale(&self, now: u64, ttl: Duration) -> bool {
        now.saturating_sub(self.fetched) >= ttl.as_secs()
    }
}

// ---- the cache -------------------------------------------------------------

/// The part of the cache that threads share: the entries, the log they are
/// appended to, and the counters.
struct Store {
    entries: HashMap<String, Entry>,
    /// The append handle, or None if the directory cannot be written to - the
    /// cache then works for as long as the process lives and forgets on exit,
    /// which is better than refusing to look anything up.
    log: Option<File>,
    lines: u64,
    hits: u64,
    fetched: u64,
    refused: u64,
    failed: u64,
}

/// One outstanding fetch, for the threads waiting on it.
struct Flight {
    done: Mutex<bool>,
    wake: Condvar,
}

pub struct Hexdb {
    dir: PathBuf,
    images: PathBuf,
    ttl: Duration,
    tls: Arc<rustls::ClientConfig>,
    store: Mutex<Store>,
    /// The keys being fetched right now. Held only while claiming or
    /// releasing one, never across a request.
    inflight: Mutex<HashMap<String, Arc<Flight>>>,
}

/// A claimed key, released when the fetch that claimed it is over. Everything
/// waiting on it is woken whether the fetch worked or not, so a failure costs
/// the other threads the fetch's time rather than for ever.
struct Claim<'a> {
    db: &'a Hexdb,
    key: String,
}

impl Drop for Claim<'_> {
    fn drop(&mut self) {
        let flight = lock(&self.db.inflight).remove(&self.key);
        if let Some(f) = flight {
            *lock(&f.done) = true;
            f.wake.notify_all();
        }
    }
}

/// Whole seconds since the epoch, from the same clock as [`crate::map::now`].
fn now_secs() -> u64 {
    crate::map::now() as u64
}

impl Hexdb {
    /// Open the cache under `dir`, creating it and the store if absent. `ttl`
    /// is how long an entry stays good.
    pub fn open(dir: &Path, ttl: Duration) -> io::Result<Hexdb> {
        let images = dir.join("images");
        fs::create_dir_all(&images)?;
        let path = dir.join("entries.log");
        let (entries, lines) = read_log(&path);
        let log = OpenOptions::new().create(true).append(true).open(&path)?;

        // The roots are the ones compiled in rather than the machine's, so the
        // bridge trusts the same certificates on a Raspberry Pi with an empty
        // /etc/ssl as it does on a desktop, and a receiver that is never
        // updated does not quietly stop being able to look anything up.
        let mut roots = rustls::RootCertStore::empty();
        roots.extend(webpki_roots::TLS_SERVER_ROOTS.iter().cloned());
        // ring for the same reason the server side names it: aws-lc-rs wants
        // cmake, which this tree does not.
        let provider = Arc::new(rustls::crypto::ring::default_provider());
        let tls = rustls::ClientConfig::builder_with_provider(provider)
            .with_safe_default_protocol_versions()
            .map_err(|e| io::Error::new(ErrorKind::InvalidData, e))?
            .with_root_certificates(roots)
            .with_no_client_auth();

        Ok(Hexdb {
            dir: dir.to_path_buf(),
            images,
            ttl,
            tls: Arc::new(tls),
            store: Mutex::new(Store {
                entries,
                log: Some(log),
                lines,
                hits: 0,
                fetched: 0,
                refused: 0,
                failed: 0,
            }),
            inflight: Mutex::new(HashMap::new()),
        })
    }

    /// `https://hexdb.io/api/v1/aircraft/<icao>` - registration, type, operator.
    pub fn aircraft(&self, icao: &str) -> Lookup {
        self.json(Kind::Aircraft, icao, "/api/v1/aircraft/")
    }

    /// `https://hexdb.io/api/v1/route/icao/<callsign>` - where it is going today.
    pub fn route(&self, callsign: &str) -> Lookup {
        self.json(Kind::Route, callsign, "/api/v1/route/icao/")
    }

    /// `https://hexdb.io/api/v1/airport/icao/<code>` - the field's name.
    pub fn airport(&self, code: &str) -> Lookup {
        self.json(Kind::Airport, code, "/api/v1/airport/icao/")
    }

    /// The photograph of an airframe as JPEG bytes.
    ///
    /// Two requests: hexdb's `hex-image-thumb` answers with the image's URL as
    /// one line of text, and the image itself is wherever that points. The
    /// bytes go in a file of their own under `images/`, so the log stays a log
    /// of short lines; an airframe with no photograph is remembered as having
    /// none, and is not asked about again until the entry expires.
    pub fn photo(&self, icao: &str) -> Option<Vec<u8>> {
        let key = normalise(Kind::Photo, icao)?;
        let ck = cache_key(Kind::Photo, &key);

        if let Some(e) = self.fresh(&ck) {
            self.count(|s| s.hits += 1);
            return self.image_of(&e);
        }
        let _claim = match self.claim(&ck) {
            Some(c) => c,
            // Another thread was already fetching it; whatever it filed is the
            // answer, and there is no sense in asking again.
            None => {
                self.count(|s| s.hits += 1);
                return self.cached(&ck).and_then(|e| self.image_of(&e));
            }
        };

        let url = match self.get(HOST, &format!("/hex-image-thumb?hex={key}")) {
            Ok((200, body)) => String::from_utf8_lossy(&body).trim().to_string(),
            Ok((404, _)) => {
                self.count(|s| s.fetched += 1);
                self.record(&ck, None);
                return None;
            }
            Ok(_) => return self.stale_image(&ck, false),
            Err(_) => return self.stale_image(&ck, true),
        };

        // Only https URLs are fetched; anything else is recorded as no
        // photograph.
        let Some((host, path)) = split_https(&url) else {
            self.count(|s| s.fetched += 1);
            self.record(&ck, None);
            return None;
        };

        match self.get(&host, &path) {
            Ok((200, jpeg)) if !jpeg.is_empty() => {
                let name = format!("{key}.jpg");
                if write_atomically(&self.images.join(&name), &jpeg).is_err() {
                    // The bytes are good but the disk is not: hand them over
                    // without filing them, so the popup is filled this once.
                    self.count(|s| s.failed += 1);
                    return Some(jpeg);
                }
                self.count(|s| s.fetched += 1);
                self.record(&ck, Some(name));
                Some(jpeg)
            }
            Ok((404, _)) => {
                self.count(|s| s.fetched += 1);
                self.record(&ck, None);
                None
            }
            Ok(_) => self.stale_image(&ck, false),
            Err(_) => self.stale_image(&ck, true),
        }
    }

    /// Lookups served from what was already on disk, lookups filled from
    /// hexdb, answers that were neither a body nor a 404 - a redirect, a 429,
    /// a 5xx - requests that never got an answer at all, and how many entries
    /// the store holds. An expired entry served because the refetch failed
    /// counts once as a failure and once as a lookup served from cache.
    #[cfg(test)]
    fn stats(&self) -> (u64, u64, u64, u64, usize) {
        let s = lock(&self.store);
        (s.hits, s.fetched, s.refused, s.failed, s.entries.len())
    }

    // ---- one lookup --------------------------------------------------------

    /// The three JSON endpoints, which differ only in their path and in what a
    /// key may look like.
    fn json(&self, kind: Kind, key: &str, prefix: &str) -> Lookup {
        // A key with anything but letters and digits in it is refused rather
        // than escaped: hexdb has no such key, and it has no business in a URL
        // path or in a store line either.
        let Some(key) = normalise(kind, key) else { return Lookup::Missing };
        let ck = cache_key(kind, &key);

        if let Some(e) = self.fresh(&ck) {
            self.count(|s| s.hits += 1);
            return answer(&e);
        }
        let _claim = match self.claim(&ck) {
            Some(c) => c,
            None => {
                self.count(|s| s.hits += 1);
                return self.cached(&ck).map_or(Lookup::Unavailable, |e| answer(&e));
            }
        };

        match self.get(HOST, &format!("{prefix}{key}")) {
            Ok((200, body)) => match String::from_utf8(body) {
                Ok(text) => {
                    self.count(|s| s.fetched += 1);
                    self.record(&ck, Some(text.clone()));
                    Lookup::Found(text)
                }
                // A body that is not UTF-8 is not JSON, so it is not filed.
                Err(_) => self.stale(&ck, true),
            },
            Ok((404, _)) => {
                self.count(|s| s.fetched += 1);
                self.record(&ck, None);
                Lookup::Missing
            }
            Ok(_) => self.stale(&ck, false),
            Err(_) => self.stale(&ck, true),
        }
    }

    /// What to say when a fetch came to nothing: the expired entry if there is
    /// one, rather than nothing at all. `transport` is true for a network
    /// failure and false for an unexpected status.
    fn stale(&self, ck: &str, transport: bool) -> Lookup {
        self.count(|s| if transport { s.failed += 1 } else { s.refused += 1 });
        match self.cached(ck) {
            Some(e) => {
                self.count(|s| s.hits += 1);
                answer(&e)
            }
            None => Lookup::Unavailable,
        }
    }

    /// [`Hexdb::stale`], for the photograph.
    fn stale_image(&self, ck: &str, transport: bool) -> Option<Vec<u8>> {
        self.count(|s| if transport { s.failed += 1 } else { s.refused += 1 });
        let e = self.cached(ck)?;
        self.count(|s| s.hits += 1);
        self.image_of(&e)
    }

    /// The bytes an entry points at, or None for an airframe with no
    /// photograph. A missing image file is treated as no photograph.
    fn image_of(&self, e: &Entry) -> Option<Vec<u8>> {
        let name = e.body.as_deref()?;
        // The name came out of the store, so it is checked before it is joined
        // on: a line someone has edited must not reach outside images/.
        if name.contains('/') || name.contains('\\') || name.contains("..") {
            return None;
        }
        fs::read(self.images.join(name)).ok()
    }

    // ---- the entries -------------------------------------------------------

    fn cached(&self, ck: &str) -> Option<Entry> {
        lock(&self.store).entries.get(ck).cloned()
    }

    /// The entry, if it is young enough to be served without asking again.
    fn fresh(&self, ck: &str) -> Option<Entry> {
        let now = now_secs();
        let s = lock(&self.store);
        s.entries.get(ck).filter(|e| !e.stale(now, self.ttl)).cloned()
    }

    /// File an answer, in memory and at the end of the log.
    ///
    /// The append happens under the same lock as the map so the two cannot
    /// disagree, which costs the microseconds of one short write to a file the
    /// kernel has already buffered. It is not flushed: losing the last line to
    /// a power cut costs one lookup.
    fn record(&self, ck: &str, body: Option<String>) {
        let entry = Entry { fetched: now_secs(), body };
        let line = format_line(ck, &entry);
        let mut s = lock(&self.store);
        s.entries.insert(ck.to_string(), entry);
        if let Some(log) = s.log.as_mut() {
            if log.write_all(line.as_bytes()).is_err() {
                // A disk that has filled up or gone away. Carry on in memory
                // rather than stop looking things up.
                s.log = None;
            } else {
                s.lines += 1;
            }
        }
        let entries = s.entries.len() as u64;
        if s.lines > COMPACT_FLOOR && s.lines > entries * 2 {
            self.compact(&mut s);
        }
    }

    /// Write the live entries to a new file and put it in the old one's place.
    /// The rename is the atomic part: until it returns, the log on disk is the
    /// old one, complete.
    fn compact(&self, s: &mut Store) {
        let path = self.dir.join("entries.log");
        let mut text = String::new();
        for (k, e) in &s.entries {
            text.push_str(&format_line(k, e));
        }
        if write_atomically(&path, text.as_bytes()).is_err() {
            return;
        }
        // The old handle still points at the file that was renamed over, so
        // appending to it would write into nothing.
        match OpenOptions::new().create(true).append(true).open(&path) {
            Ok(f) => {
                s.log = Some(f);
                s.lines = s.entries.len() as u64;
            }
            Err(_) => s.log = None,
        }
    }

    fn count(&self, f: impl FnOnce(&mut Store)) {
        f(&mut lock(&self.store));
    }

    /// Claim a key for fetching, or wait for whoever has already claimed it.
    ///
    /// `Some` means this thread is the one that asks. `None` means another
    /// thread has been and gone, and its answer is in the store - the caller
    /// reads it rather than asking again, so two threads wanting the same key
    /// at the same moment make exactly one request between them.
    fn claim(&self, ck: &str) -> Option<Claim<'_>> {
        let mut map = lock(&self.inflight);
        if let Some(f) = map.get(ck).cloned() {
            drop(map);
            let mut done = lock(&f.done);
            while !*done {
                done = f.wake.wait(done).unwrap_or_else(|e| e.into_inner());
            }
            return None;
        }
        map.insert(ck.to_string(), Arc::new(Flight { done: Mutex::new(false), wake: Condvar::new() }));
        Some(Claim { db: self, key: ck.to_string() })
    }

    // ---- the request -------------------------------------------------------

    /// One HTTPS GET, connection and all. Redirects are not followed; a 3xx
    /// counts as refused.
    fn get(&self, host: &str, path: &str) -> io::Result<(u16, Vec<u8>)> {
        let sock = connect(host)?;
        sock.set_read_timeout(Some(TIMEOUT))?;
        sock.set_write_timeout(Some(TIMEOUT))?;
        let name = rustls::pki_types::ServerName::try_from(host.to_string())
            .map_err(|e| io::Error::new(ErrorKind::InvalidInput, e))?;
        let conn = rustls::ClientConnection::new(Arc::clone(&self.tls), name)
            .map_err(io::Error::other)?;
        let mut tls = rustls::StreamOwned::new(conn, sock);

        // Connection: close because each lookup is one request minutes apart,
        // so a kept connection would only be a socket sitting idle at the far
        // end.
        let req = format!(
            "GET {path} HTTP/1.1\r\nHost: {host}\r\nUser-Agent: {USER_AGENT}\r\n\
             Accept: */*\r\nConnection: close\r\n\r\n"
        );
        tls.write_all(req.as_bytes())?;
        tls.flush()?;
        read_response(&mut tls)
    }

    // ---- for the tests -----------------------------------------------------

    #[cfg(test)]
    fn peek(&self, kind: Kind, key: &str) -> Option<Entry> {
        self.cached(&cache_key(kind, &normalise(kind, key)?))
    }

    #[cfg(test)]
    fn file(&self, kind: Kind, key: &str, body: Option<&str>) {
        let key = normalise(kind, key).expect("a key the tests meant to be valid");
        self.record(&cache_key(kind, &key), body.map(str::to_string));
    }
}

/// A cached entry as a [`Lookup`]. A remembered miss answers as a miss.
fn answer(e: &Entry) -> Lookup {
    match &e.body {
        Some(text) => Lookup::Found(text.clone()),
        None => Lookup::Missing,
    }
}

fn cache_key(kind: Kind, key: &str) -> String {
    format!("{}/{}", kind.letter(), key)
}

/// The key in the form hexdb uses, or None for a key hexdb cannot have.
///
/// hexdb's keys are upper-case letters and digits, and a length that is wrong
/// for the endpoint is a key it has never heard of, so rejecting one here
/// saves a request rather than losing an answer. The same check guards what
/// the page asks for and what is read back from the log.
fn normalise(kind: Kind, key: &str) -> Option<String> {
    let key = key.trim().to_ascii_uppercase();
    if key.is_empty() || !key.bytes().all(|b| b.is_ascii_alphanumeric()) {
        return None;
    }
    match kind {
        // An ICAO address is six hex digits and nothing else.
        Kind::Aircraft | Kind::Photo => {
            (key.len() == 6 && key.bytes().all(|b| b.is_ascii_hexdigit())).then_some(key)
        }
        // A callsign is up to eight characters; this allows up to 12
        // characters, for non-standard callsigns.
        Kind::Route => (key.len() <= 12).then_some(key),
        // IATA three, ICAO four.
        Kind::Airport => (3..=4).contains(&key.len()).then_some(key),
    }
}

// ---- the store on disk -----------------------------------------------------
//
// One line per entry: the kind's letter, the key, the fetch time in seconds
// since the epoch, a plus for an answer or a minus for a miss, and the body
// with its backslashes and line breaks escaped so a line is always a line.
// Reading stops at the first field that does not parse and carries on with the
// next line, which is what makes a torn tail after a power cut harmless.

fn format_line(ck: &str, e: &Entry) -> String {
    let (kind, key) = ck.split_once('/').unwrap_or(("?", ck));
    match &e.body {
        Some(body) => format!("{kind} {key} {} + {}\n", e.fetched, escape(body)),
        None => format!("{kind} {key} {} -\n", e.fetched),
    }
}

fn parse_line(line: &str) -> Option<(String, Entry)> {
    let mut f = line.splitn(5, ' ');
    let kind = Kind::from_letter(f.next()?)?;
    let key = normalise(kind, f.next()?)?;
    let fetched: u64 = f.next()?.parse().ok()?;
    let body = match f.next()? {
        "+" => Some(unescape(f.next()?)),
        "-" => None,
        _ => return None,
    };
    Some((cache_key(kind, &key), Entry { fetched, body }))
}

/// Everything the log holds, and how many lines it took to say it. Lines that
/// do not parse (a torn tail, or a line from another version) are skipped.
fn read_log(path: &Path) -> (HashMap<String, Entry>, u64) {
    let mut entries = HashMap::new();
    let mut lines = 0;
    let Ok(text) = fs::read_to_string(path) else { return (entries, lines) };
    for line in text.lines() {
        lines += 1;
        if let Some((k, e)) = parse_line(line) {
            // The last word wins: an entry appended later is a refetch of an
            // entry appended earlier.
            entries.insert(k, e);
        }
    }
    (entries, lines)
}

fn escape(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            _ => out.push(c),
        }
    }
    out
}

fn unescape(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut it = s.chars();
    while let Some(c) = it.next() {
        if c != '\\' {
            out.push(c);
            continue;
        }
        match it.next() {
            Some('n') => out.push('\n'),
            Some('r') => out.push('\r'),
            Some('\\') => out.push('\\'),
            // Anything else was not written by escape(); keep both characters
            // rather than lose one.
            Some(other) => {
                out.push('\\');
                out.push(other);
            }
            None => out.push('\\'),
        }
    }
    out
}

/// Write a file by writing another one and renaming it over the top, so a
/// reader only ever sees the whole of one version or the whole of the other.
/// On failure the temporary file is removed.
fn write_atomically(path: &Path, bytes: &[u8]) -> io::Result<()> {
    let tmp = path.with_extension("tmp");
    let written = File::create(&tmp).and_then(|mut f| {
        f.write_all(bytes)?;
        f.sync_all()
    });
    let result = written.and_then(|()| fs::rename(&tmp, path));
    if result.is_err() {
        let _ = fs::remove_file(&tmp);
    }
    result
}

// ---- HTTP over TLS ---------------------------------------------------------

/// The first address the name resolves to that will take a connection.
/// Resolution has no timeout of its own in the standard library; the connect
/// that follows it does.
fn connect(host: &str) -> io::Result<TcpStream> {
    let mut last = None;
    for addr in (host, 443u16).to_socket_addrs()? {
        match TcpStream::connect_timeout(&addr, TIMEOUT) {
            Ok(s) => {
                let _ = s.set_nodelay(true);
                return Ok(s);
            }
            Err(e) => last = Some(e),
        }
    }
    Err(last.unwrap_or_else(|| io::Error::new(ErrorKind::NotFound, "that name resolved to nothing")))
}

/// How a response says where its body ends.
#[derive(Debug, PartialEq, Eq)]
enum Framing {
    Length(usize),
    /// Cloudflare sits in front of hexdb and answers this way.
    Chunked,
    /// Neither header: the body is whatever arrives until the peer hangs up,
    /// which is what `Connection: close` amounts to.
    Eof,
}

/// The status line and the two headers that matter, from a complete head.
fn parse_head(head: &str) -> Option<(u16, Framing)> {
    let mut lines = head.split("\r\n");
    let status: u16 = lines.next()?.split(' ').nth(1)?.parse().ok()?;
    let mut framing = Framing::Eof;
    for line in lines {
        let Some((name, value)) = line.split_once(':') else { continue };
        let name = name.trim().to_ascii_lowercase();
        let value = value.trim();
        // Chunked wins where both are given, as RFC 9112 says it must:
        // a proxy that rewrote the body may have left the old length behind.
        if name == "transfer-encoding" && value.to_ascii_lowercase().contains("chunked") {
            framing = Framing::Chunked;
        } else if name == "content-length" && framing != Framing::Chunked {
            framing = Framing::Length(value.parse().ok()?);
        }
    }
    Some((status, framing))
}

/// A chunked body taken apart as the bytes arrive, so a chunk split across two
/// reads costs nothing but the wait for the rest of it.
struct Chunked {
    raw: Vec<u8>,
    out: Vec<u8>,
    done: bool,
}

impl Chunked {
    fn new() -> Chunked {
        Chunked { raw: Vec::new(), out: Vec::new(), done: false }
    }

    fn feed(&mut self, bytes: &[u8]) -> io::Result<()> {
        if self.done {
            // Trailers after the last chunk say nothing this needs.
            return Ok(());
        }
        self.raw.extend_from_slice(bytes);
        loop {
            let Some(eol) = find(&self.raw, b"\r\n") else {
                // A chunk header is a handful of hex digits and perhaps an
                // extension; this much without a line ending is not one.
                if self.raw.len() > 256 {
                    return Err(bad("a chunk header with no end to it"));
                }
                return Ok(());
            };
            let line = &self.raw[..eol];
            let digits = line.split(|&b| b == b';').next().unwrap_or(line);
            let text = std::str::from_utf8(digits).map_err(|_| bad("a chunk size that is not text"))?;
            let size = usize::from_str_radix(text.trim(), 16)
                .map_err(|_| bad("a chunk size that is not a number"))?;
            if size == 0 {
                self.done = true;
                self.raw.clear();
                return Ok(());
            }
            if self.out.len() + size > MAX_BODY {
                return Err(bad(format!("a response body past the {} MiB cap", MAX_BODY >> 20)));
            }
            // The header, its line ending, the chunk, and the line ending
            // after it.
            if self.raw.len() < eol + 2 + size + 2 {
                return Ok(());
            }
            self.out.extend_from_slice(&self.raw[eol + 2..eol + 2 + size]);
            self.raw.drain(..eol + 2 + size + 2);
        }
    }
}

fn bad(what: impl Into<String>) -> io::Error {
    io::Error::new(ErrorKind::InvalidData, what.into())
}

/// One response, head and body, from anything that reads bytes.
///
/// Written against `Read` rather than against the socket so the framing can be
/// exercised on byte slices, which is where the awkward cases are: a chunk
/// that arrives in two pieces, a length that arrives before its body, a peer
/// that hangs up without a close_notify.
fn read_response<R: Read>(rd: &mut R) -> io::Result<(u16, Vec<u8>)> {
    let mut buf = Vec::new();
    let mut chunk = [0u8; 16 * 1024];

    let head_end = loop {
        if let Some(at) = find(&buf, b"\r\n\r\n") {
            break at + 4;
        }
        if buf.len() > HEAD_LIMIT {
            return Err(bad(format!("a response head longer than {} KiB", HEAD_LIMIT >> 10)));
        }
        match fill(rd, &mut chunk)? {
            0 => return Err(bad("the connection closed before the response head")),
            n => buf.extend_from_slice(&chunk[..n]),
        }
    };

    let head = String::from_utf8_lossy(&buf[..head_end]).into_owned();
    let Some((status, framing)) = parse_head(&head) else {
        return Err(bad("a response head that is not HTTP"));
    };
    let rest: Vec<u8> = buf[head_end..].to_vec();

    let body = match framing {
        Framing::Length(want) => {
            if want > MAX_BODY {
                return Err(bad(format!("a Content-Length past the {} MiB cap", MAX_BODY >> 20)));
            }
            let mut body = rest;
            while body.len() < want {
                match fill(rd, &mut chunk)? {
                    0 => return Err(bad("the body stopped short of its Content-Length")),
                    n => body.extend_from_slice(&chunk[..n]),
                }
            }
            body.truncate(want);
            body
        }
        Framing::Chunked => {
            let mut dec = Chunked::new();
            dec.feed(&rest)?;
            while !dec.done {
                match fill(rd, &mut chunk)? {
                    0 => return Err(bad("the chunked body stopped before its last chunk")),
                    n => dec.feed(&chunk[..n])?,
                }
            }
            dec.out
        }
        Framing::Eof => {
            let mut body = rest;
            loop {
                match fill(rd, &mut chunk)? {
                    0 => break,
                    n => body.extend_from_slice(&chunk[..n]),
                }
                if body.len() > MAX_BODY {
                    return Err(bad(format!("a response body past the {} MiB cap", MAX_BODY >> 20)));
                }
            }
            body
        }
    };
    Ok((status, body))
}

/// One read, with the two things that are not really errors folded away: an
/// interrupted call, and a peer that closed the TCP connection without sending
/// a TLS close_notify first. The latter is what `Connection: close` usually
/// looks like in practice, and rustls reports it as an unexpected end of file.
fn fill<R: Read>(rd: &mut R, buf: &mut [u8]) -> io::Result<usize> {
    loop {
        match rd.read(buf) {
            Ok(n) => return Ok(n),
            Err(e) if e.kind() == ErrorKind::Interrupted => continue,
            Err(e) if e.kind() == ErrorKind::UnexpectedEof => return Ok(0),
            Err(e) => return Err(e),
        }
    }
}

/// The host and the path of an https URL, or None for anything else.
fn split_https(url: &str) -> Option<(String, String)> {
    let rest = url.strip_prefix("https://")?;
    let (host, path) = match rest.find('/') {
        Some(at) => (&rest[..at], &rest[at..]),
        None => (rest, "/"),
    };
    // A host with credentials or a port in it is not what hexdb answers with,
    // and neither is an empty one.
    if host.is_empty() || host.contains('@') || host.contains(':') {
        return None;
    }
    Some((host.to_string(), path.to_string()))
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::atomic::{AtomicU32, Ordering};

    /// A directory of this test's own, gone by the time the test returns.
    struct Temp(PathBuf);

    impl Temp {
        fn new(what: &str) -> Temp {
            static N: AtomicU32 = AtomicU32::new(0);
            let n = N.fetch_add(1, Ordering::Relaxed);
            let dir = std::env::temp_dir().join(format!(
                "anrb-hexdb-{what}-{}-{n}",
                std::process::id()
            ));
            let _ = fs::remove_dir_all(&dir);
            fs::create_dir_all(&dir).expect("a temporary directory");
            Temp(dir)
        }

        fn path(&self) -> &Path {
            &self.0
        }
    }

    impl Drop for Temp {
        fn drop(&mut self) {
            let _ = fs::remove_dir_all(&self.0);
        }
    }

    const DAY: Duration = Duration::from_secs(24 * 60 * 60);

    fn ttl() -> Duration {
        DAY * 10
    }

    // ---- the store ---------------------------------------------------------

    /// Written, closed, opened again: the same answers, without a word to
    /// hexdb.
    #[test]
    fn entries_survive_a_restart() {
        let tmp = Temp::new("restart");
        {
            let db = Hexdb::open(tmp.path(), ttl()).expect("open");
            db.file(Kind::Aircraft, "3c6444", Some(r#"{"Registration":"D-AIUH"}"#));
            db.file(Kind::Route, "dlh2ab", Some(r#"{"route":"EDDM-EDDL"}"#));
            db.file(Kind::Airport, "eddl", Some(r#"{"airport":"Dusseldorf"}"#));
            assert_eq!(db.stats().4, 3);
        }
        let db = Hexdb::open(tmp.path(), ttl()).expect("reopen");
        assert_eq!(db.stats().4, 3);
        assert_eq!(db.aircraft("3C6444"), Lookup::Found(r#"{"Registration":"D-AIUH"}"#.to_string()));
        assert_eq!(db.route("DLH2AB"), Lookup::Found(r#"{"route":"EDDM-EDDL"}"#.to_string()));
        assert_eq!(db.airport("EDDL"), Lookup::Found(r#"{"airport":"Dusseldorf"}"#.to_string()));
        // Three answers served from disk, nothing fetched.
        assert_eq!(db.stats().0, 3);
        assert_eq!(db.stats().1, 0);
    }

    /// A body with a newline and a backslash in it comes back as it went in,
    /// and does not become two lines on the way.
    #[test]
    fn a_body_with_line_breaks_survives_a_restart() {
        let tmp = Temp::new("escapes");
        let odd = "{\"a\":\"one\\two\",\n \"b\":\"three\"}\r\n";
        {
            let db = Hexdb::open(tmp.path(), ttl()).expect("open");
            db.file(Kind::Aircraft, "abc123", Some(odd));
        }
        let text = fs::read_to_string(tmp.path().join("entries.log")).expect("the log");
        assert_eq!(text.lines().count(), 1, "one entry is one line: {text:?}");
        let db = Hexdb::open(tmp.path(), ttl()).expect("reopen");
        assert_eq!(db.aircraft("ABC123"), Lookup::Found(odd.to_string()));
    }

    /// Ten days and a minute old is stale; a moment old is not.
    #[test]
    fn an_old_entry_is_stale() {
        let now = now_secs();
        let fresh = Entry { fetched: now, body: None };
        let old = Entry { fetched: now - (10 * 24 * 60 * 60 + 60), body: None };
        assert!(!fresh.stale(now, ttl()));
        assert!(old.stale(now, ttl()));
        // A clock that has jumped backwards leaves entries alone rather than
        // expiring the lot.
        assert!(!fresh.stale(now - 3600, ttl()));

        let tmp = Temp::new("stale");
        let db = Hexdb::open(tmp.path(), ttl()).expect("open");
        db.file(Kind::Aircraft, "3c6444", Some("{}"));
        assert!(db.fresh("a/3C6444").is_some());
        // Age it in place: the entry is still there, but not to be served
        // without asking again.
        {
            let mut s = lock(&db.store);
            if let Some(e) = s.entries.get_mut("a/3C6444") {
                e.fetched = now - 11 * 24 * 60 * 60;
            }
        }
        assert!(db.fresh("a/3C6444").is_none(), "past the ttl");
        assert!(db.cached("a/3C6444").is_some(), "but still there to serve if hexdb is down");
    }

    /// An address hexdb has never heard of is written down as such, so the
    /// next popup does not ask again.
    #[test]
    fn a_miss_is_remembered() {
        let tmp = Temp::new("miss");
        {
            let db = Hexdb::open(tmp.path(), ttl()).expect("open");
            db.file(Kind::Aircraft, "000001", None);
            db.file(Kind::Photo, "000002", None);
            assert_eq!(db.aircraft("000001"), Lookup::Missing);
        }
        let db = Hexdb::open(tmp.path(), ttl()).expect("reopen");
        assert_eq!(db.aircraft("000001"), Lookup::Missing);
        assert_eq!(db.photo("000002"), None);
        assert_eq!(db.stats().1, 0, "nothing was fetched");
        let e = db.peek(Kind::Aircraft, "000001").expect("the miss is an entry of its own");
        assert_eq!(e.body, None, "written down as a miss rather than as an empty answer");
    }

    /// The same string as an address, a callsign and a field is three entries.
    #[test]
    fn keys_of_different_kinds_do_not_collide() {
        let tmp = Temp::new("collide");
        {
            let db = Hexdb::open(tmp.path(), ttl()).expect("open");
            db.file(Kind::Route, "EDDL", Some("route"));
            db.file(Kind::Airport, "EDDL", Some("airport"));
            db.file(Kind::Aircraft, "ABCDEF", Some("aircraft"));
            db.file(Kind::Photo, "ABCDEF", Some("ABCDEF.jpg"));
        }
        let db = Hexdb::open(tmp.path(), ttl()).expect("reopen");
        assert_eq!(db.stats().4, 4);
        assert_eq!(db.route("EDDL"), Lookup::Found("route".to_string()));
        assert_eq!(db.airport("EDDL"), Lookup::Found("airport".to_string()));
        assert_eq!(db.aircraft("ABCDEF"), Lookup::Found("aircraft".to_string()));
        assert_eq!(db.peek(Kind::Photo, "ABCDEF").and_then(|e| e.body), Some("ABCDEF.jpg".to_string()));
    }

    /// A log with a torn tail and a line from nowhere still gives up the
    /// entries around them.
    #[test]
    fn a_corrupt_line_does_not_lose_the_others() {
        let tmp = Temp::new("corrupt");
        let now = now_secs();
        let log = format!(
            "a 3C6444 {now} + {{\"ok\":1}}\n\
             \n\
             this is not a line at all\n\
             a ZZZZZZ {now} + {{\"ok\":2}}\n\
             q 3C6445 {now} + {{\"ok\":3}}\n\
             a 3C6446 later + {{\"ok\":4}}\n\
             f EDDL {now} + {{\"airport\":\"Dusseldorf\"}}\n\
             r DLH2AB {now} - \n\
             a 3C64",
        );
        fs::write(tmp.path().join("entries.log"), &log).expect("write the log");

        let db = Hexdb::open(tmp.path(), ttl()).expect("open");
        assert_eq!(db.stats().4, 3, "the three that parse");
        assert_eq!(db.aircraft("3C6444"), Lookup::Found("{\"ok\":1}".to_string()));
        assert_eq!(db.airport("EDDL"), Lookup::Found("{\"airport\":\"Dusseldorf\"}".to_string()));
        assert_eq!(db.route("DLH2AB"), Lookup::Missing);
        // The lines that did not parse left nothing behind: a kind letter that
        // is not one, a fetch time that is not a number, a key of the wrong
        // shape, and the tail the power cut took.
        assert_eq!(db.peek(Kind::Aircraft, "3C6445"), None, "the kind letter q");
        assert_eq!(db.peek(Kind::Aircraft, "3C6446"), None, "a fetch time of 'later'");
    }

    /// The log stops growing without bound: rewriting the same key is
    /// compacted away, and the entries survive it.
    #[test]
    fn the_log_is_compacted() {
        let tmp = Temp::new("compact");
        {
            let db = Hexdb::open(tmp.path(), ttl()).expect("open");
            for i in 0..200 {
                db.file(Kind::Aircraft, &format!("{:06X}", i % 4), Some(&format!("{{\"n\":{i}}}")));
            }
            assert_eq!(db.stats().4, 4);
        }
        let text = fs::read_to_string(tmp.path().join("entries.log")).expect("the log");
        assert!(text.lines().count() < 70, "compacted, not 200 lines: {}", text.lines().count());
        assert!(!tmp.path().join("entries.tmp").exists(), "the temporary file is renamed, not left");
        let db = Hexdb::open(tmp.path(), ttl()).expect("reopen");
        assert_eq!(db.stats().4, 4);
        assert_eq!(db.aircraft("000003"), Lookup::Found("{\"n\":199}".to_string()));
    }

    /// Nothing that is not a key hexdb could have gets as far as a request.
    #[test]
    fn a_key_that_could_not_be_one_is_refused() {
        let tmp = Temp::new("keys");
        let db = Hexdb::open(tmp.path(), ttl()).expect("open");
        assert_eq!(db.aircraft("3c644"), Lookup::Missing, "five digits");
        assert_eq!(db.aircraft("3c644g"), Lookup::Missing, "not hex");
        assert_eq!(db.aircraft("../../etc"), Lookup::Missing);
        assert_eq!(db.route("DLH 2AB"), Lookup::Missing, "a space in a path");
        assert_eq!(db.route("DLH-2AB"), Lookup::Missing, "a dash");
        assert_eq!(db.route("ABCDEFGHIJKLM"), Lookup::Missing, "longer than 12");
        assert_eq!(db.route(""), Lookup::Missing);
        assert_eq!(db.airport("ED"), Lookup::Missing, "too short for a field");
        assert_eq!(db.airport("EDDLX"), Lookup::Missing, "too long for a field");
        assert_eq!(db.photo("nope"), None);
        assert_eq!(db.stats(), (0, 0, 0, 0, 0), "and nothing was counted as a lookup");
    }

    /// An image whose file has gone from under the entry reads as no image,
    /// rather than as an error or a panic.
    #[test]
    fn an_image_entry_without_its_file_is_no_image() {
        let tmp = Temp::new("image");
        let db = Hexdb::open(tmp.path(), ttl()).expect("open");
        fs::write(tmp.path().join("images/ABCDEF.jpg"), b"\xff\xd8jpeg").expect("write the image");
        db.file(Kind::Photo, "ABCDEF", Some("ABCDEF.jpg"));
        db.file(Kind::Photo, "ABCDE0", Some("ABCDE0.jpg"));
        db.file(Kind::Photo, "ABCDE1", Some("../entries.log"));
        assert_eq!(db.photo("ABCDEF"), Some(b"\xff\xd8jpeg".to_vec()));
        assert_eq!(db.photo("ABCDE0"), None, "the entry says there is one, the file is gone");
        assert_eq!(db.photo("ABCDE1"), None, "and a name that climbs out of images/ is refused");
    }

    // ---- the response parser ----------------------------------------------

    /// A reader that hands over what it was given in the pieces it was given,
    /// so a body split across reads can be exercised without a socket.
    struct Pieces(Vec<Vec<u8>>);

    impl Read for Pieces {
        fn read(&mut self, buf: &mut [u8]) -> io::Result<usize> {
            if self.0.is_empty() {
                return Ok(0);
            }
            let piece = self.0.remove(0);
            let n = piece.len().min(buf.len());
            buf[..n].copy_from_slice(&piece[..n]);
            if n < piece.len() {
                self.0.insert(0, piece[n..].to_vec());
            }
            Ok(n)
        }
    }

    fn pieces(parts: &[&str]) -> Pieces {
        Pieces(parts.iter().map(|p| p.as_bytes().to_vec()).collect())
    }

    #[test]
    fn a_content_length_body_parses() {
        let mut rd = pieces(&[
            "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: 17\r\n\r\n",
            "{\"Registration\":",
            "\"D-AIUH\"}and then some trailing rubbish",
        ]);
        let (status, body) = read_response(&mut rd).expect("a response");
        assert_eq!(status, 200);
        assert_eq!(body, b"{\"Registration\":\"", "exactly the seventeen bytes it promised");

        // A 404 with a body of its own, all in one read.
        let mut rd = pieces(&["HTTP/1.1 404 Not Found\r\nContent-Length: 3\r\n\r\nno!"]);
        assert_eq!(read_response(&mut rd).expect("a 404"), (404, b"no!".to_vec()));

        // A body that stops short of what it promised is an error, not a
        // truncated answer filed as if it were whole.
        let mut rd = pieces(&["HTTP/1.1 200 OK\r\nContent-Length: 10\r\n\r\nshort"]);
        assert!(read_response(&mut rd).is_err());
    }

    #[test]
    fn a_chunked_body_parses_however_it_arrives() {
        let whole = "HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n\
                     b\r\n{\"route\":\"E\r\n9\r\nDDM-EDDL\"\r\n1\r\n}\r\n0\r\n\r\n";
        let mut rd = pieces(&[whole]);
        let (status, body) = read_response(&mut rd).expect("a response");
        assert_eq!(status, 200);
        assert_eq!(body, br#"{"route":"EDDM-EDDL"}"#);

        // The same bytes, split so that a chunk header, a chunk body and the
        // final zero chunk each straddle a read.
        for at in 1..whole.len() {
            let mut rd = pieces(&[&whole[..at], &whole[at..]]);
            let (_, body) = read_response(&mut rd).unwrap_or_else(|e| panic!("split at {at}: {e}"));
            assert_eq!(body, br#"{"route":"EDDM-EDDL"}"#, "split at {at}");
        }

        // One byte at a time, which is every split at once.
        let mut rd = Pieces(whole.bytes().map(|b| vec![b]).collect());
        assert_eq!(read_response(&mut rd).expect("a response").1, br#"{"route":"EDDM-EDDL"}"#);
    }

    #[test]
    fn a_chunked_body_ends_at_its_zero_chunk() {
        // Nothing but the terminator: an empty body, not an error.
        let mut rd = pieces(&["HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n0\r\n\r\n"]);
        assert_eq!(read_response(&mut rd).expect("an empty body"), (200, Vec::new()));

        // A trailer after the zero chunk is read past and ignored.
        let mut rd = pieces(&[
            "HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n",
            "4\r\nhexd\r\n2\r\nb!\r\n0\r\nX-Checksum: 7\r\n\r\n",
        ]);
        assert_eq!(read_response(&mut rd).expect("a response").1, b"hexdb!".to_vec());

        // A chunk extension on the size line, and upper-case hex.
        let mut rd = pieces(&[
            "HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\nA;name=value\r\n0123456789\r\n0\r\n\r\n",
        ]);
        assert_eq!(read_response(&mut rd).expect("a response").1, b"0123456789".to_vec());

        // A body that stops before its last chunk is an error.
        let mut rd = pieces(&["HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\n4\r\nhexd\r\n"]);
        assert!(read_response(&mut rd).is_err());

        // A size that is not hex is refused rather than read as zero.
        let mut rd = pieces(&["HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\n\r\nzz\r\n"]);
        assert!(read_response(&mut rd).is_err());
    }

    /// Chunked is taken over Content-Length where a response carries both, and
    /// a response with neither runs to the end of the connection.
    #[test]
    fn the_framing_is_read_from_the_head() {
        assert_eq!(
            parse_head("HTTP/1.1 200 OK\r\nContent-Length: 12\r\n"),
            Some((200, Framing::Length(12)))
        );
        assert_eq!(
            parse_head("HTTP/1.1 301 Moved\r\ncontent-length: 0\r\nLocation: /elsewhere\r\n"),
            Some((301, Framing::Length(0)))
        );
        assert_eq!(
            parse_head("HTTP/1.1 200 OK\r\nContent-Length: 12\r\nTransfer-Encoding: chunked\r\n"),
            Some((200, Framing::Chunked))
        );
        assert_eq!(
            parse_head("HTTP/1.1 200 OK\r\nTransfer-Encoding: chunked\r\nContent-Length: 12\r\n"),
            Some((200, Framing::Chunked))
        );
        assert_eq!(parse_head("HTTP/1.1 200 OK\r\n"), Some((200, Framing::Eof)));
        assert_eq!(parse_head("not a status line at all\r\n"), None);

        let mut rd = pieces(&["HTTP/1.1 200 OK\r\n\r\nhttps://ex", "ample.invalid/a.jpg\n"]);
        let (status, body) = read_response(&mut rd).expect("a response");
        assert_eq!((status, String::from_utf8_lossy(&body).trim().to_string()),
                   (200, "https://example.invalid/a.jpg".to_string()));
    }

    /// The URL that comes back from hex-image-thumb, taken apart.
    #[test]
    fn an_image_url_is_split_or_refused() {
        assert_eq!(
            split_https("https://hexdb.io/img/3C6444.jpg"),
            Some(("hexdb.io".to_string(), "/img/3C6444.jpg".to_string()))
        );
        assert_eq!(split_https("https://hexdb.io"), Some(("hexdb.io".to_string(), "/".to_string())));
        assert_eq!(split_https("http://hexdb.io/img.jpg"), None, "plain http is not fetched");
        assert_eq!(split_https(""), None);
        assert_eq!(split_https("not a url"), None);
        assert_eq!(split_https("https://user@elsewhere.invalid/x"), None);
        assert_eq!(split_https("https://hexdb.io:8443/x"), None);
    }

    /// Two threads asking for the same key at the same moment: one of them
    /// fetches, the other waits and is handed what the first filed.
    #[test]
    fn one_claim_at_a_time_per_key() {
        let tmp = Temp::new("claim");
        let db = Arc::new(Hexdb::open(tmp.path(), ttl()).expect("open"));
        let first = db.claim("a/3C6444").expect("the first thread claims it");

        let waiter = {
            let db = Arc::clone(&db);
            std::thread::spawn(move || db.claim("a/3C6444").is_none())
        };
        // Long enough for the other thread to have reached the wait; the test
        // does not depend on it, since the claim is held until after the join.
        std::thread::sleep(Duration::from_millis(50));
        db.record("a/3C6444", Some("{}".to_string()));
        drop(first);
        assert!(waiter.join().expect("the waiting thread"), "it waited rather than fetching");
        assert_eq!(db.aircraft("3C6444"), Lookup::Found("{}".to_string()));
        assert!(lock(&db.inflight).is_empty(), "and the claim is released");
    }
}
