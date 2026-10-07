//! What the map bridge holds and what it hands to the page.
//!
//! A browser cannot open a raw TCP socket, so the receiver's feed is read
//! here, kept for fifteen minutes per aircraft, and served over HTTP. The page
//! asks for everything once and then for points newer than the last sequence
//! number it saw, which keeps a poll to a few kilobytes rather than every
//! track every second.
//!
//! The page (web/tracks.js, tested by web/test_tracks.cjs) assumes these
//! thinning and expiry rules.

use std::collections::HashMap;
use std::sync::Mutex;
use std::time::{SystemTime, UNIX_EPOCH};

use anrb::tracker::Aircraft;

use crate::lock;
use crate::stats::Stats;

/// How long a track is kept after its last point, in seconds.
pub const HISTORY: f64 = 15.0 * 60.0;
/// How long an aircraft may be quiet before the page takes it off the map, in
/// seconds.
pub const INACTIVE: f64 = 60.0;
/// Thinning: a point closer than this in time and in distance to the last one
/// says nothing the map can draw.
const MIN_DT: f64 = 2.0;
const MIN_MOVE: f64 = 0.0004; // degrees, roughly 40 m

/// Seconds since the epoch, which is the clock the page draws on.
pub fn now() -> f64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0.0, |d| d.as_secs_f64())
}

struct Point {
    seq: u64,
    t: f64,
    lon: f64,
    lat: f64,
    alt: Option<i32>,
}

/// One aircraft as the page needs it: the fields it labels, and the track.
#[derive(Default)]
struct Record {
    callsign: Option<String>,
    alt: Option<i32>,
    speed: Option<f64>,
    track: Option<f64>,
    vrate: Option<i32>,
    squawk: Option<u16>,
    ground: bool,
    /// The fields Beast carries and BaseStation has no column for, rendered
    /// once when they change rather than on every update.
    extra: String,
    last: f64,
    pts: Vec<Point>,
}

/// The reader's rejection counters, shown in the stats card.
#[derive(Default)]
struct Checks {
    bad_parity: u64,
    implausible: u64,
    resyncs: u64,
}

struct Inner {
    ac: HashMap<u32, Record>,
    seq: u64,
    connected: bool,
    /// Beast frames or BaseStation lines read since the bridge started.
    messages: u64,
    checks: Checks,
    stats: Stats,
}

pub struct State {
    inner: Mutex<Inner>,
    source: &'static str,
    /// Whether the bridge was given a directory of airline logos. The page
    /// asks for one only when there is somewhere to get it from.
    logos: bool,
    /// Whether the bridge answers lookups at all. With them off it makes no
    /// outbound request, and the page is told so rather than asking for
    /// registrations and routes that can only come back as 404s.
    lookups: bool,
}

impl State {
    pub fn new(source: &'static str) -> State {
        State {
            inner: Mutex::new(Inner {
                ac: HashMap::new(),
                seq: 0,
                connected: false,
                messages: 0,
                checks: Checks::default(),
                stats: Stats::new(None),
            }),
            source,
            logos: false,
            lookups: false,
        }
    }

    /// Where the receiver is, which the range statistics are measured from.
    pub fn set_site(&mut self, site: (f64, f64)) {
        lock(&self.inner).stats = Stats::new(Some(site));
    }

    /// The statistics as the page reads them.
    pub fn stats_json(&self) -> String {
        lock(&self.inner).stats.json(now())
    }

    /// The statistics as text, for [`State::load_stats`] after a restart.
    pub fn save_stats(&self) -> String {
        lock(&self.inner).stats.save()
    }

    /// Read back statistics saved by [`State::save_stats`].
    pub fn load_stats(&self, text: &str) {
        lock(&self.inner).stats.load(text, now());
    }

    /// Say that `/logo/<code>` will answer.
    pub fn serving_logos(&mut self) {
        self.logos = true;
    }

    /// Say that the `/api/` and `/photo/` endpoints will answer.
    pub fn serving_lookups(&mut self) {
        self.lookups = true;
    }

    pub fn set_connected(&self, up: bool) {
        lock(&self.inner).connected = up;
    }

    /// The reader's running totals, shown in the stats card.
    pub fn counters(&self, messages: u64, bad_parity: u64, implausible: u64, resyncs: u64) {
        let mut s = lock(&self.inner);
        s.messages = messages;
        s.checks = Checks {
            bad_parity,
            implausible,
            resyncs,
        };
        let t = now();
        let active = s.ac.values().filter(|r| t - r.last <= INACTIVE).count();
        s.stats.counts(messages, active, t);
    }

    /// A position the tracker has published.
    pub fn point(&self, icao: u32, lat: f64, lon: f64, alt: Option<i32>, now: f64) {
        let mut s = lock(&self.inner);
        s.stats.position(lat, lon, now);
        let seq = s.seq;
        let r = s.ac.entry(icao).or_default();
        r.last = now;
        if let Some(a) = alt {
            r.alt = Some(a);
        }
        if let Some(p) = r.pts.last() {
            if now - p.t < MIN_DT
                && (lon - p.lon).abs() < MIN_MOVE
                && (lat - p.lat).abs() < MIN_MOVE
            {
                return;
            }
        }
        let alt = r.alt;
        r.pts.push(Point {
            seq: seq + 1,
            t: now,
            lon,
            lat,
            alt,
        });
        s.seq += 1;
    }

    /// Take the tracker's current picture of one aircraft.
    pub fn fields(&self, a: &Aircraft, seen: f64, now: f64) {
        let mut s = lock(&self.inner);
        let r = s.ac.entry(a.icao).or_default();
        r.last = now - seen;
        r.callsign = a.callsign.clone();
        r.alt = a.alt;
        r.speed = a.speed;
        r.track = a.heading;
        r.vrate = a.vrate;
        r.squawk = a.squawk;
        r.ground = a.on_ground;
        r.extra = extra(a);
    }

    /// Drop points that have fallen out of the window, and aircraft left with
    /// nothing at all.
    pub fn expire(&self, now: f64) {
        let mut s = lock(&self.inner);
        let cutoff = now - HISTORY;
        s.ac.retain(|_, r| {
            r.pts.retain(|p| p.t >= cutoff);
            !r.pts.is_empty() || r.last >= cutoff
        });
    }

    /// The message the page reads, as JSON: points newer than `since`, the
    /// current fields of every active aircraft, and the ones to forget.
    ///
    /// A page that has seen more than exists is talking to a bridge that
    /// restarted since, and is sent everything this one holds.
    pub fn updates(&self, since: u64) -> String {
        let now = now();
        let s = lock(&self.inner);
        let since = if since > s.seq { 0 } else { since };

        let mut out = String::with_capacity(4096);
        out.push_str("{\"now\":");
        num(&mut out, now, 1);
        out.push_str(",\"seq\":");
        out.push_str(&s.seq.to_string());
        out.push_str(",\"connected\":");
        out.push_str(if s.connected { "true" } else { "false" });
        out.push_str(",\"source\":\"");
        out.push_str(self.source);
        out.push_str("\",\"logos\":");
        out.push_str(if self.logos { "true" } else { "false" });
        out.push_str(",\"lookups\":");
        out.push_str(if self.lookups { "true" } else { "false" });
        out.push_str(",\"messages\":");
        out.push_str(&s.messages.to_string());
        out.push_str(",\"checks\":{\"bad_parity\":");
        out.push_str(&s.checks.bad_parity.to_string());
        out.push_str(",\"implausible\":");
        out.push_str(&s.checks.implausible.to_string());
        out.push_str(",\"resyncs\":");
        out.push_str(&s.checks.resyncs.to_string());
        out.push_str("},\"ac\":{");

        let mut drop = Vec::new();
        let mut first = true;
        for (&icao, r) in s.ac.iter() {
            let age = now - r.last;
            if age > INACTIVE {
                drop.push(icao);
                continue;
            }
            if !first {
                out.push(',');
            }
            first = false;
            out.push_str(&format!("\"{icao:06X}\":{{\"cs\":"));
            match &r.callsign {
                Some(cs) => string(&mut out, cs),
                None => out.push_str("null"),
            }
            out.push_str(",\"alt\":");
            int(&mut out, r.alt);
            out.push_str(",\"gs\":");
            opt(&mut out, r.speed, 1);
            out.push_str(",\"trk\":");
            opt(&mut out, r.track, 1);
            out.push_str(",\"vr\":");
            int(&mut out, r.vrate);
            out.push_str(",\"sq\":");
            match r.squawk {
                Some(q) => out.push_str(&format!("\"{q:04}\"")),
                None => out.push_str("null"),
            }
            out.push_str(",\"gnd\":");
            out.push_str(if r.ground { "1" } else { "0" });
            out.push_str(",\"age\":");
            num(&mut out, age, 1);
            if !r.extra.is_empty() {
                out.push_str(",\"x\":");
                out.push_str(&r.extra);
            }
            out.push_str(",\"new\":[");
            let mut firstp = true;
            for p in r.pts.iter().filter(|p| p.seq > since) {
                if !firstp {
                    out.push(',');
                }
                firstp = false;
                out.push('[');
                num(&mut out, p.t, 1);
                out.push(',');
                num(&mut out, p.lon, 5);
                out.push(',');
                num(&mut out, p.lat, 5);
                out.push(',');
                int(&mut out, p.alt);
                out.push(']');
            }
            out.push_str("]}");
        }
        out.push_str("},\"drop\":[");
        for (i, icao) in drop.iter().enumerate() {
            if i > 0 {
                out.push(',');
            }
            out.push_str(&format!("\"{icao:06X}\""));
        }
        out.push_str("],\"history\":");
        out.push_str(&(HISTORY as u64).to_string());
        out.push_str(",\"inactive\":");
        out.push_str(&(INACTIVE as u64).to_string());
        out.push('}');
        out
    }
}

/// The Beast-only fields, as the object the popup reads. Empty when the
/// aircraft has sent none of them.
fn extra(a: &Aircraft) -> String {
    let mut f: Vec<String> = Vec::new();
    if let Some(c) = a.category {
        f.push(format!("\"category\":\"{c}\""));
    }
    if let Some(v) = a.emergency {
        f.push(format!("\"emergency\":{v}"));
    }
    if let Some(v) = a.gnss_alt {
        f.push(format!("\"gnss_alt\":{v}"));
    }
    if let Some(v) = a.geo_minus_baro {
        f.push(format!("\"geo_minus_baro\":{v}"));
    }
    if let Some(s) = a.airspeed {
        if let Some(kt) = s.knots {
            f.push(format!(
                "\"{}\":{kt}",
                if s.true_airspeed { "tas" } else { "ias" }
            ));
        }
        if let Some(h) = s.heading {
            f.push(format!("\"mag_heading\":{h:.1}"));
        }
    }
    if let Some(v) = a.adsb_version {
        f.push(format!("\"version\":{v}"));
    }
    if let Some(t) = a.target {
        let mut g: Vec<String> = Vec::new();
        if let Some(alt) = t.altitude {
            g.push(format!("\"alt\":{alt}"));
            g.push(format!(
                "\"alt_source\":\"{}\"",
                if t.altitude_fms { "FMS" } else { "MCP/FCU" }
            ));
        }
        if let Some(q) = t.qnh {
            g.push(format!("\"qnh\":{q:.1}"));
        }
        if let Some(h) = t.heading {
            g.push(format!("\"heading\":{h:.1}"));
        }
        if let Some(m) = t.modes {
            let on: Vec<&str> = [
                (m.autopilot, "AP"),
                (m.alt_hold, "ALT"),
                (m.vnav, "VNAV"),
                (m.lnav, "LNAV"),
                (m.approach, "APP"),
            ]
            .into_iter()
            .filter(|(v, _)| *v)
            .map(|(_, n)| n)
            .collect();
            g.push(format!(
                "\"modes\":[{}]",
                on.iter()
                    .map(|n| format!("\"{n}\""))
                    .collect::<Vec<_>>()
                    .join(",")
            ));
        }
        g.push(format!("\"tcas\":{}", t.tcas));
        f.push(format!("\"target\":{{{}}}", g.join(",")));
    }
    if f.is_empty() {
        String::new()
    } else {
        format!("{{{}}}", f.join(","))
    }
}

/// A number with at most `dp` decimal places, trailing zeros removed. `dp` is
/// at least 1, so the formatted text has a decimal point for the trimming to
/// stop at.
fn num(out: &mut String, v: f64, dp: usize) {
    // Trailing zeros are noise on the wire, and JSON has no use for them.
    let s = format!("{v:.dp$}");
    out.push_str(s.trim_end_matches('0').trim_end_matches('.'));
}

fn opt(out: &mut String, v: Option<f64>, dp: usize) {
    match v {
        Some(v) => num(out, v, dp),
        None => out.push_str("null"),
    }
}

fn int(out: &mut String, v: Option<i32>) {
    match v {
        Some(v) => out.push_str(&v.to_string()),
        None => out.push_str("null"),
    }
}

/// A JSON string. Callsigns are six-bit ASCII and hold nothing exotic, but the
/// input is untrusted, so it is escaped.
fn string(out: &mut String, s: &str) {
    out.push('"');
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out.push('"');
}

#[cfg(test)]
mod tests {
    use super::*;
    use anrb::tracker::{Category, Tracker};

    fn state() -> State {
        State::new("beast")
    }

    /// The message carries the shape the page parses: a sequence number to
    /// poll from, the aircraft it knows, and the points since that number.
    #[test]
    fn updates_carry_new_points_only() {
        let s = state();
        let t0 = now();
        s.point(0x4009DA, 50.1, 8.5, Some(35_000), t0);
        s.point(0x4009DA, 50.2, 8.6, Some(35_000), t0 + 5.0);
        let first = s.updates(0);
        assert!(first.contains("\"4009DA\""), "{first}");
        assert_eq!(
            first.matches("[").count() - first.matches("\"drop\":[").count(),
            3,
            "two points and their array: {first}"
        );
        assert!(first.contains("\"seq\":2"));
        s.counters(7, 1, 2, 3);
        let counted = s.updates(0);
        assert!(counted.contains("\"messages\":7"), "{counted}");
        assert!(
            counted.contains("\"checks\":{\"bad_parity\":1,\"implausible\":2,\"resyncs\":3}"),
            "{counted}"
        );
        assert!(counted.contains("\"history\":900"), "{counted}");

        let again = s.updates(2);
        assert!(
            again.contains("\"new\":[]"),
            "nothing newer than the last poll: {again}"
        );
    }

    #[test]
    fn a_point_too_close_to_the_last_is_dropped() {
        let s = state();
        let t0 = now();
        s.point(0x3C6551, 50.0, 8.0, None, t0);
        s.point(0x3C6551, 50.00001, 8.00001, None, t0 + 0.5); // same place, moments later
        assert!(
            s.updates(0).contains("\"seq\":1"),
            "the second point says nothing new"
        );
        s.point(0x3C6551, 50.05, 8.05, None, t0 + 0.6); // moved
        assert!(s.updates(0).contains("\"seq\":2"));
    }

    #[test]
    fn a_quiet_aircraft_is_dropped_then_forgotten() {
        let s = state();
        let t0 = now();
        s.point(0x4009DA, 50.1, 8.5, None, t0 - INACTIVE - 1.0);
        let msg = s.updates(0);
        assert!(msg.contains("\"drop\":[\"4009DA\"]"), "{msg}");
        assert!(
            !msg.contains("\"4009DA\":{"),
            "and it is not in the fleet: {msg}"
        );

        s.expire(t0);
        assert!(
            s.updates(0).contains("\"drop\":[\"4009DA\"]"),
            "still held inside the history window, so it can come back"
        );
        s.expire(t0 + HISTORY + 1.0);
        let gone = s.updates(0);
        assert!(
            gone.contains("\"ac\":{}") && gone.contains("\"drop\":[]"),
            "{gone}"
        );
    }

    /// The fields Beast carries and BaseStation does not, as the popup reads
    /// them. A field the aircraft never sent must not appear at all.
    #[test]
    fn extra_fields_are_only_what_was_received() {
        let mut a = Aircraft::new(0x4009DA);
        assert_eq!(extra(&a), "", "nothing known yet");

        a.category = Some(Category { set: 'A', code: 5 });
        a.adsb_version = Some(2);
        let x = extra(&a);
        assert!(x.contains("\"category\":\"A5\""), "{x}");
        assert!(x.contains("\"version\":2"), "{x}");
        assert!(!x.contains("target"), "no target state was sent: {x}");
    }

    /// The tracker's own output must land in the message unchanged.
    #[test]
    fn a_decoded_frame_reaches_the_page() {
        // A DF17 identification message for 4009DA, callsign BAW11, emitter
        // category A5, with the parity made good so the tracker accepts it the
        // way it accepts one off the wire.
        let mut f = vec![
            0x8D,
            0x40,
            0x09,
            0xDA,
            (4 << 3) | 5,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
            0,
        ];
        let mut packed: u64 = 0;
        for c in [2u64, 1, 23, 49, 49, 32, 32, 32] {
            // B A W 1 1 and three pads
            packed = (packed << 6) | c;
        }
        for (i, b) in f[5..11].iter_mut().enumerate() {
            *b = (packed >> (40 - 8 * i)) as u8;
        }
        let p = anrb::crc::Crc::new().crc24(&f[..11]);
        f[11..].copy_from_slice(&[(p >> 16) as u8, (p >> 8) as u8, p as u8]);

        let mut t = Tracker::new();
        t.update_raw(&f, 0).expect("a frame the tracker accepts");
        let a = &t.table[&0x4009DA];
        let s = state();
        s.fields(a, 0.0, now());
        let msg = s.updates(0);
        assert_eq!(a.callsign.as_deref(), Some("BAW11"), "the callsign decoded");
        assert!(
            msg.contains("\"cs\":\"BAW11\""),
            "and is in the message: {msg}"
        );
        assert!(
            msg.contains("\"category\":\"A5\""),
            "with the category: {msg}"
        );
    }

    /// The head of the message says what the bridge is connected to and what
    /// it will answer.
    #[test]
    fn the_head_says_what_the_bridge_serves() {
        let mut s = State::new("sbs");
        let msg = s.updates(0);
        assert!(msg.contains("\"seq\":0,\"connected\":false,\"source\":\"sbs\",\"logos\":false,\"lookups\":false,"),
                "{msg}");
        s.serving_logos();
        s.serving_lookups();
        s.set_connected(true);
        let msg = s.updates(0);
        assert!(
            msg.contains("\"connected\":true,\"source\":\"sbs\",\"logos\":true,\"lookups\":true,"),
            "{msg}"
        );
        assert!(msg.ends_with(",\"history\":900,\"inactive\":60}"), "{msg}");
    }

    /// Every field of an aircraft, with a callsign that needs escaping, and
    /// an aircraft that has sent nothing but a position.
    #[test]
    fn every_field_of_an_aircraft_is_written() {
        let s = state();
        let t = now();
        let mut a = Aircraft::new(0x00ABCD);
        a.callsign = Some("A\"B\\C\u{1}".to_string());
        a.alt = Some(35_000);
        a.speed = Some(451.3);
        a.heading = Some(90.0);
        a.vrate = Some(-640);
        a.squawk = Some(7);
        a.on_ground = true;
        s.fields(&a, 0.0, t);
        s.point(0x4009DA, 50.123456, 8.5, None, t);
        let msg = s.updates(0);
        assert!(msg.contains(r#""00ABCD":{"cs":"A\"B\\C\u0001","alt":35000,"gs":451.3,"trk":90,"vr":-640,"sq":"0007","gnd":1,"age":"#),
                "{msg}");
        assert!(msg.contains(r#""4009DA":{"cs":null,"alt":null,"gs":null,"trk":null,"vr":null,"sq":null,"gnd":0,"age":"#),
                "{msg}");
        assert!(
            msg.contains(r#""new":[["#) && msg.contains(r#",8.5,50.12346,null]]"#),
            "{msg}"
        );
        assert!(msg.contains("]}},\"drop\":[]"), "{msg}");
        assert_eq!(msg.matches("\"cs\":").count(), 2, "two aircraft: {msg}");
    }

    /// Several quiet aircraft are all listed to drop.
    #[test]
    fn several_quiet_aircraft_are_all_dropped() {
        let s = state();
        let old = now() - INACTIVE - 5.0;
        s.point(0x000001, 50.0, 8.0, None, old);
        s.point(0x000002, 51.0, 9.0, None, old);
        let msg = s.updates(0);
        assert!(msg.contains("\"ac\":{}"), "{msg}");
        assert!(
            msg.contains("\"drop\":[\"000001\",\"000002\"]")
                || msg.contains("\"drop\":[\"000002\",\"000001\"]"),
            "{msg}"
        );
    }

    /// Every Beast-only field, in the order the popup is given them.
    #[test]
    fn every_extra_field_is_written() {
        use anrb::tracker::{Airspeed, Modes, TargetState};
        let mut a = Aircraft::new(0x4009DA);
        a.category = Some(Category { set: 'B', code: 2 });
        a.emergency = Some(1);
        a.gnss_alt = Some(35_100);
        a.geo_minus_baro = Some(-75);
        a.airspeed = Some(Airspeed {
            heading: Some(123.4),
            knots: Some(250),
            true_airspeed: false,
        });
        a.adsb_version = Some(2);
        a.target = Some(TargetState {
            altitude: Some(36_000),
            altitude_fms: false,
            qnh: Some(1013.2),
            heading: Some(270.0),
            modes: Some(Modes {
                autopilot: true,
                lnav: true,
                ..Modes::default()
            }),
            tcas: true,
        });
        assert_eq!(
            extra(&a),
            r#"{"category":"B2","emergency":1,"gnss_alt":35100,"geo_minus_baro":-75,"ias":250,"mag_heading":123.4,"version":2,"target":{"alt":36000,"alt_source":"MCP/FCU","qnh":1013.2,"heading":270.0,"modes":["AP","LNAV"],"tcas":true}}"#
        );

        // True airspeed with no heading, and a target from the FMS with
        // nothing else set.
        let mut a = Aircraft::new(0x4009DA);
        a.airspeed = Some(Airspeed {
            heading: None,
            knots: Some(480),
            true_airspeed: true,
        });
        a.target = Some(TargetState {
            altitude: Some(24_000),
            altitude_fms: true,
            ..TargetState::default()
        });
        assert_eq!(
            extra(&a),
            r#"{"tas":480,"target":{"alt":24000,"alt_source":"FMS","tcas":false}}"#
        );

        // An airspeed message with neither field, and modes all off.
        let mut a = Aircraft::new(0x4009DA);
        a.airspeed = Some(Airspeed {
            heading: None,
            knots: None,
            true_airspeed: false,
        });
        a.target = Some(TargetState {
            modes: Some(Modes::default()),
            ..TargetState::default()
        });
        assert_eq!(extra(&a), r#"{"target":{"modes":[],"tcas":false}}"#);
    }
}
