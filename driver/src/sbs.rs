//! BaseStation (SBS-1) output on a TCP port, the text format most ADS-B tools
//! read on port 30003.
//!
//! One comma-separated line per decoded message, to any number of readers,
//! over a [`Feed`].
//!
//! Message types 1-6 and 8 are emitted. Types 5 and 6 from surveillance
//! replies need the address-overlaid pass, except type 6 for TC28 emergency
//! status. Type 7 is not produced.

use crate::feed::{Event, Feed, ReaderInfo};
use crate::tracker::{Aircraft, Update};

/// A BaseStation feed on a TCP port.
pub struct SbsServer {
    feed: Feed,
}

impl SbsServer {
    /// Listen on `port` on every interface; 0 lets the system choose.
    pub fn bind(port: u16) -> std::io::Result<Self> {
        Ok(SbsServer { feed: Feed::bind(port)? })
    }
    /// The port listened on.
    pub fn port(&self) -> u16 { self.feed.port() }
    /// Readers currently attached.
    pub fn clients(&self) -> usize { self.feed.clients() }
    /// Lines sent while at least one reader was attached.
    pub fn lines(&self) -> u64 { self.feed.sent() }
    /// Readers cut off for falling behind.
    pub fn dropped(&self) -> u64 { self.feed.dropped() }
    /// Accept readers and flush queued bytes; call every time round the loop.
    pub fn poll(&mut self) { self.feed.poll(); }
    /// Who is attached.
    pub fn readers(&self) -> Vec<ReaderInfo> { self.feed.readers() }
    /// Joins and departures since the last call.
    pub fn take_events(&mut self) -> Vec<Event> { self.feed.take_events() }

    /// Emit the line this update produced, if the format has a place for it.
    pub fn emit(&mut self, a: &Aircraft, what: Update, now: std::time::SystemTime) {
        let (d, t) = stamp(now);
        if let Some(line) = format_line(a, what, &d, &t) {
            self.feed.send((line + "\r\n").as_bytes());
        }
    }
}

/// One BaseStation line for this update, without its terminator: 22
/// comma-separated fields, the date and time given twice (generated and
/// logged, which are the same moment here). `None` for what the format has no
/// message for - target state and operational status.
fn format_line(a: &Aircraft, what: Update, date: &str, time: &str) -> Option<String> {
    let tt = match what {
        Update::Identification       => 1,
        Update::SurfacePosition      => 2,
        Update::AirbornePosition     => 3,
        Update::Velocity             => 4,
        Update::SurveillanceAltitude => 5,
        Update::SurveillanceIdentity | Update::Status => 6,
        Update::Address              => 8,
        Update::TargetState | Update::OperationalStatus => return None,
    };

    // Field 11 onwards. Each message carries only what it said, so
    // a reader's history matches the transmissions rather than the tracker.
    let f = |o: Option<String>| o.unwrap_or_default();
    let callsign = if tt == 1 { f(a.callsign.clone()) } else { String::new() };
    let alt = if matches!(tt, 3 | 5) { f(a.alt.map(|v| v.to_string())) } else { String::new() };
    let (gs, trk) = if tt == 4 {
        (f(a.speed.map(|v| format!("{v:.0}"))), f(a.heading.map(|v| format!("{v:.0}"))))
    } else { (String::new(), String::new()) };
    // A position message that did not produce a fix - half a CPR pair, or
    // one still waiting to be confirmed - has no position to report.
    let fixed = a.t_pos == Some(a.t_seen);
    let (lat, lon) = if matches!(tt, 2 | 3) && fixed {
        (f(a.lat.map(|v| format!("{v:.5}"))), f(a.lon.map(|v| format!("{v:.5}"))))
    } else { (String::new(), String::new()) };
    let vr = if tt == 4 { f(a.vrate.map(|v| v.to_string())) } else { String::new() };
    let gnd = match tt { 2 => "-1", 3 => "0", _ => "" };
    let squawk = if tt == 6 { f(a.squawk.map(|v| format!("{v:04}"))) } else { String::new() };
    // Field 20 is the emergency flag, which only TC28 states.
    let emergency = match (what, a.emergency) {
        (Update::Status, Some(e)) => if e != 0 { "-1" } else { "0" },
        _ => "",
    };

    Some(format!("MSG,{tt},1,1,{:06X},1,{date},{time},{date},{time},\
{callsign},{alt},{gs},{trk},{lat},{lon},{vr},{squawk},,{emergency},,{gnd}", a.icao))
}

/// BaseStation timestamps are local wall clock, to the millisecond.
fn stamp(now: std::time::SystemTime) -> (String, String) {
    let t = crate::clock::local(now);
    (format!("{:04}/{:02}/{:02}", t.year, t.month, t.day),
     format!("{:02}:{:02}:{:02}.{:03}", t.hour, t.minute, t.second, t.millis))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tracker::Aircraft;

    /// `format_line` with placeholder timestamps: the fields are what the
    /// tests pin.
    fn line_for(a: &Aircraft, what: Update) -> String {
        format_line(a, what, "D", "T").unwrap_or_default()
    }

    #[test]
    fn every_line_has_the_twenty_two_fields_a_reader_expects() {
        let a = Aircraft { icao: 0x3C6551, callsign: Some("DLH8AB".into()),
                           alt: Some(35_000), lat: Some(50.1), lon: Some(8.5),
                           speed: Some(412.0), heading: Some(74.0), vrate: Some(-1408),
                           ..Default::default() };
        for w in [Update::Identification, Update::SurfacePosition,
                  Update::AirbornePosition, Update::Velocity, Update::Address,
                  Update::SurveillanceAltitude, Update::SurveillanceIdentity,
                  Update::Status] {
            let l = line_for(&a, w);
            assert_eq!(l.split(',').count(), 22, "{w:?} produced {l}");
            assert!(l.starts_with("MSG,"));
        }
    }

    /// A message must not carry fields it did not report: a velocity message
    /// has no position in it, and a position message has no ground speed.
    #[test]
    fn fields_follow_the_message_not_the_tracker() {
        let a = Aircraft { icao: 0x4009DA, callsign: Some("BAW11".into()),
                           alt: Some(35_000), lat: Some(50.1), lon: Some(8.5),
                           t_pos: Some(0), speed: Some(412.0), heading: Some(74.0),
                           vrate: Some(0), squawk: Some(1000), ..Default::default() };
        let vel = line_for(&a, Update::Velocity);
        let v: Vec<&str> = vel.split(',').collect();
        assert_eq!(v[14], "", "velocity carries no latitude");
        assert_eq!(v[15], "", "velocity carries no longitude");
        assert_eq!(v[12], "412", "but it does carry ground speed");

        let pos = line_for(&a, Update::AirbornePosition);
        let p: Vec<&str> = pos.split(',').collect();
        assert_eq!(p[12], "", "position carries no ground speed");
        assert_eq!(p[11], "35000", "but it does carry altitude");
        assert_eq!(p[21], "0", "and says it is airborne");

        let all = line_for(&a, Update::Address);
        let c: Vec<&str> = all.split(',').collect();
        assert!(c[10..21].iter().all(|f| f.is_empty()), "an all-call carries only the address");

        let id = line_for(&a, Update::SurveillanceIdentity);
        let i: Vec<&str> = id.split(',').collect();
        assert_eq!((i[1], i[17]), ("6", "1000"), "a squawk reply carries the squawk");
        assert_eq!(i[19], "", "but says nothing about an emergency");
    }

    /// Half a CPR pair is a position message with no position in it yet.
    #[test]
    fn a_position_message_without_a_fix_has_no_position() {
        let a = Aircraft { icao: 0x4009DA, lat: Some(50.1), lon: Some(8.5),
                           t_pos: Some(1_000), t_seen: 1_500, ..Default::default() };
        let p: Vec<String> = line_for(&a, Update::AirbornePosition).split(',').map(String::from).collect();
        assert_eq!((p[14].as_str(), p[15].as_str()), ("", ""), "the fix is from an earlier message");
    }

    #[test]
    fn target_state_has_no_basestation_message() {
        assert_eq!(format_line(&Aircraft::default(), Update::TargetState, "D", "T"), None);
    }
}
