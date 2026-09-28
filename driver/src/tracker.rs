//! ADS-B field decoding and aircraft tracking.
//!
//! Identification and category, airborne and surface position, velocity and
//! airspeed, emergency status, target state and ADS-B version from DF17/18;
//! the bare address from a DF11 all-call; altitude and squawk from the
//! surveillance replies.
//!
//! Comm-B registers (the MB field of DF20/21) and the integrity and accuracy
//! figures are not decoded.

use std::collections::HashMap;

use crate::crc::{ApMap, Crc};

// ---- altitude -------------------------------------------------------------

fn gray2bin(mut g: u32) -> i32 {
    let mut b = 0u32;
    while g != 0 {
        b ^= g;
        g >>= 1;
    }
    b as i32
}

/// A Gillham (100 ft) altitude from the 12-bit AC field, laid out
/// C1 A1 C2 A2 C4 A4 B1 Q B2 D2 B4 D4 with Q clear.
///
/// D2 D4 A1 A2 A4 B1 B2 B4 are a Gray code counting 500 ft bands. C1 C2 C4
/// are a Gray code for the 100 ft step within the band, 1 to 5, which counts
/// down in odd bands. `None` for a C combination the code never uses.
fn gillham_alt(ac: u32) -> Option<i32> {
    let bit = |n: u32| (ac >> n) & 1;
    let (c1, a1, c2, a2, c4, a4, b1) = (bit(11), bit(10), bit(9), bit(8), bit(7), bit(6), bit(5));
    let (b2, d2, b4, d4) = (bit(3), bit(2), bit(1), bit(0));
    let n500 = gray2bin(d2 << 7 | d4 << 6 | a1 << 5 | a2 << 4 | a4 << 3 | b1 << 2 | b2 << 1 | b4);
    let n100 = match gray2bin(c1 << 2 | c2 << 1 | c4) {
        0 | 5 | 6 => return None,
        // C1 alone is the fifth step.
        7 => 5,
        n => n,
    };
    let n100 = if n500 % 2 == 1 { 6 - n100 } else { n100 };
    Some(n500 * 500 + n100 * 100 - 1300)
}

/// The 12-bit AC field carried by DF17 position messages. `None` when the
/// field is unused (all zero) or holds an illegal Gillham code.
pub fn ac12(ac: i32) -> Option<i32> {
    if ac == 0 {
        return None;
    }
    if (ac >> 4) & 1 != 0 {
        // Q bit: 25 ft steps
        let n = ((ac & 0x0FE0) >> 1) | (ac & 0x000F);
        return Some(n * 25 - 1000);
    }
    gillham_alt(ac as u32)
}

// ---- compact position reporting -------------------------------------------

/// Number of longitude zones at a given latitude.
pub fn nl(lat: f64) -> i32 {
    let lat = lat.abs();
    if lat >= 90.0 {
        return 1;
    }
    if lat == 0.0 {
        return 59;
    }
    const NZ: f64 = 15.0;
    let c = (std::f64::consts::PI / 180.0 * lat).cos();
    let x = (1.0 - (1.0 - (std::f64::consts::PI / (2.0 * NZ)).cos()) / (c * c)).clamp(-1.0, 1.0);
    (2.0 * std::f64::consts::PI / x.acos())
        .floor()
        .clamp(1.0, 59.0) as i32
}

fn cpr_mod(a: f64, b: f64) -> f64 {
    let r = a % b;
    if r < 0.0 {
        r + b
    } else {
        r
    }
}
fn cpr_modi(a: i32, b: i32) -> i32 {
    let r = a % b;
    if r < 0 {
        r + b
    } else {
        r
    }
}

/// Global decode: one even and one odd frame give an unambiguous position.
pub fn cpr_global(
    lat_e: u32,
    lon_e: u32,
    lat_o: u32,
    lon_o: u32,
    odd_is_newer: bool,
) -> Option<(f64, f64)> {
    const D0: f64 = 360.0 / 60.0;
    const D1: f64 = 360.0 / 59.0;
    const S: f64 = 1.0 / 131072.0;

    let j = (((59.0 * lat_e as f64 - 60.0 * lat_o as f64) * S) + 0.5).floor() as i32;
    let mut rlat0 = D0 * (cpr_modi(j, 60) as f64 + lat_e as f64 * S);
    let mut rlat1 = D1 * (cpr_modi(j, 59) as f64 + lat_o as f64 * S);
    if rlat0 >= 270.0 {
        rlat0 -= 360.0;
    }
    if rlat1 >= 270.0 {
        rlat1 -= 360.0;
    }
    if !(-90.0..=90.0).contains(&rlat0) || !(-90.0..=90.0).contains(&rlat1) {
        return None;
    }

    let (nl0, nl1) = (nl(rlat0), nl(rlat1));
    if nl0 != nl1 {
        return None;
    } // the pair straddles a zone boundary

    let (ni, m, rlat, lon_src) = if odd_is_newer {
        let ni = (nl1 - 1).max(1);
        let m = ((lon_e as f64 * (nl1 - 1) as f64 - lon_o as f64 * nl1 as f64) * S + 0.5).floor()
            as i32;
        (ni, m, rlat1, lon_o)
    } else {
        let ni = nl0.max(1);
        let m = ((lon_e as f64 * (nl0 - 1) as f64 - lon_o as f64 * nl0 as f64) * S + 0.5).floor()
            as i32;
        (ni, m, rlat0, lon_e)
    };
    let mut rlon = (360.0 / ni as f64) * (cpr_modi(m, ni) as f64 + lon_src as f64 * S);
    rlon = cpr_mod(rlon, 360.0);
    if rlon > 180.0 {
        rlon -= 360.0;
    }
    Some((rlat, rlon))
}

/// Local decode against a known nearby position: one frame is enough.
pub fn cpr_local(
    lat_cpr: u32,
    lon_cpr: u32,
    odd: bool,
    surface: bool,
    ref_lat: f64,
    ref_lon: f64,
) -> Option<(f64, f64)> {
    let span = if surface { 90.0 } else { 360.0 };
    const S: f64 = 1.0 / 131072.0;

    let dlat = span / (60.0 - if odd { 1.0 } else { 0.0 });
    let j = (ref_lat / dlat).floor()
        + (0.5 + cpr_mod(ref_lat, dlat) / dlat - lat_cpr as f64 * S).floor();
    let rlat = dlat * (j + lat_cpr as f64 * S);
    if !(-90.0..=90.0).contains(&rlat) {
        return None;
    }
    if (rlat - ref_lat).abs() > if surface { 45.0 } else { 90.0 } {
        return None;
    }

    let n = (nl(rlat) - if odd { 1 } else { 0 }).max(1);
    let dlon = span / n as f64;
    let m = (ref_lon / dlon).floor()
        + (0.5 + cpr_mod(ref_lon, dlon) / dlon - lon_cpr as f64 * S).floor();
    let mut rlon = dlon * (m + lon_cpr as f64 * S);
    while rlon > 180.0 {
        rlon -= 360.0;
    }
    while rlon < -180.0 {
        rlon += 360.0;
    }
    Some((rlat, rlon))
}

// ---- velocity and identification ------------------------------------------

/// TC19 subtypes 1 and 2: ground speed, track and vertical rate. The rate is
/// `None` when the message says it has none - which is not the same as level
/// flight, a rate of 0 that it can also report.
pub fn velocity(me: &[u8]) -> Option<(f64, f64, Option<i32>)> {
    let st = me[0] & 7;
    if st != 1 && st != 2 {
        return None;
    }
    let ew_sign = (me[1] >> 2) & 1;
    let ew = (((me[1] & 3) as i32) << 8) | me[2] as i32;
    let ns_sign = (me[3] >> 7) & 1;
    let ns = (((me[3] & 0x7F) as i32) << 3) | (me[4] >> 5) as i32;
    if ew == 0 || ns == 0 {
        return None;
    }

    let mut vx = (ew - 1) as f64 * if ew_sign != 0 { -1.0 } else { 1.0 };
    let mut vy = (ns - 1) as f64 * if ns_sign != 0 { -1.0 } else { 1.0 };
    if st == 2 {
        vx *= 4.0;
        vy *= 4.0;
    } // supersonic
    let speed = (vx * vx + vy * vy).sqrt();
    let mut heading = vx.atan2(vy) * 180.0 / std::f64::consts::PI;
    if heading < 0.0 {
        heading += 360.0;
    }

    let vr_sign = (me[4] >> 3) & 1;
    let vr = (((me[4] & 7) as i32) << 6) | (me[5] >> 2) as i32;
    let vrate = (vr != 0).then(|| (vr - 1) * 64 * if vr_sign != 0 { -1 } else { 1 });
    Some((speed, heading, vrate))
}

const CS6: &[u8; 64] = b"#ABCDEFGHIJKLMNOPQRSTUVWXYZ#####_###############0123456789######";

/// TC1-4: the eight-character flight identification.
pub fn callsign(me: &[u8]) -> Option<String> {
    let mut v: u64 = 0;
    for &b in &me[1..7] {
        v = (v << 8) | b as u64;
    }
    let mut s = String::with_capacity(8);
    for i in 0..8 {
        s.push(CS6[((v >> (42 - 6 * i)) & 0x3F) as usize] as char);
    }
    // Index 32 is the pad, which this alphabet renders '_', and '#' marks a
    // 6-bit code the identification alphabet does not assign. Both are stripped
    // from the end. A '#' left inside the string means the frame is corrupt,
    // and the callsign is discarded.
    let t = s.trim_end_matches(['_', '#']).to_string();
    if t.is_empty() || t.contains('#') {
        None
    } else {
        Some(t)
    }
}

// ---- the rest of the extended squitter ------------------------------------

/// Bits `first..first+n` of a message, numbered from 1 at the most
/// significant bit the way the standards number them.
fn bits(b: &[u8], first: usize, n: usize) -> u32 {
    let mut v = 0u32;
    for i in 0..n {
        let k = first - 1 + i;
        v = (v << 1) | ((b[k >> 3] >> (7 - (k & 7))) & 1) as u32;
    }
    v
}

/// What kind of aircraft is transmitting, from TC1-4: the type code picks the
/// set (4 is A, 1 is D) and the three CA bits the entry within it, so A3 is a
/// large aircraft and B6 an unmanned one. Entry 0 is "no information".
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Category {
    pub set: char,
    pub code: u8,
}

impl std::fmt::Display for Category {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        write!(f, "{}{}", self.set, self.code)
    }
}

pub fn category(me: &[u8]) -> Option<Category> {
    let set = match me[0] >> 3 {
        4 => 'A',
        3 => 'B',
        2 => 'C',
        1 => 'D',
        _ => return None,
    };
    let code = me[0] & 7;
    (code != 0).then_some(Category { set, code })
}

/// The 13-bit identity field of DF5/21 and TC28, as the four octal digits a
/// controller would read out, packed as a decimal number: 7700 is 7700. The
/// pulses arrive interleaved - C1 A1 C2 A2 C4 A4 X B1 D1 B2 D2 B4 D4.
pub fn squawk(id: u32) -> u16 {
    let p = |i: u32| ((id >> (12 - i)) & 1) as u16;
    let a = p(5) << 2 | p(3) << 1 | p(1);
    let b = p(11) << 2 | p(9) << 1 | p(7);
    let c = p(4) << 2 | p(2) << 1 | p(0);
    let d = p(12) << 2 | p(10) << 1 | p(8);
    a * 1000 + b * 100 + c * 10 + d
}

/// The 13-bit AC field of DF0/4/16/20: the 12-bit ADS-B form with an M bit
/// inserted at bit 6. `None` when unused, metric (M set), or illegal. Taking
/// the M bit out and reusing [`ac12`] keeps both forms on one Gillham path.
pub fn ac13(ac: u32) -> Option<i32> {
    if (ac >> 6) & 1 != 0 {
        return None;
    }
    ac12((((ac & 0x1F80) >> 1) | (ac & 0x3F)) as i32)
}

/// TC19 subtypes 3 and 4, sent when the aircraft has no ground velocity to
/// give: airspeed, and the magnetic heading it is flying.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Airspeed {
    pub heading: Option<f64>,
    /// Knots; `true_airspeed` says whether this is TAS or IAS.
    pub knots: Option<u16>,
    pub true_airspeed: bool,
}

pub fn airspeed(me: &[u8]) -> Option<Airspeed> {
    let st = me[0] & 7;
    if st != 3 && st != 4 {
        return None;
    }
    let heading = (bits(me, 14, 1) != 0).then(|| bits(me, 15, 10) as f64 * 360.0 / 1024.0);
    let raw = bits(me, 26, 10) as u16;
    let knots = (raw != 0).then(|| (raw - 1) * if st == 4 { 4 } else { 1 });
    Some(Airspeed {
        heading,
        knots,
        true_airspeed: bits(me, 25, 1) != 0,
    })
}

/// TC19, every subtype: GNSS height minus barometric altitude, in feet.
pub fn geo_minus_baro(me: &[u8]) -> Option<i32> {
    let raw = (me[6] & 0x7F) as i32;
    if raw == 0 || raw == 127 {
        return None;
    }
    Some((raw - 1) * 25 * if me[6] & 0x80 != 0 { -1 } else { 1 })
}

/// Autopilot modes from TC29, present when the aircraft says they are valid.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Modes {
    pub autopilot: bool,
    pub vnav: bool,
    pub alt_hold: bool,
    pub approach: bool,
    pub lnav: bool,
}

/// TC29 target state and status (DO-260B, subtype 1): what the crew has set.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TargetState {
    /// Selected altitude, feet, and whether it came from the FMS rather than
    /// the panel (MCP/FCU).
    pub altitude: Option<i32>,
    pub altitude_fms: bool,
    /// Barometric pressure setting, hPa.
    pub qnh: Option<f64>,
    pub heading: Option<f64>,
    pub modes: Option<Modes>,
    pub tcas: bool,
}

pub fn target_state(me: &[u8]) -> Option<TargetState> {
    // Subtype 0 is DO-260A's layout, which puts different fields here.
    if (me[0] >> 1) & 3 != 1 {
        return None;
    }
    let alt = bits(me, 10, 11) as i32;
    let qnh = bits(me, 21, 9);
    let flag = |b| bits(me, b, 1) != 0;
    Some(TargetState {
        altitude: (alt != 0).then(|| (alt - 1) * 32),
        altitude_fms: flag(9),
        qnh: (qnh != 0).then(|| 800.0 + (qnh - 1) as f64 * 0.8),
        heading: flag(30).then(|| bits(me, 31, 9) as f64 * 360.0 / 512.0),
        modes: flag(47).then(|| Modes {
            autopilot: flag(48),
            vnav: flag(49),
            alt_hold: flag(50),
            approach: flag(52),
            lnav: flag(54),
        }),
        tcas: flag(53),
    })
}

// ---- tracking -------------------------------------------------------------

/// One half of a CPR pair, with the frame it came from so a fix can say
/// which messages it rests on.
#[derive(Clone, Copy, Debug)]
pub(crate) struct CprHalf {
    lat: u32,
    lon: u32,
    ms: u32,
    surface: bool,
    frame: u64,
    /// Bits the decoder repaired in it, and the frame itself - kept so a
    /// refused position can be traced to what it was decoded from.
    corrected: u8,
    raw: [u8; 14],
}

/// A position refused for being out of reach of the published one, with the
/// messages it was decoded from.
#[derive(Clone, Copy, Debug)]
pub struct Refusal {
    pub icao: u32,
    /// Where it decoded to.
    pub lat: f64,
    pub lon: f64,
    /// The published position it was judged against, and how far and how
    /// long before that was.
    pub from: (f64, f64),
    pub metres: f64,
    pub after_ms: u32,
    /// From an even/odd pair, or from this one message decoded against the
    /// published position.
    pub pair: bool,
    /// Bits repaired and the frame: the message that produced it, and for a
    /// pair the other half.
    pub this: (u8, [u8; 14]),
    pub other: Option<(u8, [u8; 14])>,
}

/// A decoded position and the pair of messages it came from.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Fix {
    lat: f64,
    lon: f64,
    ms: u32,
    from: (u64, u64),
    /// Whether any message under it had bits repaired, and which grid its
    /// latitude was taken from - the parity of the newer half, whose zone
    /// width the fix is a multiple of. Together they say which later
    /// messages are entitled to agree with it.
    repaired: bool,
    grid_odd: bool,
}

/// Great-circle distance in metres.
fn distance(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let (p1, p2) = (lat1.to_radians(), lat2.to_radians());
    let dp = p2 - p1;
    let dl = (lon2 - lon1).to_radians();
    let h = (dp / 2.0).sin().powi(2) + p1.cos() * p2.cos() * (dl / 2.0).sin().powi(2);
    2.0 * 6_371_000.0 * h.sqrt().min(1.0).asin()
}

/// Could an aircraft have got from one place to the other in the time
/// between? Loose on purpose - 2 km for quantisation plus Mach 2 - because the
/// fixes it is there to catch land hundreds of kilometres away.
fn reachable(a: (f64, f64, u32), b: (f64, f64, u32)) -> bool {
    let dt = a.2.abs_diff(b.2) as f64 / 1000.0;
    distance(a.0, a.1, b.0, b.1) <= 2_000.0 + 700.0 * dt
}

/// What became of a decoded position.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Placed {
    Accepted,
    /// Held as a candidate until something independent agrees with it.
    Pending,
    /// Out of reach of a recent confirmed position: where it decoded to,
    /// whether that was from a pair, and the position it was judged against,
    /// which is passed back because a run of refusals drops that position.
    Implausible {
        lat: f64,
        lon: f64,
        pair: bool,
        from: (f64, f64, u32),
    },
}

/// What is currently known about one aircraft.
#[derive(Clone, Debug, Default)]
pub struct Aircraft {
    pub icao: u32,
    pub callsign: Option<String>,
    pub category: Option<Category>,
    /// Barometric altitude, feet.
    pub alt: Option<i32>,
    /// Height from GNSS, from TC20-22 positions, feet.
    pub gnss_alt: Option<i32>,
    /// GNSS height minus barometric altitude, from velocity messages.
    pub geo_minus_baro: Option<i32>,
    /// A position is only published once two fixes decoded from different
    /// messages agree, or one agrees with a recent published position. A
    /// single corrupt CPR pair can otherwise decode to an arbitrary position.
    pub lat: Option<f64>,
    pub lon: Option<f64>,
    /// Ground speed (knots), track and vertical rate (ft/min).
    pub speed: Option<f64>,
    pub heading: Option<f64>,
    pub vrate: Option<i32>,
    /// From airspeed messages, which aircraft send instead of ground speed.
    pub airspeed: Option<Airspeed>,
    /// Mode A code, from surveillance replies or TC28.
    pub squawk: Option<u16>,
    /// TC28 emergency state: 0 none, 1 general, 2 medical, 3 minimum fuel,
    /// 4 no communications, 5 unlawful interference, 6 downed aircraft.
    pub emergency: Option<u8>,
    pub target: Option<TargetState>,
    /// ADS-B version from TC31: 0, 1 (DO-260A) or 2 (DO-260B).
    pub adsb_version: Option<u8>,
    pub on_ground: bool,
    /// Frames seen from this address.
    pub frames: u64,
    /// Receiver milliseconds at the last frame, and at the last position fix
    /// (set whenever `lat` and `lon` are).
    pub t_seen: u32,
    pub t_pos: Option<u32>,
    /// When `alt` was last set, for judging a surveillance reply against it.
    pub(crate) t_alt: Option<u32>,
    /// The two halves of a CPR pair. Surface and airborne use different zone
    /// sizes, so a pair made of one of each decodes to a wrong position.
    pub(crate) cpr_even: Option<CprHalf>,
    pub(crate) cpr_odd: Option<CprHalf>,
    /// A fix waiting for a second one to vouch for it.
    pub(crate) candidate: Option<Fix>,
    /// Consecutive refusals against the published position, and the time of
    /// the last one. Three within 30 s drop the published position.
    pub(crate) refused: u8,
    pub(crate) t_refused: Option<u32>,
}

impl Aircraft {
    /// An aircraft nothing is known about yet. The bookkeeping fields (the
    /// CPR halves, the candidate fix, the refusal counters) are private to the
    /// tracker, so records are created with this function.
    pub fn new(icao: u32) -> Aircraft {
        Aircraft {
            icao,
            ..Default::default()
        }
    }

    /// The published position, if it is recent enough to judge a new fix by.
    fn recent_position(&self, now: u32) -> Option<(f64, f64, u32)> {
        match (self.lat, self.lon, self.t_pos) {
            (Some(lat), Some(lon), Some(t)) if now.wrapping_sub(t) <= 60_000 => Some((lat, lon, t)),
            _ => None,
        }
    }

    fn publish(&mut self, f: Fix) {
        self.lat = Some(f.lat);
        self.lon = Some(f.lon);
        self.t_pos = Some(f.ms);
        self.candidate = None;
        self.refused = 0;
        self.t_refused = None;
    }

    /// Forget where the aircraft was, after a run of refusals said the
    /// published position could not be squared with what is arriving.
    fn unplace(&mut self) {
        self.lat = None;
        self.lon = None;
        self.t_pos = None;
        self.candidate = None;
        self.refused = 0;
        self.t_refused = None;
    }

    /// Decide what one position message publishes, given the global fix it
    /// completed, if any.
    ///
    /// Against a recent published position, anything within reach is enough.
    /// Without one, the fix waits as a candidate until a later message
    /// agrees with it: either a global fix that shares neither half with it,
    /// or the message's own local decode, relative to the candidate. The
    /// local decode is only near the candidate if the candidate is right (or
    /// off by a whole number of zones), so one corrupt message cannot vouch
    /// for itself - but a weak aircraft heard once every ten seconds still
    /// gets placed on its third message.
    ///
    /// A repaired bit in a CPR latitude shifts the zone index by a whole
    /// step, and a later message of the same parity then decodes locally into
    /// the same wrong zone and agrees with it. The even and odd grids are 6°
    /// and 6.101° wide and share no common multiple, so a message of the other
    /// parity does not agree. A repaired candidate is therefore confirmed only
    /// by a message of the other parity, or by a disjoint pair.
    fn place(&mut self, global: Option<Fix>, m: CprHalf, odd: bool) -> Option<Placed> {
        let at = |f: &Fix| (f.lat, f.lon, f.ms);
        let local = |r: (f64, f64, u32)| -> Option<Fix> {
            let (lat, lon) = cpr_local(m.lat, m.lon, odd, m.surface, r.0, r.1)?;
            reachable(r, (lat, lon, m.ms)).then_some(Fix {
                lat,
                lon,
                ms: m.ms,
                from: (m.frame, m.frame),
                repaired: m.corrected > 0,
                grid_odd: odd,
            })
        };

        let recent = self.recent_position(m.ms);
        if let Some(r) = recent {
            if let Some(f) = global.filter(|g| reachable(r, at(g))).or_else(|| local(r)) {
                self.publish(f);
                return Some(Placed::Accepted);
            }
        }
        // No recent position, or nothing here that follows from it. Two
        // agreeing fixes also replace a published position that was wrong.
        if let Some(c) = self.candidate.filter(|c| m.ms.wrapping_sub(c.ms) <= 60_000) {
            let near = |g: &Fix| reachable(at(&c), at(g));
            let disjoint = |g: &Fix| g.from.0 != c.from.0 && g.from.1 != c.from.1;
            let witness = global.filter(|g| disjoint(g) && near(g)).or_else(|| {
                if c.repaired && odd == c.grid_odd {
                    None
                } else {
                    local(at(&c))
                }
            });
            if let Some(w) = witness {
                self.publish(global.filter(near).unwrap_or(w));
                return Some(Placed::Accepted);
            }
        }
        if global.is_some() {
            self.candidate = global;
        }
        match recent {
            // What was refused: the pair's fix if there was one, else this
            // message decoded against the published position.
            Some(r) => global
                .map(|g| (g.lat, g.lon, true))
                .or_else(|| {
                    cpr_local(m.lat, m.lon, odd, m.surface, r.0, r.1).map(|(a, b)| (a, b, false))
                })
                .map(|(lat, lon, pair)| {
                    // Three refusals within 30 s mean the published position
                    // is wrong. Drop it so the aircraft can be placed again.
                    let run = self
                        .t_refused
                        .is_some_and(|t| m.ms.wrapping_sub(t) <= 30_000);
                    self.refused = if run { self.refused + 1 } else { 1 };
                    self.t_refused = Some(m.ms);
                    if self.refused >= 3 {
                        self.unplace();
                    }
                    Placed::Implausible {
                        lat,
                        lon,
                        pair,
                        from: r,
                    }
                }),
            None => global.map(|_| Placed::Pending),
        }
    }
}

/// What a single frame changed, so a caller can emit only what is new.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Update {
    /// The address only: a DF11 all-call, a short frame, or an extended
    /// squitter whose type code is not decoded.
    Address,
    Identification,
    SurfacePosition,
    AirbornePosition,
    Velocity,
    /// TC28: emergency state and squawk.
    Status,
    /// TC29: the autopilot's targets.
    TargetState,
    /// TC31: capabilities and ADS-B version.
    OperationalStatus,
    /// DF0/4/16/20: an altitude reply.
    SurveillanceAltitude,
    /// DF5/21: a squawk reply.
    SurveillanceIdentity,
}

/// Aircraft seen recently, keyed by address.
pub struct Tracker {
    pub table: HashMap<u32, Aircraft>,
    /// Positions refused for being out of reach of the last one.
    pub implausible: u64,
    /// Frames `update_raw` refused because their parity did not check out.
    pub bad_parity: u64,
    /// The position the last `update` refused, if it refused one. Replaced on
    /// every refusal, so a caller that wants them all takes it after each call.
    pub refusal: Option<Refusal>,
    crc: Crc,
    ap: ApMap,
}

impl Default for Tracker {
    fn default() -> Self {
        let crc = Crc::new();
        let ap = ApMap::new(&crc);
        Tracker {
            table: HashMap::new(),
            implausible: 0,
            bad_parity: 0,
            refusal: None,
            crc,
            ap,
        }
    }
}

impl Tracker {
    pub fn new() -> Self {
        Self::default()
    }
    pub fn len(&self) -> usize {
        self.table.len()
    }
    pub fn is_empty(&self) -> bool {
        self.table.is_empty()
    }

    /// Drop aircraft not heard from within `max_age` milliseconds.
    pub fn expire(&mut self, now_ms: u32, max_age: u32) -> usize {
        let before = self.table.len();
        self.table
            .retain(|_, a| now_ms.wrapping_sub(a.t_seen) <= max_age);
        before - self.table.len()
    }

    /// What the parity field holds once the CRC is taken out: zero for a clean
    /// DF17/18, the interrogator code for DF11, the address for the formats
    /// that overlay it.
    fn overlay(&self, b: &[u8]) -> u32 {
        self.ap.address(self.crc.crc24(b))
    }

    /// Fold in a frame from somewhere other than this crate's decoder - a Beast
    /// feed, say - which has to prove its parity first. The decoder only emits
    /// frames that already have.
    pub fn update_raw(&mut self, bytes: &[u8], ms: u32) -> Option<(u32, Update)> {
        let frame = crate::Frame::new(bytes, ms)?;
        let ok = match frame.df() {
            17 | 18 => self.overlay(bytes) == 0,
            11 => self.overlay(bytes) & !0x7F == 0,
            // The address-overlaid formats are checked by `update` itself,
            // against the aircraft already known.
            _ => true,
        };
        if !ok {
            self.bad_parity += 1;
            return None;
        }
        self.update(&frame)
    }

    /// Fold one decoded frame in. Returns the address it was for and what it
    /// contributed, if anything.
    pub fn update(&mut self, frame: &crate::Frame) -> Option<(u32, Update)> {
        let df = frame.df();
        let b = frame.as_bytes();
        if matches!(df, 0 | 4 | 5 | 16 | 20 | 21) {
            let icao = self.overlay(b);
            return self.surveillance(icao, df, b, frame.ms).map(|u| (icao, u));
        }
        if df != 11 && df != 17 && df != 18 {
            return None;
        }
        let icao = frame.icao();
        let ms = frame.ms;

        let a = self
            .table
            .entry(icao)
            .or_insert_with(|| Aircraft::new(icao));
        a.frames += 1;
        a.t_seen = ms;

        if df == 11 || b.len() != 14 {
            return Some((icao, Update::Address));
        }
        let tc = b[4] >> 3;
        let me = &b[4..11];

        let what = match tc {
            1..=4 => {
                if let Some(cs) = callsign(me) {
                    a.callsign = Some(cs);
                }
                a.category = category(me);
                Update::Identification
            }
            5..=18 | 20..=22 => {
                let surface = (5..=8).contains(&tc);
                a.on_ground = surface;
                let ac = bits(me, 9, 12);
                if (9..=18).contains(&tc) {
                    if let Some(alt) = ac12(ac as i32) {
                        a.alt = Some(alt);
                        a.t_alt = Some(ms);
                    }
                } else if tc >= 20 && ac != 0 {
                    a.gnss_alt = Some((ac as f64 * 3.28084).round() as i32);
                }
                let odd = bits(me, 22, 1) != 0;
                let latc = bits(b, 55, 17);
                let lonc = bits(b, 72, 17);
                let mut raw = [0u8; 14];
                raw.copy_from_slice(b);
                let half = CprHalf {
                    lat: latc,
                    lon: lonc,
                    ms,
                    surface,
                    frame: a.frames,
                    corrected: frame.corrected,
                    raw,
                };
                if odd {
                    a.cpr_odd = Some(half);
                } else {
                    a.cpr_even = Some(half);
                }

                // A global fix needs both halves, close in time, and both
                // airborne; surface pairs need a reference this does not have.
                let mut global = None;
                if let (Some(e), Some(o)) = (a.cpr_even, a.cpr_odd) {
                    if e.ms.abs_diff(o.ms) <= 10_000 && !e.surface && !o.surface {
                        global =
                            cpr_global(e.lat, e.lon, o.lat, o.lon, odd).map(|(lat, lon)| Fix {
                                lat,
                                lon,
                                ms,
                                from: (e.frame, o.frame),
                                repaired: e.corrected > 0 || o.corrected > 0,
                                grid_odd: odd,
                            });
                    }
                }
                let placed = a.place(global, half, odd);
                if let Some(Placed::Implausible {
                    lat,
                    lon,
                    pair,
                    from,
                }) = placed
                {
                    self.implausible += 1;
                    let other = if odd { a.cpr_even } else { a.cpr_odd };
                    self.refusal = Some(Refusal {
                        icao,
                        lat,
                        lon,
                        from: (from.0, from.1),
                        metres: distance(from.0, from.1, lat, lon),
                        after_ms: ms.wrapping_sub(from.2),
                        pair,
                        this: (half.corrected, half.raw),
                        other: other.filter(|_| pair).map(|o| (o.corrected, o.raw)),
                    });
                }
                if surface {
                    Update::SurfacePosition
                } else {
                    Update::AirbornePosition
                }
            }
            19 => {
                if let Some((sp, hd, vr)) = velocity(me) {
                    // Only what this message said: a rate it did not report
                    // must not carry over from an earlier one.
                    a.speed = Some(sp);
                    a.heading = Some(hd);
                    a.vrate = vr;
                }
                if let Some(s) = airspeed(me) {
                    a.airspeed = Some(s);
                }
                a.geo_minus_baro = geo_minus_baro(me);
                Update::Velocity
            }
            28 if me[0] & 7 == 1 => {
                a.emergency = Some(bits(me, 9, 3) as u8);
                a.squawk = Some(squawk(bits(me, 12, 13)));
                Update::Status
            }
            29 => {
                a.target = target_state(me);
                Update::TargetState
            }
            31 => {
                a.adsb_version = Some(bits(me, 41, 3) as u8);
                Update::OperationalStatus
            }
            _ => Update::Address,
        };
        Some((icao, what))
    }

    /// The replies whose parity is overlaid with the address. Nothing in them
    /// says who sent them except that address, so only one belonging to an
    /// aircraft already heard by other means is believed.
    fn surveillance(&mut self, icao: u32, df: u8, b: &[u8], ms: u32) -> Option<Update> {
        let a = self.table.get_mut(&icao)?;
        // Flight status 6 and 7 are unassigned; seeing one means the frame is
        // corrupt and only passed because the overlaid check cannot tell.
        if matches!(df, 4 | 5 | 20 | 21) {
            match bits(b, 6, 3) {
                6 | 7 => return None,
                fs => a.on_ground = fs == 1 || fs == 3,
            }
        }
        a.frames += 1;
        a.t_seen = ms;
        let field = bits(b, 20, 13);
        if matches!(df, 5 | 21) {
            a.squawk = Some(squawk(field));
            return Some(Update::SurveillanceIdentity);
        }
        let alt = ac13(field).filter(|v| (-1000..=60_000).contains(v))?;
        // A reply that passed the overlaid check by luck can carry any
        // altitude. Hold it to what the aircraft could have climbed since the
        // last one: 500 ft of slack plus 5000 ft a minute.
        if let (Some(prev), Some(t)) = (a.alt, a.t_alt) {
            let dt = ms.wrapping_sub(t);
            if dt <= 30_000 && alt.abs_diff(prev) as u64 > 500 + dt as u64 * 5000 / 60_000 {
                return None;
            }
        }
        a.alt = Some(alt);
        a.t_alt = Some(ms);
        Some(Update::SurveillanceAltitude)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn altitude_q_bit_form() {
        // Q=1 means 25 ft steps offset by 1000, with the twelve bits split
        // either side of the Q bit: n = ((ac & 0x0FE0) >> 1) | (ac & 0x000F).
        assert_eq!(ac12(0x010), Some(-1000), "n = 0 is the bottom of the scale");
        assert_eq!(ac12(0x011), Some(-975), "n = 1 is one 25 ft step up");
        assert_eq!(ac12(0xB50), Some(35_000), "a cruising altitude round-trips");

        // Zero is not an altitude of zero, it means the field is unused.
        assert_eq!(ac12(0), None);
    }

    /// NL is the table the whole CPR scheme rests on; these are its fixed points.
    #[test]
    fn longitude_zones() {
        assert_eq!(nl(0.0), 59);
        assert_eq!(nl(87.1), 2);
        assert_eq!(nl(90.0), 1);
        assert_eq!(nl(-90.0), 1);
        assert_eq!(nl(10.0), nl(-10.0), "symmetric about the equator");
    }

    /// A real airborne even/odd pair from 4009DA, with the CPR fields taken
    /// from its frames (8d4009da5833318e2bd82af8c6f5 even,
    /// 8d4009da583324fef1cbc5c7449d odd). It decodes to 50.33265 N, 8.73717 E.
    #[test]
    fn global_cpr_pair_decodes() {
        let (lat, lon) =
            cpr_global(50965, 120874, 32632, 117701, true).expect("pair should decode");
        assert!((lat - 50.33265).abs() < 1e-4, "latitude {lat}");
        assert!((lon - 8.73717).abs() < 1e-4, "longitude {lon}");
    }

    /// The pad character renders '_' in this alphabet, so trailing pads are
    /// trimmed as '_'.
    #[test]
    fn callsign_strips_the_pad() {
        let mut v: u64 = 0;
        // B E L 4 Q A then two pads. Letters are 1..26, digits 48..57, pad 32.
        for c in [2u64, 5, 12, 52, 17, 1, 32, 32] {
            v = (v << 6) | c;
        }
        let mut me = [0u8; 7];
        for i in 0..6 {
            me[1 + i] = (v >> (40 - 8 * i)) as u8;
        }
        let cs = callsign(&me).expect("should decode");
        assert!(
            !cs.ends_with('_'),
            "trailing pad must be stripped, got {cs}"
        );
        assert!(!cs.contains('#'));
        assert_eq!(cs, "BEL4QA", "six real characters survive");
    }

    #[test]
    // The zero shifts are the two '#' characters under test; spelling out every
    // slot keeps the packed word readable as the callsign it encodes.
    #[allow(clippy::identity_op)]
    fn callsign_rejects_interior_padding() {
        // 'BAD##DR': a '#' inside the string is not a legal callsign.
        let mut me = [0u8; 7];
        me[0] = 1 << 3;
        let packed: u64 =
            (2 << 42) | (1 << 36) | (4 << 30) | (0 << 24) | (0 << 18) | (4 << 12) | (18 << 6) | 32;
        for i in 0..6 {
            me[1 + i] = (packed >> (40 - 8 * i)) as u8;
        }
        assert_eq!(
            callsign(&me),
            None,
            "an interior '#' makes the whole callsign invalid"
        );
    }

    /// A rate field of 0 means no rate; 1 means level flight.
    #[test]
    fn vertical_rate_absent_is_not_level() {
        // TC19 subtype 1, 9 kt east, 9 kt north, then the rate field.
        let me = |vr: u8| {
            [
                19 << 3 | 1,
                0,
                10,
                1,
                (2 << 5) | (vr >> 6),
                (vr & 0x3F) << 2,
                0,
            ]
        };
        let (_, _, none) = velocity(&me(0)).expect("a velocity");
        let (_, _, level) = velocity(&me(1)).expect("a velocity");
        let (_, _, climb) = velocity(&me(2)).expect("a velocity");
        assert_eq!(none, None);
        assert_eq!(level, Some(0));
        assert_eq!(climb, Some(64));
    }

    fn hex(s: &str) -> Vec<u8> {
        (0..s.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&s[i..i + 2], 16).unwrap())
            .collect()
    }

    /// Set the parity so the frame checks out: zero for an extended squitter,
    /// the address overlaid for a surveillance reply.
    fn seal(b: &mut [u8], overlay: Option<u32>) {
        let n = b.len();
        let p = Crc::new().crc24(&b[..n - 3]) ^ overlay.unwrap_or(0);
        b[n - 3..].copy_from_slice(&[(p >> 16) as u8, (p >> 8) as u8, p as u8]);
    }

    // The 4009DA pair from global_cpr_pair_decodes, as whole frames.
    const EVEN: &str = "8d4009da5833318e2bd82af8c6f5";
    const ODD: &str = "8d4009da583324fef1cbc5c7449d";

    /// One pair is not enough to publish a position: it could be one corrupt
    /// message. The next message, decoded against it, confirms it.
    #[test]
    fn a_position_waits_for_an_independent_fix() {
        let mut t = Tracker::new();
        let pos = |t: &Tracker| t.table[&0x4009DA].lat;
        t.update_raw(&hex(EVEN), 0).unwrap();
        t.update_raw(&hex(ODD), 500).unwrap();
        assert_eq!(pos(&t), None, "the first pair is only a candidate");
        t.update_raw(&hex(EVEN), 1_000).unwrap();
        let lat = pos(&t).expect("a third message agrees");
        // Published from the even message's local decode, which sits on the
        // even grid - about 40 m from the pair's global result.
        assert!((lat - 50.33265).abs() < 1e-3, "latitude {lat}");
    }

    /// The same frame as the decoder hands over when it has had to repair it.
    fn repaired(b: &[u8], ms: u32) -> crate::Frame {
        let mut f = crate::Frame::new(b, ms).expect("a well-formed frame");
        f.corrected = 2;
        f
    }

    /// Replace the 17-bit CPR latitude of an extended squitter and make the
    /// parity good again.
    fn set_cpr_lat(b: &mut [u8], lat: u32) {
        for i in 0..17 {
            let k = 54 + i;
            let bit = (lat >> (16 - i)) & 1;
            b[k >> 3] = (b[k >> 3] & !(0x80 >> (k & 7))) | ((bit as u8) << (7 - (k & 7)));
        }
        seal(b, None);
    }

    /// The even half with a step added to its CPR latitude, parity made good
    /// again: a frame that passes parity but decodes one latitude zone away.
    fn even_a_zone_out() -> Vec<u8> {
        let mut b = hex(EVEN);
        let lat = (bits(&b, 55, 17) + 2048) & 0x1FFFF;
        set_cpr_lat(&mut b, lat);
        b
    }

    /// A repair moves the zone index a whole step, and the next message of the
    /// grid the fix was taken from decodes into that same wrong zone and
    /// agrees with it, so a repaired candidate is not confirmed from its own
    /// grid.
    #[test]
    fn a_repaired_frame_is_not_allowed_to_vouch_for_itself() {
        let mut t = Tracker::new();
        t.update(&repaired(&even_a_zone_out(), 0)).unwrap();
        t.update_raw(&hex(ODD), 500).unwrap();

        let c = t.table[&0x4009DA]
            .candidate
            .expect("the pair still decodes");
        assert!(
            distance(c.lat, c.lon, 50.33265, 8.73717) > 500_000.0,
            "a zone out: {} {}",
            c.lat,
            c.lon
        );
        let odd = hex(ODD);
        let (lat, lon) = cpr_local(
            bits(&odd, 55, 17),
            bits(&odd, 72, 17),
            true,
            false,
            c.lat,
            c.lon,
        )
        .expect("the next odd message decodes against it");
        assert!(
            reachable((c.lat, c.lon, c.ms), (lat, lon, 1_000)),
            "and lands on the wrong zone, agreeing with it"
        );

        t.update_raw(&hex(ODD), 1_000).unwrap();
        assert_eq!(
            t.table[&0x4009DA].lat, None,
            "which must not be enough to publish it"
        );

        // Clean frames put it where it belongs.
        for (i, f) in [EVEN, ODD, EVEN].iter().enumerate() {
            t.update_raw(&hex(f), 1_500 + i as u32 * 500).unwrap();
        }
        let lat = t.table[&0x4009DA]
            .lat
            .expect("clean frames still get it placed");
        assert!((lat - 50.33265).abs() < 1e-3, "latitude {lat}");
    }

    /// Three refusals in a row drop the published position, and the aircraft
    /// is placed again without waiting for the position to expire.
    #[test]
    fn a_run_of_refusals_drops_the_published_position() {
        let mut t = Tracker::new();
        t.update_raw(&hex(EVEN), 0).unwrap();
        // Half a degree from where these frames decode: far enough that every
        // fix from them is out of reach of it.
        let a = t.table.get_mut(&0x4009DA).unwrap();
        a.lat = Some(50.83265);
        a.lon = Some(8.73717);
        a.t_pos = Some(0);
        // Repaired and all of one grid, so none of them can confirm another
        // and correct it.
        for (i, f) in [ODD, ODD, ODD].iter().enumerate() {
            t.update(&repaired(&hex(f), 500 + i as u32 * 500)).unwrap();
        }
        assert_eq!(t.implausible, 3, "each one is refused, and logged");
        let a = &t.table[&0x4009DA];
        assert!(
            a.lat.is_none() && a.candidate.is_none(),
            "the position is dropped"
        );

        for (i, f) in [EVEN, ODD, EVEN].iter().enumerate() {
            t.update_raw(&hex(f), 3_000 + i as u32 * 500).unwrap();
        }
        let lat = t.table[&0x4009DA]
            .lat
            .expect("and it is placed again straight away");
        assert!((lat - 50.33265).abs() < 1e-3, "latitude {lat}");
    }

    /// The odd half with its CPR latitude changed, parity made good again: a
    /// message that decodes cleanly to the wrong place.
    fn corrupt_odd() -> Vec<u8> {
        let mut bad = hex(ODD);
        let lat = bits(&bad, 55, 17) ^ 0xA000;
        set_cpr_lat(&mut bad, lat);
        bad
    }

    /// A candidate built on a corrupt message never gets published: nothing
    /// decoded from the good messages around it lands near it.
    #[test]
    fn a_corrupt_pair_never_vouches_for_itself() {
        let mut t = Tracker::new();
        let seq = [hex(EVEN), corrupt_odd(), hex(EVEN), hex(ODD), hex(EVEN)];
        for (i, f) in seq.iter().enumerate() {
            t.update_raw(f, i as u32 * 500).unwrap();
            let a = &t.table[&0x4009DA];
            if i == 1 {
                let c = a.candidate.expect("the corrupt pair still decodes");
                assert!(
                    distance(c.lat, c.lon, 50.33265, 8.73717) > 100_000.0,
                    "and lands far away: {} {}",
                    c.lat,
                    c.lon
                );
            }
            if let Some(lat) = a.lat {
                assert!(
                    (lat - 50.33265).abs() < 1e-3,
                    "after message {i}: published {lat}"
                );
            }
        }
        assert!(
            t.table[&0x4009DA].lat.is_some(),
            "the good messages do get it placed"
        );
    }

    /// Once there is a position, a fix hundreds of kilometres from it is
    /// refused instead of teleporting the aircraft.
    #[test]
    fn a_fix_out_of_reach_is_refused() {
        let mut t = Tracker::new();
        for (i, f) in [EVEN, ODD, EVEN, ODD].iter().enumerate() {
            t.update_raw(&hex(f), i as u32 * 500).unwrap();
        }
        let before = (t.table[&0x4009DA].lat, t.table[&0x4009DA].lon);

        t.update_raw(&corrupt_odd(), 2_500)
            .expect("parity is fine, only the position is wrong");
        let a = &t.table[&0x4009DA];
        assert_eq!((a.lat, a.lon), before, "the aircraft stays where it was");
        assert_eq!(t.implausible, 1);
        let r = t.refusal.expect("and the refusal is kept for the log");
        assert_eq!((r.icao, r.pair), (0x4009DA, true));
        assert!(r.metres > 100_000.0, "{} m", r.metres);
        assert_eq!(
            r.this.1.to_vec(),
            corrupt_odd(),
            "naming the message that did it"
        );
        assert_eq!(
            r.other.map(|o| o.1.to_vec()),
            Some(hex(EVEN)),
            "and its other half"
        );
    }

    #[test]
    fn foreign_frames_must_pass_parity() {
        let mut t = Tracker::new();
        let mut f = hex(EVEN);
        f[6] ^= 0x10;
        assert_eq!(t.update_raw(&f, 0), None);
        assert_eq!(t.bad_parity, 1);
        assert!(t.is_empty(), "nothing is learned from it");
        assert_eq!(
            t.update_raw(&hex(EVEN)[..7], 0),
            None,
            "a DF17 is 14 bytes, not 7"
        );
    }

    /// A surveillance reply names its sender only through the parity, so it
    /// counts only for an aircraft already heard from.
    #[test]
    fn surveillance_replies_need_a_known_sender() {
        let mut t = Tracker::new();
        // DF5, flight status 0, identity 0x0808 (squawk 1200), bits 20-32.
        let id: u32 = 0x0808;
        let mut df5 = vec![5 << 3, 0, ((id >> 8) & 0x1F) as u8, id as u8, 0, 0, 0];
        seal(&mut df5, Some(0x4009DA));
        assert_eq!(
            t.update_raw(&df5, 0),
            None,
            "an address nothing else has named"
        );

        t.update_raw(&hex(EVEN), 0).unwrap();
        assert_eq!(
            t.update_raw(&df5, 100),
            Some((0x4009DA, Update::SurveillanceIdentity))
        );
        assert_eq!(t.table[&0x4009DA].squawk, Some(1200));
    }

    #[test]
    fn squawk_digits() {
        assert_eq!(squawk(0x0808), 1200);
        assert_eq!(squawk(0), 0);
        assert_eq!(squawk(0x1FFF & !0x40), 7777, "every pulse but X");
    }

    /// DF4/20 altitude is the 12-bit ADS-B code with an M bit spliced in.
    #[test]
    fn altitude_thirteen_bit_form() {
        let n = (35_000 + 1000) / 25u32;
        let ac = ((n >> 5) << 7) | (((n >> 4) & 1) << 5) | (1 << 4) | (n & 0xF);
        assert_eq!(ac13(ac), Some(35_000));
        assert_eq!(ac13(ac | 1 << 6), None, "metric altitudes are not decoded");
        assert_eq!(ac13(0), None);
    }

    /// 100 ft Gillham altitudes, as 13-bit fields with M and Q clear.
    #[test]
    fn altitude_gillham_form() {
        for (field, alt) in [
            (0x0400, -1000),
            (0x1400, -900),
            (0x040A, 0),
            (0x140A, 100),
            (0x1028, 1200),
            (0x0420, 2500),
            (0x06A2, 10_000),
            (0x0CA1, 35_000),
            (0x1AAB, 41_300),
            (0x0104, 126_700),
        ] {
            assert_eq!(ac13(field), Some(alt), "field {field:#06x}");
        }
        // No C pulses, and C1 C2 C4 all set, are never used.
        assert_eq!(ac13(0x0002), None, "B4 alone");
        assert_eq!(ac13(0x1500 | 0x040A), None, "every C pulse");

        // Every altitude the code can carry, encoded and decoded again.
        let gray = |n: u32| n ^ (n >> 1);
        for alt in (-1000..=126_700).step_by(100) {
            let n500 = ((alt + 1200) / 500) as u32;
            let mut n100 = ((alt + 1300) / 100) as u32 - 5 * n500;
            if n500 % 2 == 1 {
                n100 = 6 - n100;
            }
            let c = if n100 == 5 { 0b100 } else { gray(n100) };
            let g = gray(n500);
            let b = |v: u32, n: u32| (v >> n) & 1;
            // D2 D4 A1 A2 A4 B1 B2 B4 from the top of the 500 ft code.
            let (d2, d4, a1, a2, a4, b1, b2, b4) = (
                b(g, 7),
                b(g, 6),
                b(g, 5),
                b(g, 4),
                b(g, 3),
                b(g, 2),
                b(g, 1),
                b(g, 0),
            );
            let ac = b(c, 2) << 11
                | a1 << 10
                | b(c, 1) << 9
                | a2 << 8
                | b(c, 0) << 7
                | a4 << 6
                | b1 << 5
                | b2 << 3
                | d2 << 2
                | b4 << 1
                | d4;
            assert_eq!(ac12(ac as i32), Some(alt), "ac {ac:#05x}");
        }
    }

    /// A DF17 from 4009DA carrying `me`, with good parity.
    fn es(me: [u8; 7]) -> Vec<u8> {
        let mut b = vec![0x8D, 0x40, 0x09, 0xDA];
        b.extend_from_slice(&me);
        b.extend_from_slice(&[0, 0, 0]);
        seal(&mut b, None);
        b
    }

    /// A tracker that has heard 4009DA and placed it from the EVEN/ODD pair.
    fn placed() -> Tracker {
        let mut t = Tracker::new();
        for (i, f) in [EVEN, ODD, EVEN].iter().enumerate() {
            t.update_raw(&hex(f), i as u32 * 500).unwrap();
        }
        assert!(t.table[&0x4009DA].lat.is_some());
        t
    }

    /// A ground speed message: speed, track, rate and the GNSS offset.
    #[test]
    fn a_velocity_message_sets_speed_track_and_rate() {
        let mut t = Tracker::new();
        let f = hex("8d485020994409940838175b284f");
        assert_eq!(t.update_raw(&f, 0), Some((0x485020, Update::Velocity)));
        let a = &t.table[&0x485020];
        assert!((a.speed.unwrap() - 159.2).abs() < 0.1, "{:?}", a.speed);
        assert!(
            (a.heading.unwrap() - 182.88).abs() < 0.01,
            "{:?}",
            a.heading
        );
        assert_eq!(a.vrate, Some(-832));
        assert_eq!(a.geo_minus_baro, Some(550));
        assert_eq!(
            a.airspeed, None,
            "a ground speed message carries no airspeed"
        );
    }

    /// An airspeed message: TAS and magnetic heading, and no ground speed.
    #[test]
    fn an_airspeed_message_sets_the_airspeed() {
        let mut t = Tracker::new();
        let f = hex("8da05f219b06b6af189400cbc33f");
        assert_eq!(t.update_raw(&f, 0), Some((0xA05F21, Update::Velocity)));
        let a = &t.table[&0xA05F21];
        let s = a.airspeed.expect("an airspeed");
        assert_eq!(s.knots, Some(375));
        assert!(s.true_airspeed);
        assert!(
            (s.heading.unwrap() - 243.98).abs() < 0.01,
            "{:?}",
            s.heading
        );
        assert_eq!((a.speed, a.heading, a.vrate), (None, None, None));
        assert_eq!(
            a.geo_minus_baro, None,
            "the offset field is zero: not given"
        );
    }

    /// Subtype 4 counts in 4 kt steps; a zero field is no speed and a clear
    /// status bit is no heading.
    #[test]
    fn airspeed_fields() {
        // Subtype 4, heading not given, IAS, raw speed 100.
        let me = [19 << 3 | 4, 0, 0, 100 >> 3, (100 & 7) << 5, 0, 0];
        assert_eq!(
            airspeed(&me),
            Some(Airspeed {
                heading: None,
                knots: Some(396),
                true_airspeed: false
            })
        );
        let me = [19 << 3 | 3, 0, 0, 0, 0, 0, 0];
        assert_eq!(
            airspeed(&me),
            Some(Airspeed {
                heading: None,
                knots: None,
                true_airspeed: false
            })
        );
        assert_eq!(
            airspeed(&[19 << 3 | 1, 0, 0, 0, 0, 0, 0]),
            None,
            "subtype 1 is ground speed"
        );

        assert_eq!(
            geo_minus_baro(&[0, 0, 0, 0, 0, 0, 0x7F]),
            None,
            "all ones is not given"
        );
        assert_eq!(
            geo_minus_baro(&[0, 0, 0, 0, 0, 0, 0x82]),
            Some(-25),
            "sign bit set"
        );
        assert_eq!(geo_minus_baro(&[0, 0, 0, 0, 0, 0, 0x01]), Some(0));
    }

    /// TC28 subtype 1 sets the emergency state and the squawk; other
    /// subtypes are not decoded.
    #[test]
    fn a_status_message_sets_emergency_and_squawk() {
        let id = (0..0x2000)
            .find(|&id| squawk(id) == 7700 && id & 0x40 == 0)
            .unwrap();
        let mut t = Tracker::new();
        let f = es([28 << 3 | 1, 1 << 5 | (id >> 8) as u8, id as u8, 0, 0, 0, 0]);
        assert_eq!(t.update_raw(&f, 0), Some((0x4009DA, Update::Status)));
        assert_eq!(t.table[&0x4009DA].emergency, Some(1));
        assert_eq!(t.table[&0x4009DA].squawk, Some(7700));

        let f = es([28 << 3 | 2, 0xFF, 0xFF, 0, 0, 0, 0]);
        assert_eq!(
            t.update_raw(&f, 10),
            Some((0x4009DA, Update::Address)),
            "a TCAS resolution advisory"
        );
        assert_eq!(
            t.table[&0x4009DA].squawk,
            Some(7700),
            "which changes nothing"
        );
    }

    /// A DO-260B target state message, and the DO-260A layout that is not
    /// decoded.
    #[test]
    fn a_target_state_message_sets_the_targets() {
        let mut t = Tracker::new();
        let f = hex("8da05629ea21485cbf3f8cadaeeb");
        assert_eq!(t.update_raw(&f, 0), Some((0xA05629, Update::TargetState)));
        let g = t.table[&0xA05629].target.expect("a target state");
        assert_eq!((g.altitude, g.altitude_fms), (Some(16_992), false));
        assert!((g.qnh.unwrap() - 1012.8).abs() < 1e-9, "{:?}", g.qnh);
        assert!((g.heading.unwrap() - 66.8).abs() < 0.01, "{:?}", g.heading);
        assert_eq!(
            g.modes,
            Some(Modes {
                autopilot: true,
                vnav: true,
                alt_hold: false,
                approach: false,
                lnav: true
            })
        );
        assert!(g.tcas);

        let f = es([29 << 3, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]);
        assert_eq!(t.update_raw(&f, 10), Some((0x4009DA, Update::TargetState)));
        assert_eq!(t.table[&0x4009DA].target, None, "subtype 0 is not read");

        // Nothing selected: every field absent.
        let g = target_state(&[29 << 3 | 2, 0, 0, 0, 0, 0, 0]).expect("subtype 1");
        assert_eq!(g, TargetState::default());
    }

    /// TC31 gives the ADS-B version.
    #[test]
    fn an_operational_status_message_sets_the_version() {
        let mut t = Tracker::new();
        let f = es([31 << 3, 0, 0, 0, 0, 2 << 5, 0]);
        assert_eq!(
            t.update_raw(&f, 0),
            Some((0x4009DA, Update::OperationalStatus))
        );
        assert_eq!(t.table[&0x4009DA].adsb_version, Some(2));
    }

    /// Type codes with nothing decoded count the frame and nothing else.
    #[test]
    fn an_unknown_type_code_is_only_an_address() {
        let mut t = Tracker::new();
        for tc in [0u8, 23, 24, 25, 26, 27, 30] {
            let f = es([tc << 3, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC]);
            assert_eq!(
                t.update_raw(&f, 0),
                Some((0x4009DA, Update::Address)),
                "type code {tc}"
            );
        }
        let a = &t.table[&0x4009DA];
        assert_eq!(a.frames, 7);
        assert_eq!((a.callsign.as_deref(), a.alt, a.lat), (None, None, None));
    }

    /// TC20-22 carry GNSS height in metres in place of barometric altitude.
    #[test]
    fn a_gnss_position_sets_the_gnss_height() {
        let mut t = Tracker::new();
        let mut f = hex(EVEN);
        f[4] = 20 << 3 | (f[4] & 7);
        f[5] = (1000 >> 4) as u8;
        f[6] = ((1000 & 0xF) << 4) as u8 | (f[6] & 0x0F);
        seal(&mut f, None);
        assert_eq!(
            t.update_raw(&f, 0),
            Some((0x4009DA, Update::AirbornePosition))
        );
        let a = &t.table[&0x4009DA];
        assert_eq!(a.gnss_alt, Some(3281), "1000 m in feet");
        assert_eq!(a.alt, None, "and no barometric altitude");

        // A zero height field is no height.
        let mut t = Tracker::new();
        f[5] = 0;
        f[6] &= 0x0F;
        seal(&mut f, None);
        t.update_raw(&f, 0).unwrap();
        assert_eq!(t.table[&0x4009DA].gnss_alt, None);
    }

    /// Replace the 17-bit CPR longitude of an extended squitter and make the
    /// parity good again.
    fn set_cpr_lon(b: &mut [u8], lon: u32) {
        for i in 0..17 {
            let k = 71 + i;
            let bit = (lon >> (16 - i)) & 1;
            b[k >> 3] = (b[k >> 3] & !(0x80 >> (k & 7))) | ((bit as u8) << (7 - (k & 7)));
        }
        seal(b, None);
    }

    /// A surface position message for 4009DA at about 50.3330 N 8.7380 E,
    /// even or odd.
    fn surface(odd: bool) -> Vec<u8> {
        // TC6, then movement, track and time left at zero.
        let mut b = es([6 << 3, 0, 0, 0, 0, 0, 0]);
        let (lat, lon) = if odd {
            (130_566, 77_638)
        } else {
            (72_789, 90_358)
        };
        if odd {
            b[6] |= 0x04;
        }
        set_cpr_lat(&mut b, lat);
        set_cpr_lon(&mut b, lon);
        b
    }

    /// A surface message puts the aircraft on the ground, and is decoded
    /// against the position already published.
    #[test]
    fn a_surface_position_is_decoded_against_the_last_one() {
        let mut t = placed();
        assert!(!t.table[&0x4009DA].on_ground);
        assert_eq!(
            t.update_raw(&surface(false), 2_000),
            Some((0x4009DA, Update::SurfacePosition))
        );
        let a = &t.table[&0x4009DA];
        assert!(a.on_ground);
        assert_eq!(a.t_pos, Some(2_000));
        let (lat, lon) = (a.lat.unwrap(), a.lon.unwrap());
        assert!(distance(lat, lon, 50.3330, 8.7380) < 10.0, "{lat} {lon}");

        assert_eq!(
            t.update_raw(&surface(true), 2_500),
            Some((0x4009DA, Update::SurfacePosition))
        );
        let a = &t.table[&0x4009DA];
        assert!(distance(a.lat.unwrap(), a.lon.unwrap(), 50.3331, 8.7381) < 10.0);
    }

    /// Without a position to decode against, a surface pair places nothing:
    /// surface pairs have no global decode.
    #[test]
    fn a_surface_pair_alone_places_nothing() {
        let mut t = Tracker::new();
        for (i, odd) in [false, true, false, true].into_iter().enumerate() {
            assert_eq!(
                t.update_raw(&surface(odd), i as u32 * 500).map(|u| u.1),
                Some(Update::SurfacePosition)
            );
        }
        let a = &t.table[&0x4009DA];
        assert!(a.on_ground);
        assert_eq!((a.lat, a.candidate.is_none()), (None, true));
        assert_eq!(t.implausible, 0);
    }

    /// A refusal decoded from this one message against the published position,
    /// because the pair it would complete is too far apart in time.
    #[test]
    fn a_single_message_can_be_refused() {
        let mut t = placed();
        t.update_raw(&corrupt_odd(), 20_000)
            .expect("parity is fine");
        assert_eq!(t.implausible, 1);
        let r = t.refusal.expect("a refusal");
        assert!(!r.pair, "the even half is 19 s old, so there is no pair");
        assert_eq!(r.other, None);
        assert_eq!(r.after_ms, 20_000 - 1_000);
        assert!(
            t.table[&0x4009DA].lat.is_some(),
            "one refusal keeps the position"
        );
    }

    /// DF4/20 altitude field for `ft`, in 25 ft steps.
    fn alt13(ft: i32) -> u32 {
        let n = ((ft + 1000) / 25) as u32;
        ((n >> 5) << 7) | (((n >> 4) & 1) << 5) | (1 << 4) | (n & 0xF)
    }

    /// A surveillance reply of format `df` from 4009DA, with flight status
    /// `fs` and a 13-bit field, in the short or long form.
    fn reply(df: u8, fs: u8, field: u32) -> Vec<u8> {
        let mut b = vec![df << 3 | fs, 0, (field >> 8) as u8 & 0x1F, field as u8];
        if df >= 16 {
            b.extend_from_slice(&[0; 7]);
        }
        b.extend_from_slice(&[0, 0, 0]);
        seal(&mut b, Some(0x4009DA));
        b
    }

    /// Altitude replies are believed only within what the aircraft could
    /// have climbed since the last altitude, and only with a flight status
    /// that exists.
    #[test]
    fn surveillance_altitude_is_held_to_what_is_possible() {
        let mut t = Tracker::new();
        t.update_raw(&hex(EVEN), 0).unwrap();
        assert_eq!(
            t.table[&0x4009DA].alt,
            Some(9075),
            "from the position message"
        );
        let alt = |t: &Tracker| t.table[&0x4009DA].alt;

        assert_eq!(
            t.update_raw(&reply(4, 0, alt13(9100)), 100),
            Some((0x4009DA, Update::SurveillanceAltitude))
        );
        assert_eq!(alt(&t), Some(9100));
        assert!(!t.table[&0x4009DA].on_ground);

        assert_eq!(
            t.update_raw(&reply(4, 0, alt13(20_000)), 200),
            None,
            "11 000 ft in a tenth of a second"
        );
        assert_eq!(alt(&t), Some(9100));
        assert_eq!(
            t.update_raw(&reply(20, 0, alt13(9300)), 12_100),
            Some((0x4009DA, Update::SurveillanceAltitude)),
            "200 ft in 12 s is within reach"
        );
        assert_eq!(
            t.update_raw(&reply(0, 0, alt13(20_000)), 50_000),
            Some((0x4009DA, Update::SurveillanceAltitude)),
            "after 30 s any altitude is taken"
        );
        assert_eq!(alt(&t), Some(20_000));

        for fs in [6, 7] {
            assert_eq!(
                t.update_raw(&reply(4, fs, alt13(20_000)), 50_100),
                None,
                "flight status {fs}"
            );
        }
        assert_eq!(
            t.update_raw(&reply(16, 0, 0x0104), 50_200),
            None,
            "126 700 ft is out of range"
        );
        assert_eq!(
            t.update_raw(&reply(4, 0, alt13(20_000) | 1 << 6), 50_300),
            None,
            "metric"
        );
        assert_eq!(alt(&t), Some(20_000));

        assert_eq!(
            t.update_raw(&reply(4, 1, alt13(20_000)), 50_400),
            Some((0x4009DA, Update::SurveillanceAltitude))
        );
        assert!(
            t.table[&0x4009DA].on_ground,
            "flight status 1 is on the ground"
        );
        assert_eq!(
            t.update_raw(&reply(21, 3, 0x0808), 50_500),
            Some((0x4009DA, Update::SurveillanceIdentity))
        );
        assert_eq!(t.table[&0x4009DA].squawk, Some(1200));
        assert!(t.table[&0x4009DA].on_ground, "and so is 3");
        assert_eq!(t.table[&0x4009DA].t_seen, 50_500);
    }

    /// A DF11 all-call names an address; the parity may carry an
    /// interrogator code in its low seven bits and nothing else.
    #[test]
    fn an_all_call_names_an_address() {
        let mut t = Tracker::new();
        let mut f = vec![11 << 3 | 5, 0x40, 0x09, 0xDA, 0, 0, 0];
        seal(&mut f, Some(0x12));
        assert_eq!(t.update_raw(&f, 0), Some((0x4009DA, Update::Address)));
        assert_eq!(t.table[&0x4009DA].frames, 1);

        seal(&mut f, Some(0x1234));
        assert_eq!(t.update_raw(&f, 0), None);
        assert_eq!(t.bad_parity, 1);
    }

    #[test]
    fn tracker_expires_by_age() {
        let mut t = Tracker::new();
        t.table.insert(
            0x3C6551,
            Aircraft {
                t_seen: 1_000,
                ..Aircraft::new(0x3C6551)
            },
        );
        t.table.insert(
            0x4009DA,
            Aircraft {
                t_seen: 90_000,
                ..Aircraft::new(0x4009DA)
            },
        );
        assert_eq!(t.expire(100_000, 60_000), 1, "the stale one goes");
        assert_eq!(t.len(), 1);
        assert!(t.table.contains_key(&0x4009DA));
    }
}
