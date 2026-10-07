//! What the receiver has seen over the last day: how far it reaches in each
//! direction, and how many messages and aircraft it had per minute.
//!
//! The range needs the receiver's position (`--site`). Without it only the
//! per-minute counts are kept.

use std::collections::VecDeque;
use std::fmt::Write;

/// Degrees per range sector.
pub const SECTOR: usize = 5;
/// Range sectors around the receiver.
pub const SECTORS: usize = 360 / SECTOR;
/// Hours the range is kept for.
const HOURS: usize = 24;
/// Minutes the counts are kept for.
const MINUTES: u64 = 24 * 60;
/// Farther than any receiver sees: a position beyond this is a decoding
/// error, not range.
const MAX_NM: f64 = 500.0;
/// Mean earth radius in nautical miles.
const EARTH_NM: f64 = 3440.065;

/// One minute's counts.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Minute {
    /// Minutes since the epoch.
    t: u64,
    messages: u64,
    /// The most aircraft on the map at any point in the minute.
    aircraft: u32,
}

pub struct Stats {
    site: Option<(f64, f64)>,
    /// The farthest range in each sector, in nautical miles, one row per hour
    /// slot. `hour[i]` is the hour (since the epoch) slot `i` holds; a slot
    /// from an older hour is cleared before use.
    range: [[f32; SECTORS]; HOURS],
    hour: [u64; HOURS],
    /// Completed minutes, oldest first, and the one being counted.
    minutes: VecDeque<Minute>,
    current: Option<Minute>,
    /// The feed's running message count when last seen.
    last_total: u64,
}

impl Stats {
    pub fn new(site: Option<(f64, f64)>) -> Stats {
        Stats {
            site,
            range: [[0.0; SECTORS]; HOURS],
            hour: [0; HOURS],
            minutes: VecDeque::new(),
            current: None,
            last_total: 0,
        }
    }

    /// A position the tracker has published, at `now` (seconds since the
    /// epoch).
    pub fn position(&mut self, lat: f64, lon: f64, now: f64) {
        let Some(site) = self.site else { return };
        let (nm, bearing) = distance_bearing(site, (lat, lon));
        if !(nm.is_finite() && nm <= MAX_NM) {
            return;
        }
        let hour = (now / 3600.0) as u64;
        let slot = (hour % HOURS as u64) as usize;
        if self.hour[slot] != hour {
            self.hour[slot] = hour;
            self.range[slot] = [0.0; SECTORS];
        }
        let sector = (bearing as usize / SECTOR) % SECTORS;
        let r = &mut self.range[slot][sector];
        *r = r.max(nm as f32);
    }

    /// The feed's running message count and the aircraft on the map, at
    /// `now`. A count lower than the last one means the feed started again,
    /// and counts from zero.
    pub fn counts(&mut self, total: u64, aircraft: usize, now: f64) {
        let delta = if total >= self.last_total {
            total - self.last_total
        } else {
            total
        };
        self.last_total = total;
        let t = (now / 60.0) as u64;
        let m = match self.current {
            Some(m) if m.t == t => m,
            other => {
                if let Some(done) = other {
                    self.minutes.push_back(done);
                }
                while self.minutes.front().is_some_and(|m| m.t + MINUTES <= t) {
                    self.minutes.pop_front();
                }
                Minute {
                    t,
                    messages: 0,
                    aircraft: 0,
                }
            }
        };
        self.current = Some(Minute {
            t,
            messages: m.messages + delta,
            aircraft: m.aircraft.max(aircraft as u32),
        });
    }

    /// The farthest range per sector over the last 24 hours.
    fn day_range(&self, now: f64) -> [f32; SECTORS] {
        let hour = (now / 3600.0) as u64;
        let mut out = [0.0f32; SECTORS];
        for (slot, row) in self.range.iter().enumerate() {
            if self.hour[slot] + (HOURS as u64) > hour && self.hour[slot] <= hour {
                for (o, r) in out.iter_mut().zip(row) {
                    *o = o.max(*r);
                }
            }
        }
        out
    }

    /// What the page reads: the site, the range per sector over the last
    /// day, and each completed minute of the last day as `[seconds since the
    /// epoch at its start, messages, aircraft]`.
    pub fn json(&self, now: f64) -> String {
        let mut out = String::from("{\"site\":");
        match self.site {
            Some((lat, lon)) => {
                let _ = write!(out, "[{lat:.5},{lon:.5}]");
            }
            None => out.push_str("null"),
        }
        let _ = write!(out, ",\"sector\":{SECTOR},\"range\":[");
        for (i, r) in self.day_range(now).iter().enumerate() {
            if i > 0 {
                out.push(',');
            }
            let _ = write!(out, "{}", (r * 10.0).round() / 10.0);
        }
        out.push_str("],\"minutes\":[");
        let first = (now / 60.0) as u64;
        let mut sep = "";
        for m in self.minutes.iter().filter(|m| m.t + MINUTES > first) {
            let _ = write!(out, "{sep}[{},{},{}]", m.t * 60, m.messages, m.aircraft);
            sep = ",";
        }
        out.push_str("]}");
        out
    }

    /// The counts and ranges as text, to be read back by [`Stats::load`]
    /// after a restart. The site is not saved: it comes from the command
    /// line.
    pub fn save(&self) -> String {
        let mut out = String::from("anrb-stats 1\n");
        for (slot, row) in self.range.iter().enumerate() {
            if row.iter().any(|&r| r > 0.0) {
                let _ = write!(out, "hour {}", self.hour[slot]);
                for r in row {
                    let _ = write!(out, " {r}");
                }
                out.push('\n');
            }
        }
        for m in self.minutes.iter().chain(self.current.iter()) {
            let _ = writeln!(out, "minute {} {} {}", m.t, m.messages, m.aircraft);
        }
        out
    }

    /// Read back what [`Stats::save`] wrote, keeping what is still inside
    /// the last day at `now`. Lines that do not parse are skipped, so a file
    /// cut short by a power cut loses only its last line.
    pub fn load(&mut self, text: &str, now: f64) {
        let mut lines = text.lines();
        if lines.next() != Some("anrb-stats 1") {
            return;
        }
        let hour_now = (now / 3600.0) as u64;
        let minute_now = (now / 60.0) as u64;
        for line in lines {
            let mut f = line.split_whitespace();
            match f.next() {
                Some("hour") => {
                    let Some(hour) = f.next().and_then(|h| h.parse::<u64>().ok()) else {
                        continue;
                    };
                    let row: Vec<f32> = f.filter_map(|r| r.parse().ok()).collect();
                    if row.len() != SECTORS || hour + HOURS as u64 <= hour_now || hour > hour_now {
                        continue;
                    }
                    let slot = (hour % HOURS as u64) as usize;
                    self.hour[slot] = hour;
                    self.range[slot].copy_from_slice(&row);
                }
                Some("minute") => {
                    let v: Vec<u64> = f.filter_map(|x| x.parse().ok()).collect();
                    if let [t, messages, aircraft] = v[..] {
                        if t + MINUTES > minute_now && t < minute_now {
                            self.minutes.push_back(Minute {
                                t,
                                messages,
                                aircraft: aircraft as u32,
                            });
                        }
                    }
                }
                _ => {}
            }
        }
        self.minutes.make_contiguous().sort_by_key(|m| m.t);
    }
}

/// Great-circle distance in nautical miles, and initial bearing in degrees
/// clockwise from north (0 to 360), from `a` to `b`, both `(lat, lon)` in
/// degrees.
pub fn distance_bearing(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let (la1, lo1) = (a.0.to_radians(), a.1.to_radians());
    let (la2, lo2) = (b.0.to_radians(), b.1.to_radians());
    let (dla, dlo) = (la2 - la1, lo2 - lo1);
    let h = (dla / 2.0).sin().powi(2) + la1.cos() * la2.cos() * (dlo / 2.0).sin().powi(2);
    let nm = 2.0 * EARTH_NM * h.sqrt().min(1.0).asin();
    let y = dlo.sin() * la2.cos();
    let x = la1.cos() * la2.sin() - la1.sin() * la2.cos() * dlo.cos();
    let bearing = (y.atan2(x).to_degrees() + 360.0) % 360.0;
    (nm, bearing)
}

/// A receiver position as `--site` and `ANRB_SITE` give it: `LAT,LON` in
/// decimal degrees.
pub fn parse_site(s: &str) -> Result<(f64, f64), String> {
    let err = || format!("{s}: expected LAT,LON in decimal degrees, e.g. 50.05,8.57");
    let (lat, lon) = s.split_once(',').ok_or_else(err)?;
    let lat: f64 = lat.trim().parse().map_err(|_| err())?;
    let lon: f64 = lon.trim().parse().map_err(|_| err())?;
    if !(-90.0..=90.0).contains(&lat) || !(-180.0..=180.0).contains(&lon) {
        return Err(err());
    }
    Ok((lat, lon))
}

#[cfg(test)]
mod tests {
    use super::*;

    const FRA: (f64, f64) = (50.0333, 8.5706);
    /// 2026-10-07 12:00 UTC, on the hour.
    const NOON: f64 = 1_791_374_400.0;

    #[test]
    fn distance_and_bearing_match_known_values() {
        // Frankfurt to Munich airport: about 160 NM, to the south-east.
        let (nm, b) = distance_bearing(FRA, (48.3538, 11.7861));
        assert!((nm - 155.0).abs() < 10.0, "{nm}");
        assert!((120.0..140.0).contains(&b), "{b}");
        // Due north and due west.
        let (nm, b) = distance_bearing((50.0, 8.0), (51.0, 8.0));
        assert!(
            (nm - 60.0).abs() < 0.2,
            "a degree of latitude is 60 NM: {nm}"
        );
        assert!(b.abs() < 1e-9, "{b}");
        let (_, b) = distance_bearing((0.0, 10.0), (0.0, 9.0));
        assert!((b - 270.0).abs() < 1e-9, "{b}");
        assert_eq!(distance_bearing(FRA, FRA).0, 0.0);
    }

    #[test]
    fn the_farthest_position_per_sector_is_kept() {
        let mut s = Stats::new(Some((50.0, 8.0)));
        s.position(51.0, 8.0, NOON); // 60 NM north
        s.position(50.5, 8.0, NOON); // nearer, same sector
        s.position(49.0, 8.0, NOON); // 60 NM south
        s.position(50.0, 8.0, NOON); // at the site: sector 0, no range
        let r = s.day_range(NOON);
        assert!((r[0] - 60.0).abs() < 0.5, "{}", r[0]);
        assert!((r[180 / SECTOR] - 60.0).abs() < 0.5, "{}", r[180 / SECTOR]);
        assert_eq!(r.iter().filter(|&&x| x > 0.0).count(), 2);
    }

    #[test]
    fn a_position_beyond_any_horizon_is_not_range() {
        let mut s = Stats::new(Some((50.0, 8.0)));
        s.position(60.0, 8.0, NOON); // 600 NM
        assert!(s.day_range(NOON).iter().all(|&r| r == 0.0));
    }

    #[test]
    fn without_a_site_there_is_no_range() {
        let mut s = Stats::new(None);
        s.position(51.0, 8.0, NOON);
        assert!(s
            .json(NOON)
            .starts_with("{\"site\":null,\"sector\":5,\"range\":[0,0,"));
    }

    #[test]
    fn range_older_than_a_day_is_dropped() {
        let mut s = Stats::new(Some((50.0, 8.0)));
        s.position(51.0, 8.0, NOON);
        assert!(s.day_range(NOON + 23.0 * 3600.0)[0] > 0.0, "23 hours later");
        assert_eq!(s.day_range(NOON + 24.0 * 3600.0)[0], 0.0, "24 hours later");
        // The slot is reused for the new hour, starting empty.
        s.position(50.5, 8.0, NOON + 24.0 * 3600.0);
        let r = s.day_range(NOON + 24.0 * 3600.0)[0];
        assert!((r - 30.0).abs() < 0.5, "{r}");
    }

    #[test]
    fn messages_are_counted_per_minute() {
        let mut s = Stats::new(None);
        s.counts(100, 3, NOON);
        s.counts(160, 5, NOON + 30.0);
        s.counts(200, 4, NOON + 61.0); // next minute
        s.counts(20, 4, NOON + 125.0); // the feed restarted
        assert_eq!(
            s.minutes.iter().copied().collect::<Vec<_>>(),
            [
                Minute {
                    t: NOON as u64 / 60,
                    messages: 160,
                    aircraft: 5
                },
                Minute {
                    t: NOON as u64 / 60 + 1,
                    messages: 40,
                    aircraft: 4
                },
            ]
        );
        assert_eq!(
            s.current.unwrap().messages,
            20,
            "counted from zero after a restart"
        );
        let j = s.json(NOON + 125.0);
        assert!(
            j.ends_with(&format!(
                "\"minutes\":[[{},160,5],[{},40,4]]}}",
                NOON as u64,
                NOON as u64 + 60
            )),
            "{j}"
        );
    }

    #[test]
    fn minutes_older_than_a_day_are_dropped() {
        let mut s = Stats::new(None);
        s.counts(10, 1, NOON);
        s.counts(20, 1, NOON + 60.0);
        assert_eq!(s.minutes.len(), 1);
        s.counts(30, 1, NOON + 24.0 * 3600.0 + 60.0);
        assert!(
            s.minutes.iter().all(|m| m.t > NOON as u64 / 60),
            "{:?}",
            s.minutes
        );
    }

    #[test]
    fn what_is_saved_loads_back() {
        let mut s = Stats::new(Some((50.0, 8.0)));
        s.position(51.0, 8.0, NOON - 3600.0);
        s.position(49.5, 8.0, NOON);
        s.counts(100, 2, NOON - 120.0);
        s.counts(150, 3, NOON - 60.0);
        s.counts(170, 3, NOON);
        s.counts(180, 3, NOON + 60.0); // NOON's minute is complete
        let text = s.save();
        let mut t = Stats::new(Some((50.0, 8.0)));
        t.load(
            &(text.clone() + "minute 1 2\nhour x\nnonsense\n"),
            NOON + 60.0,
        );
        assert_eq!(t.json(NOON + 60.0), s.json(NOON + 60.0), "{text}");
        // A day later nothing is left.
        let mut u = Stats::new(Some((50.0, 8.0)));
        u.load(&text, NOON + 25.0 * 3600.0);
        assert_eq!(u.save(), "anrb-stats 1\n");
        // Anything else is ignored.
        let mut v = Stats::new(None);
        v.load("something else\nminute 1 2 3\n", NOON);
        assert!(v.minutes.is_empty());
    }

    #[test]
    fn a_site_is_two_numbers_in_range() {
        assert_eq!(parse_site("50.05,8.57"), Ok((50.05, 8.57)));
        assert_eq!(parse_site(" -33.9 , 151.2 "), Ok((-33.9, 151.2)));
        for bad in ["", "50.05", "50.05;8.57", "north,east", "91,0", "0,181"] {
            assert!(parse_site(bad).is_err(), "{bad}");
        }
    }
}
