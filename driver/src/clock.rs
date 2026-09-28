//! Wall-clock time as the receiver reports it.
//!
//! Three things need local time - the UNLOCK argument (the vendor sends
//! milliseconds since local midnight), SBS timestamps (BaseStation is local
//! wall clock) and the server log - and they have to agree, so a line in the
//! log can be lined up with a line on the wire. They all come from here.

use std::time::Duration;
#[cfg(feature = "usb")]
use std::time::SystemTime;

/// Local calendar time, broken down, to the millisecond.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LocalTime {
    pub year: i32,
    pub month: u32,
    pub day: u32,
    pub hour: u32,
    pub minute: u32,
    pub second: u32,
    pub millis: u32,
}

impl LocalTime {
    /// Milliseconds since local midnight.
    pub fn millis_of_day(&self) -> u32 {
        ((self.hour * 60 + self.minute) * 60 + self.second) * 1000 + self.millis
    }
}

/// `now` in the local time zone: the TZ variable if set, the system's zone
/// otherwise - the same answer localtime_r gives, on any platform.
#[cfg(feature = "usb")]
pub fn local(now: SystemTime) -> LocalTime {
    use chrono::{Datelike, Timelike};
    let t = chrono::DateTime::<chrono::Local>::from(now);
    LocalTime {
        year: t.year(),
        month: t.month(),
        day: t.day(),
        hour: t.hour(),
        minute: t.minute(),
        second: t.second(),
        // chrono carries a leap second as 1000+ ms; clamp it into the second.
        millis: t.timestamp_subsec_millis().min(999),
    }
}

/// An elapsed time as hours:minutes:seconds. Hours keep counting rather than
/// wrapping, so a receiver that has been up for three days says so.
pub fn hms(d: Duration) -> String {
    let s = d.as_secs();
    format!("{:02}:{:02}:{:02}", s / 3600, (s / 60) % 60, s % 60)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hms_reads_like_a_clock_and_carries_past_a_day() {
        assert_eq!(hms(Duration::from_secs(0)), "00:00:00");
        assert_eq!(hms(Duration::from_secs(61)), "00:01:01");
        assert_eq!(hms(Duration::from_secs(3661)), "01:01:01");
        assert_eq!(hms(Duration::from_secs(86_400 * 3 + 7_384)), "74:03:04");
    }

    #[test]
    fn millis_of_day_counts_from_midnight() {
        let t = LocalTime {
            year: 2026,
            month: 9,
            day: 18,
            hour: 1,
            minute: 2,
            second: 3,
            millis: 4,
        };
        assert_eq!(t.millis_of_day(), 3_723_004);
    }

    #[cfg(feature = "usb")]
    #[test]
    fn local_time_is_a_real_calendar_time() {
        let t = local(std::time::SystemTime::now());
        assert!(t.year >= 2024 && (1..=12).contains(&t.month) && (1..=31).contains(&t.day));
        assert!(t.hour < 24 && t.minute < 60 && t.second < 61 && t.millis < 1000);
    }
}
