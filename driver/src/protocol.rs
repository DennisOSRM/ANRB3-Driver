//! The RadarBox's own protocol: everything the box says that is not a Mode-S
//! sample, and everything the host says to it.
//!
//! Pure functions over bytes and strings, so it needs no device and is tested
//! without one. The driver (`dev`) does the talking and calls in here to know
//! what it heard; the Mode-S decoder (`decode`) never sees any of it - it is
//! handed sample bursts and nothing else.
//!
//! * commands - UNLOCK, ANSWER and the challenge transform
//! * replies - the text lines the box sends back
//! * bursts - the framing of the binary stream: `00 0a` terminators, PONG,
//!   and the LOCKED notice that can arrive inside it

// ---- commands -------------------------------------------------------------

/// The handshake constant, folded.
///
/// The transform recovered from rb.dll is `rotr(bswap(q) ^ K, 11) ^ K` with
/// `K = 0x5F20_7A43`. Rotation is a permutation of bit positions, so it
/// distributes over XOR and the two XORs collapse into one:
///
/// ```text
/// rotr(x ^ K, 11) ^ K = rotr(x, 11) ^ rotr(K, 11) ^ K = rotr(x, 11) ^ K'
/// ```
///
/// `K'` is `rotr(K,11) ^ K`. The two forms agree over all 2^32 possible
/// challenges, and the device accepts answers built either way.
///
/// The vendor binary holds a second constant, 0x27E374B3, for the other end of
/// the handshake; the receiver side never needs it.
pub(crate) const HS_KEY_FOLDED: u32 = 0x174B_9E4C;

/// The challenge-response transform: the value sent after `ANSWER` for a
/// given `QUESTION`.
pub fn hs_answer(q: u32) -> u32 {
    q.swap_bytes().rotate_right(11) ^ HS_KEY_FOLDED
}

/// `UNLOCK` with the eight-digit argument the vendor sends. The device does not
/// check the value; see `device::unlock_value` for what goes in it.
pub fn unlock(value: u32) -> String { format!("UNLOCK {value:08X}") }

/// Ends the session: the last thing ANRB.exe sends when it shuts down. The box
/// stops streaming at once and does not reply. (The vendor sends three
/// `~2*_OFF` commands first; they switch subsystems other RadarBox models
/// have and this one does not, so they are not sent here.) Bare LF, like the
/// other control commands.
pub const LOCK: &str = "LOCK";

/// The keepalive. The device drops the session when these stop, and answers
/// each one with `PONG`.
pub const PING: &str = "PING";

/// Asks for the firmware string; [`firmware`] finds it in the answer.
pub const VERSION: &str = "VERSION";

/// Pulses the front-panel LED. Accepted only with a bare LF.
pub const SIGNAL: &str = "SIGNAL";

/// The reply to a `QUESTION`.
pub fn answer(question: u32) -> String { format!("ANSWER {:08X}", hs_answer(question)) }

// ---- replies --------------------------------------------------------------

/// One text line from the box, as far as the handshake cares.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Reply<'a> {
    /// The challenge after UNLOCK.
    Question(u32),
    /// The handshake succeeded.
    Unlocked,
    /// The keepalive reply.
    Pong,
    /// Anything else, trimmed.
    Other(&'a str),
}

pub fn reply(line: &str) -> Reply<'_> {
    let t = line.trim();
    if let Some(q) = t.strip_prefix("QUESTION ").and_then(|h| u32::from_str_radix(h.trim(), 16).ok()) {
        return Reply::Question(q);
    }
    if t.contains("UNLOCKED") { return Reply::Unlocked; }
    if t == "PONG" { return Reply::Pong; }
    Reply::Other(t)
}

/// The answer to `VERSION`: `Fw: ` and the version, e.g. "Fw: 02.03.1".
///
/// Searched for rather than expected on its own line: if the box is still
/// streaming when it answers, the reply arrives attached to binary data. Only
/// the `Fw:` marker followed by text containing a digit is accepted, so
/// printable sample bytes or a malformed QUESTION are not taken for a version.
pub fn firmware(line: &str) -> Option<&str> {
    const MARK: &str = "Fw:";
    let rest = &line[line.find(MARK)?..];
    let end = rest.find(|c: char| !(' '..='~').contains(&c)).unwrap_or(rest.len());
    let fw = rest[..end].trim_end();
    fw[MARK.len()..].chars().any(|c| c.is_ascii_digit()).then_some(fw)
}

// ---- bursts ---------------------------------------------------------------

/// Split one device buffer into candidate bursts.
///
/// Frames arrive LF-delimited: 56 or 112 sample bytes followed by `00 0a`. A
/// single USB transfer usually carries several. Segments shorter than
/// [`MIN_SEGMENT`] bytes cannot hold a frame; if that leaves nothing, the whole
/// buffer is returned as one segment so it is still counted.
pub fn segments(buf: &[u8]) -> Segments<'_> {
    Segments { buf, q: 0, segstart: 0, emitted: false, tail_done: false, fallback_done: false }
}

/// Shortest segment that can hold a frame, in bytes.
pub const MIN_SEGMENT: usize = 20;

/// Borrowing iterator over the segments of one buffer. Allocates nothing, so
/// the decode path can run without touching the heap at all.
pub struct Segments<'a> {
    buf: &'a [u8],
    q: usize,
    segstart: usize,
    emitted: bool,
    tail_done: bool,
    fallback_done: bool,
}

impl Iterator for Segments<'_> {
    type Item = (usize, usize);

    fn next(&mut self) -> Option<(usize, usize)> {
        let len = self.buf.len();
        while self.q + 1 < len {
            if !(self.buf[self.q] == 0x00 && self.buf[self.q + 1] == 0x0a) {
                self.q += 1;
                continue;
            }
            let start = self.segstart;
            let slen = self.q - start;
            self.segstart = self.q + 2;
            self.q += 2;
            if slen >= MIN_SEGMENT {
                self.emitted = true;
                return Some((start, slen));
            }
        }
        if !self.tail_done {
            self.tail_done = true;
            if len - self.segstart >= MIN_SEGMENT {
                self.emitted = true;
                return Some((self.segstart, len - self.segstart));
            }
        }
        // Nothing was long enough to be a frame: hand back the whole buffer
        // once, so it is still counted as a burst that failed to decode.
        if !self.emitted && !self.fallback_done {
            self.fallback_done = true;
            self.emitted = true;
            return Some((0, len));
        }
        None
    }
}

/// Allocating convenience form. Prefer [`segments`] on a hot path.
pub fn split(buf: &[u8]) -> Vec<(usize, usize)> {
    segments(buf).collect()
}

/// True if the buffer is the ASCII keepalive reply rather than sample data.
pub fn is_pong(buf: &[u8]) -> bool {
    buf.starts_with(b"PONG")
}

/// True if a keepalive reply is anywhere in the burst.
///
/// It has to be searched for, not only tested at the start: a PONG arriving
/// within 4 ms of aircraft data is assembled into the same burst, which is not
/// rare - over a 5-minute checkout 41 of 150 PONGs landed at a non-zero offset,
/// with a longest run of 4 consecutive against a watchdog threshold of 5, so a
/// prefix test would eventually tear down a healthy link. A chance "PONG" in
/// sample data is four specific bytes in a few million positions per run, and
/// its failure is benign: it delays noticing a dead link, where the prefix
/// test's failure resets a live one.
pub fn contains_pong(buf: &[u8]) -> bool {
    buf.windows(4).any(|w| w == b"PONG")
}

/// Does this burst carry the device's LOCKED notice?
///
/// "UNLOCKED" contains "LOCKED", so a plain substring search fires on the
/// successful handshake as well - which would send the driver into a re-auth
/// loop at the exact moment it had succeeded. The match has to be a whole
/// word.
pub fn is_locked_notice(b: &[u8]) -> bool {
    b.windows(6).enumerate().any(|(i, w)| {
        if w != b"LOCKED" {
            return false;
        }
        let before_ok = i == 0 || !b[i - 1].is_ascii_alphabetic();
        let after_ok = b.get(i + 6).is_none_or(|c| !c.is_ascii_alphabetic());
        before_ok && after_ok
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn locked(s: &str) -> bool { is_locked_notice(s.as_bytes()) }

    /// The successful handshake reply contains the failure token; a
    /// substring search would re-authenticate at the moment it succeeded,
    /// and keep doing it.
    #[test]
    fn unlocked_is_not_a_lock_notice() {
        assert!(!locked("UNLOCKED"));
        assert!(!locked("UNLOCKED\r\n"));
        assert!(!locked("RELOCKED"));
        assert!(!locked("LOCKEDX"));
    }

    #[test]
    fn lock_notice_is_recognised() {
        assert!(locked("LOCKED"));
        assert!(locked("LOCKED\r\n"));
        assert!(locked("\r\nLOCKED\r\n"));
        // Bursts are binary, so the notice can be preceded by anything.
        assert!(is_locked_notice(&[0x8D, 0x00, b'L', b'O', b'C', b'K', b'E', b'D']));
    }

    #[test]
    fn ordinary_traffic_is_not_a_lock_notice() {
        assert!(!locked("PONG"));
        assert!(!locked(""));
        assert!(!locked("LOCKE"));
    }

    /// The handshake transform, pinned against pairs the hardware has
    /// accepted. The code folds the key; these check that folding did not
    /// change the answer, which is all the device checks.
    #[test]
    fn handshake_answers_match_the_device() {
        for (q, a) in [
            (0xAA00_6688u32, 0x021A_928Cu32),
            (0xE1E9_2C91,    0x2B79_BBD1),
            (0x01D2_D8D8,    0x5770_8556),      // these three drew UNLOCKED from
            (0xEAEC_33F6,    0x8A15_5831),      // the device in a live session
            (0x372E_E9B4,    0xD1BD_0369),
            (0xCB26_8D8E,    0xCE3A_4FE8),      // and these two with the folded key
            (0x422D_F8BE,    0xBF1C_4149),
        ] {
            assert_eq!(hs_answer(q), a, "answer for QUESTION {q:08X}");
        }
    }

    /// The folded constant has to be exactly rotr(K,11)^K, or every answer above
    /// is wrong the same way and those pairs stop being evidence. It must also
    /// agree with the two-XOR form recovered from rb.dll.
    #[test]
    fn folded_key_matches_the_binary_form() {
        const K: u32 = 0x5F20_7A43;
        assert_eq!(K.rotate_right(11) ^ K, HS_KEY_FOLDED);
        let mut q: u32 = 0;
        for _ in 0..200_000 {
            q = q.wrapping_mul(2_654_435_761).wrapping_add(12_345);
            let two_xor = (q.swap_bytes() ^ K).rotate_right(11) ^ K;
            assert_eq!(hs_answer(q), two_xor, "diverged at {q:08X}");
        }
    }

    #[test]
    fn replies_are_told_apart() {
        assert_eq!(reply("QUESTION 1A2B3C4D"), Reply::Question(0x1A2B_3C4D));
        assert_eq!(reply("QUESTION 1a2b3c4d\r"), Reply::Question(0x1A2B_3C4D));
        assert_eq!(reply("QUESTION ZZ"), Reply::Other("QUESTION ZZ"));
        assert_eq!(reply("UNLOCKED"), Reply::Unlocked);
        assert_eq!(reply(" PONG "), Reply::Pong);
        assert_eq!(firmware("Fw: 02.03.1"), Some("Fw: 02.03.1"));
        assert_eq!(firmware("PONG"), None);
        assert_eq!(firmware(""), None);
        assert_eq!(unlock(0x0259_821B), "UNLOCK 0259821B");
        assert_eq!(answer(0xAA00_6688), "ANSWER 021A928C");
    }

    /// A restart straight after a previous session finds the box still
    /// streaming, and the VERSION reply comes back inside sample data.
    #[test]
    fn firmware_is_found_inside_sample_data() {
        let glued = String::from_utf8_lossy(b"\x8d\x4b\x17\xfc\x59Fw: 02.03.1").into_owned();
        assert_eq!(firmware(&glued), Some("Fw: 02.03.1"));
        let trailing = String::from_utf8_lossy(b"Fw: 02.03.1 \x00\x8d\x4b").into_owned();
        assert_eq!(firmware(&trailing), Some("Fw: 02.03.1"));
    }

    /// And nothing without the marker is taken for a version, however
    /// printable it happens to be.
    #[test]
    fn firmware_needs_its_marker() {
        assert_eq!(firmware("QUESTION ZZ"), None);
        assert_eq!(firmware("AB12"), None);
        assert_eq!(firmware("UNLOCKED"), None);
        assert_eq!(firmware("Fw:"), None);
        assert_eq!(firmware("Fw: \u{fffd}\u{1}"), None);
    }

    #[test]
    fn splitter_handles_edge_cases() {
        // No delimiter, long enough to be a frame: one segment.
        assert_eq!(split(&[0xAA; 60]), vec![(0, 60)]);
        // Too short for a frame: still returned once so it is counted.
        assert_eq!(split(&[0xAA; 5]), vec![(0, 5)]);
        // Two frames separated by the terminator.
        let mut b = vec![0xAAu8; 30];
        b.extend_from_slice(&[0x00, 0x0a]);
        b.extend(std::iter::repeat_n(0xBBu8, 40));
        assert_eq!(split(&b), vec![(0, 30), (32, 40)]);
        // A runt before the terminator is dropped, the tail kept.
        let mut c = vec![0xAAu8; 4];
        c.extend_from_slice(&[0x00, 0x0a]);
        c.extend(std::iter::repeat_n(0xBBu8, 40));
        assert_eq!(split(&c), vec![(6, 40)]);
        assert!(is_pong(b"PONG\r\n"));
        assert!(!is_pong(b"PON"));
    }

    #[test]
    fn a_pong_glued_to_data_is_still_found() {
        assert!(contains_pong(b"PONG"));
        assert!(contains_pong(&[0x8D, 0x4B, 0x17, b'P', b'O', b'N', b'G', 0x00, 0x0a]));
        assert!(!contains_pong(b"PON"));
        assert!(is_pong(b"PONG\r\n") && !is_pong(&[0x8D, b'P', b'O', b'N', b'G']));
    }
}
