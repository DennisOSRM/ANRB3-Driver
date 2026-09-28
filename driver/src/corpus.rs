//! The recorded-corpus formats, and the fingerprint over what decodes from
//! them. A matching fingerprint means two decoder builds recovered
//! byte-identical frames with the same correction depths.

use crate::Frame;
use std::fmt;

/// The header of a receiver log.
pub const RAW_MAGIC: &[u8; 8] = b"ANRBRAW1";

/// Bursts from a receiver log: [`RAW_MAGIC`], then per burst
/// `[u32 ms][u16 len]` and the bytes, little-endian. `None` when the header is
/// missing.
pub fn raw_log(d: &[u8]) -> Option<RawLog<'_>> {
    d.starts_with(RAW_MAGIC).then_some(RawLog {
        d,
        i: RAW_MAGIC.len(),
    })
}

pub struct RawLog<'a> {
    d: &'a [u8],
    i: usize,
}

impl<'a> Iterator for RawLog<'a> {
    /// Milliseconds since the receiver started, and the burst as received.
    type Item = (u32, &'a [u8]);
    fn next(&mut self) -> Option<Self::Item> {
        let h = self.d.get(self.i..self.i + 6)?;
        let ms = u32::from_le_bytes([h[0], h[1], h[2], h[3]]);
        let len = u16::from_le_bytes([h[4], h[5]]) as usize;
        let data = self.d.get(self.i + 6..self.i + 6 + len)?;
        self.i += 6 + len;
        Some((ms, data))
    }
}

/// Bursts from a bare burst file: `[u16 len]` and the bytes, repeated.
pub fn burst_file(d: &[u8]) -> BurstFile<'_> {
    BurstFile { d, i: 0 }
}

pub struct BurstFile<'a> {
    d: &'a [u8],
    i: usize,
}

impl<'a> Iterator for BurstFile<'a> {
    type Item = &'a [u8];
    fn next(&mut self) -> Option<Self::Item> {
        let h = self.d.get(self.i..self.i + 2)?;
        let len = u16::from_le_bytes([h[0], h[1]]) as usize;
        let data = self.d.get(self.i + 2..self.i + 2 + len)?;
        self.i += 2 + len;
        Some(data)
    }
}

/// FNV-1a over every decoded frame's bytes and its correction depth, printed
/// as `FINGERPRINT frames=… hash=…`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Fingerprint {
    pub frames: usize,
    pub hash: u64,
}

impl Fingerprint {
    const PRIME: u64 = 1_099_511_628_211;

    pub fn new() -> Self {
        Fingerprint {
            frames: 0,
            hash: 0xcbf2_9ce4_8422_2325,
        }
    }

    pub fn add(&mut self, f: &Frame) {
        for &b in f.as_bytes() {
            self.hash = (self.hash ^ b as u64).wrapping_mul(Self::PRIME);
        }
        self.hash = (self.hash ^ f.corrected as u64).wrapping_mul(Self::PRIME);
        self.frames += 1;
    }
}

impl Default for Fingerprint {
    fn default() -> Self {
        Self::new()
    }
}

impl fmt::Display for Fingerprint {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "FINGERPRINT frames={} hash={:016x}",
            self.frames, self.hash
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn readers_stop_cleanly_at_a_truncated_tail() {
        let mut raw = RAW_MAGIC.to_vec();
        raw.extend_from_slice(&7u32.to_le_bytes());
        raw.extend_from_slice(&3u16.to_le_bytes());
        raw.extend_from_slice(&[1, 2, 3]);
        raw.extend_from_slice(&9u32.to_le_bytes());
        raw.extend_from_slice(&5u16.to_le_bytes());
        raw.extend_from_slice(&[4, 5]); // claims 5, has 2
        let v: Vec<_> = raw_log(&raw).unwrap().collect();
        assert_eq!(v, vec![(7, &[1u8, 2, 3][..])]);
        assert!(raw_log(b"not a log").is_none());

        let b = [2u8, 0, 0xAA, 0xBB, 4, 0, 0xCC]; // second one truncated
        let v: Vec<_> = burst_file(&b).collect();
        assert_eq!(v, vec![&[0xAAu8, 0xBB][..]]);
    }

    use crate::decode::synth;

    #[test]
    fn raw_log_reads_back_what_was_written() {
        let a = synth::burst(&synth::df11(0x4CA2D6));
        let b = vec![0x5A; 300];
        let d = synth::raw_log(&[(0, &a), (70_000, &[]), (u32::MAX, &b)]);
        let v: Vec<_> = raw_log(&d).expect("magic").collect();
        assert_eq!(v, vec![(0, &a[..]), (70_000, &[][..]), (u32::MAX, &b[..])]);
    }

    #[test]
    fn raw_log_needs_the_whole_magic() {
        assert!(raw_log(b"").is_none());
        assert!(raw_log(b"ANRBRAW").is_none());
        assert!(raw_log(b"ANRBRAW2").is_none());
        assert_eq!(raw_log(RAW_MAGIC).expect("magic alone").count(), 0);
    }

    #[test]
    fn raw_log_stops_at_a_truncated_header() {
        let d = synth::raw_log(&[(5, &[1, 2])]);
        for cut in 1..=6 {
            let mut t = d.clone();
            t.extend_from_slice(&[9u8; 6][..cut - 1]); // 0 to 5 bytes of a header
            assert_eq!(raw_log(&t).unwrap().count(), 1, "{} header bytes", cut - 1);
        }
        // Cut inside the first record's header: nothing at all.
        assert_eq!(raw_log(&d[..RAW_MAGIC.len() + 5]).unwrap().count(), 0);
    }

    #[test]
    fn burst_file_reads_back_what_was_written() {
        let a = synth::burst(&synth::df11(1));
        let d = synth::burst_file(&[&a, &[], &[7]]);
        let v: Vec<_> = burst_file(&d).collect();
        assert_eq!(v, vec![&a[..], &[][..], &[7u8][..]]);
        assert_eq!(burst_file(&[]).count(), 0);
        assert_eq!(burst_file(&[3]).count(), 0, "half a length");
    }

    fn frame(hex: &str, corrected: u8) -> Frame {
        let mut f = Frame::new(&synth::hex(hex), 0).unwrap();
        f.corrected = corrected;
        f
    }

    /// FNV-1a steps over the frame bytes and then the correction depth,
    /// starting from the standard 64-bit offset basis.
    #[test]
    fn fingerprint_is_fnv1a_over_bytes_and_depth() {
        let empty = Fingerprint::new();
        assert_eq!(empty, Fingerprint::default());
        assert_eq!(
            empty.to_string(),
            "FINGERPRINT frames=0 hash=cbf29ce484222325"
        );

        let f = frame("5d4ca2d6e1f0a8", 1);
        let mut want = 0xcbf2_9ce4_8422_2325u64;
        for b in f.as_bytes().iter().chain(&[1u8]) {
            want = (want ^ *b as u64).wrapping_mul(0x100_0000_01b3);
        }
        let mut fp = Fingerprint::new();
        fp.add(&f);
        assert_eq!((fp.frames, fp.hash), (1, want));
        assert_eq!(
            fp.to_string(),
            format!("FINGERPRINT frames=1 hash={want:016x}")
        );
    }

    #[test]
    fn fingerprint_depends_on_order_and_depth() {
        let a = frame("8d4009da5833318e2bd82af8c6f5", 0);
        let b = frame("8d4009da583324fef1cbc5c7449d", 0);
        let run = |fs: &[Frame]| {
            let mut fp = Fingerprint::new();
            fs.iter().for_each(|f| fp.add(f));
            fp
        };
        assert_eq!(run(&[a, b]), run(&[a, b]));
        assert_ne!(run(&[a, b]).hash, run(&[b, a]).hash);
        assert_ne!(
            run(&[a]).hash,
            run(&[frame("8d4009da5833318e2bd82af8c6f5", 2)]).hash
        );
        assert_eq!(run(&[a, b]).frames, 2);
    }
}
