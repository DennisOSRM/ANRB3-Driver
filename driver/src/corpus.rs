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
    d.starts_with(RAW_MAGIC).then_some(RawLog { d, i: RAW_MAGIC.len() })
}

pub struct RawLog<'a> { d: &'a [u8], i: usize }

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
pub fn burst_file(d: &[u8]) -> BurstFile<'_> { BurstFile { d, i: 0 } }

pub struct BurstFile<'a> { d: &'a [u8], i: usize }

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
pub struct Fingerprint { pub frames: usize, pub hash: u64 }

impl Fingerprint {
    const PRIME: u64 = 1_099_511_628_211;

    pub fn new() -> Self { Fingerprint { frames: 0, hash: 1_469_598_103_934_665_603 } }

    pub fn add(&mut self, f: &Frame) {
        for &b in f.as_bytes() {
            self.hash = (self.hash ^ b as u64).wrapping_mul(Self::PRIME);
        }
        self.hash = (self.hash ^ f.corrected as u64).wrapping_mul(Self::PRIME);
        self.frames += 1;
    }
}

impl Default for Fingerprint {
    fn default() -> Self { Self::new() }
}

impl fmt::Display for Fingerprint {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "FINGERPRINT frames={} hash={:016x}", self.frames, self.hash)
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
        raw.extend_from_slice(&[4, 5]);                     // claims 5, has 2
        let v: Vec<_> = raw_log(&raw).unwrap().collect();
        assert_eq!(v, vec![(7, &[1u8, 2, 3][..])]);
        assert!(raw_log(b"not a log").is_none());

        let b = [2u8, 0, 0xAA, 0xBB, 4, 0, 0xCC];             // second one truncated
        let v: Vec<_> = burst_file(&b).collect();
        assert_eq!(v, vec![&[0xAAu8, 0xBB][..]]);
    }
}
