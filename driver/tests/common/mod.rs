//! Synthetic device bursts for tests: Mode-S frames turned into the sample
//! format the receiver delivers, plus the recorded-log formats around them.
//!
//! The device sends 8 samples per Mode-S bit, packed one sample per bit, most
//! significant bit first. A bit is pulse-position modulated: a 1 has its
//! energy in the first four samples, a 0 in the last four. A burst is the
//! samples of one frame and nothing else, so a 56-bit frame is 56 bytes and a
//! 112-bit frame 112 bytes.
//!
//! Short frames (downlink formats below 16) start with a 0 bit, so their first
//! four samples are zero. Stage 1 of the demodulator drops leading zeros and
//! puts four zero samples back in front of bursts of 100 bytes or fewer, which
//! puts the group grid back where the frame started. Long frames start with a
//! 1 bit and need no prefix. Encoding the frame as it is, with no lead-in,
//! therefore gives the burst the device would send.
//!
//! This file uses only the standard library. The integration tests include it
//! as `mod common`, and the library's unit tests include the same file through
//! a `#[path]` module, so there is one copy.

#![allow(dead_code)]

/// Samples per Mode-S bit.
pub const SAMPLES_PER_BIT: usize = 8;

/// Bytes from a hex string. Panics on bad input.
pub fn hex(s: &str) -> Vec<u8> {
    assert!(s.len().is_multiple_of(2), "odd hex length");
    (0..s.len() / 2)
        .map(|i| u8::from_str_radix(&s[i * 2..i * 2 + 2], 16).unwrap())
        .collect()
}

/// Bit `i` of `bytes`, counting from the most significant bit of byte 0.
pub fn bit(bytes: &[u8], i: usize) -> bool {
    bytes[i >> 3] & (0x80 >> (i & 7)) != 0
}

/// Invert bit `i` of `bytes`.
pub fn flip(bytes: &mut [u8], i: usize) {
    bytes[i >> 3] ^= 0x80 >> (i & 7);
}

/// The Mode-S CRC-24 remainder of `data` followed by 24 zero bits. Appended
/// to `data`, it makes a frame whose parity checks. Computed bit by bit, so
/// it does not share code with the library's table-driven CRC.
pub fn parity(data: &[u8]) -> u32 {
    const POLY: u32 = 0x00FF_F409;
    let mut rem = 0u32;
    let nbits = data.len() * 8 + 24;
    for i in 0..nbits {
        let inbit = i < data.len() * 8 && bit(data, i);
        let top = rem & 0x0080_0000 != 0;
        rem = ((rem << 1) | u32::from(inbit)) & 0x00FF_FFFF;
        if top {
            rem ^= POLY;
        }
    }
    rem
}

/// `data` with its parity field appended: a frame whose CRC is zero.
pub fn with_parity(data: &[u8]) -> Vec<u8> {
    with_ap(data, 0)
}

/// `data` with its parity field XORed with `addr` appended: the address and
/// parity field of the address-overlaid formats (DF 0, 4, 5, 16, 20, 21).
pub fn with_ap(data: &[u8], addr: u32) -> Vec<u8> {
    let p = parity(data) ^ addr;
    let mut v = data.to_vec();
    v.extend_from_slice(&[(p >> 16) as u8, (p >> 8) as u8, p as u8]);
    v
}

/// A DF11 all-call reply from `addr`, capability 5, interrogator code 0.
pub fn df11(addr: u32) -> Vec<u8> {
    with_parity(&[0x5D, (addr >> 16) as u8, (addr >> 8) as u8, addr as u8])
}

/// A DF4 surveillance altitude reply addressed by `addr`.
pub fn df4(addr: u32) -> Vec<u8> {
    with_ap(&[0x20, 0x00, 0x1A, 0x19], addr)
}

/// A DF17 extended squitter from `addr` with the 56-bit message `me`.
pub fn df17(addr: u32, me: [u8; 7]) -> Vec<u8> {
    let mut d = vec![0x8D, (addr >> 16) as u8, (addr >> 8) as u8, addr as u8];
    d.extend_from_slice(&me);
    with_parity(&d)
}

/// One sample per element (0 or 1) for `frame`, 8 per bit, PPM encoded.
pub fn samples(frame: &[u8]) -> Vec<u8> {
    let mut s = Vec::with_capacity(frame.len() * 8 * SAMPLES_PER_BIT);
    for i in 0..frame.len() * 8 {
        let b = u8::from(bit(frame, i));
        s.extend_from_slice(&[b, b, b, b, 1 - b, 1 - b, 1 - b, 1 - b]);
    }
    s
}

/// Set all 8 samples of Mode-S bit `i` to zero. Both halves then carry the
/// same energy, so the bit reads as 0 with no confidence either way.
pub fn erase(samples: &mut [u8], i: usize) {
    samples[i * SAMPLES_PER_BIT..(i + 1) * SAMPLES_PER_BIT].fill(0);
}

/// Pack one sample per element into bytes, most significant bit first. A
/// partial last byte is padded with zeros.
pub fn pack(samples: &[u8]) -> Vec<u8> {
    let mut v = vec![0u8; samples.len().div_ceil(8)];
    for (i, &s) in samples.iter().enumerate() {
        if s != 0 {
            v[i >> 3] |= 0x80 >> (i & 7);
        }
    }
    v
}

/// The burst the device sends for `frame`.
pub fn burst(frame: &[u8]) -> Vec<u8> {
    pack(&samples(frame))
}

/// Deterministic pseudo-random bytes (a 64-bit LCG), for noise bursts.
pub fn noise(seed: u64, len: usize) -> Vec<u8> {
    let mut x = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15) | 1;
    (0..len)
        .map(|_| {
            x = x
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            (x >> 56) as u8
        })
        .collect()
}

/// An `ANRBRAW1` receiver log: the magic, then per record `[u32 ms][u16 len]`
/// and the bytes, little-endian.
pub fn raw_log(records: &[(u32, &[u8])]) -> Vec<u8> {
    let mut v = b"ANRBRAW1".to_vec();
    for &(ms, d) in records {
        v.extend_from_slice(&ms.to_le_bytes());
        v.extend_from_slice(&(d.len() as u16).to_le_bytes());
        v.extend_from_slice(d);
    }
    v
}

/// A bare burst file: `[u16 len]` and the bytes, per burst.
pub fn burst_file(bursts: &[&[u8]]) -> Vec<u8> {
    let mut v = Vec::new();
    for d in bursts {
        v.extend_from_slice(&(d.len() as u16).to_le_bytes());
        v.extend_from_slice(d);
    }
    v
}

/// Bursts joined the way one USB transfer carries them: each followed by the
/// `00 0a` terminator.
pub fn transfer(bursts: &[&[u8]]) -> Vec<u8> {
    let mut v = Vec::new();
    for d in bursts {
        v.extend_from_slice(d);
        v.extend_from_slice(&[0x00, 0x0a]);
    }
    v
}
