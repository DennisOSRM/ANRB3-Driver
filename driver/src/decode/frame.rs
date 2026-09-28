//! The recovered frame, and the address helpers the decoder shares.

/// The 24-bit address in bytes 1-3 of a frame: the ICAO address for DF 11,
/// 17 and 18.
pub(super) fn icao(fr: &[u8]) -> u32 {
    ((fr[1] as u32) << 16) | ((fr[2] as u32) << 8) | fr[3] as u32
}

/// False for the all-zeros and all-ones addresses, which no aircraft has.
pub(super) fn is_valid_icao(a: u32) -> bool {
    a != 0 && a != 0xFF_FFFF
}

/// One recovered Mode-S frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frame {
    pub(super) bytes: [u8; 14],
    pub(super) len: u8,
    /// Number of bit errors corrected to recover it: 0, 1 or 2.
    pub corrected: u8,
    /// Milliseconds since the receiver started.
    pub ms: u32,
}

impl Frame {
    /// A frame received from elsewhere: 7 or 14 bytes, the length its
    /// downlink format calls for. Nothing about its parity is checked here.
    pub fn new(bytes: &[u8], ms: u32) -> Option<Frame> {
        let want = if bytes.first()? >> 3 < 16 { 7 } else { 14 };
        if bytes.len() != want {
            return None;
        }
        let mut b = [0u8; 14];
        b[..want].copy_from_slice(bytes);
        Some(Frame {
            bytes: b,
            len: want as u8,
            corrected: 0,
            ms,
        })
    }
    pub fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.len as usize]
    }
    pub fn len(&self) -> usize {
        self.len as usize
    }
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }
    /// Downlink format, the first 5 bits.
    pub fn df(&self) -> u8 {
        self.bytes[0] >> 3
    }
    /// 24-bit ICAO address. Meaningful for DF 11, 17 and 18.
    pub fn icao(&self) -> u32 {
        icao(&self.bytes)
    }
    /// ADS-B type code, for the 112-bit extended squitters only.
    pub fn type_code(&self) -> Option<u8> {
        if self.len == 14 && matches!(self.df(), 17 | 18) {
            Some(self.bytes[4] >> 3)
        } else {
            None
        }
    }
    pub fn hex(&self) -> String {
        self.as_bytes().iter().map(|b| format!("{b:02x}")).collect()
    }
}
