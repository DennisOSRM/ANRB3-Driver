//! FTDI FT232R vendor protocol over libusb.
//!
//! The RadarBox's product id is not in the kernel's `ftdi_sio` table, so no
//! `/dev/ttyUSB*` appears and the vendor requests are issued directly.

use rusb::{DeviceHandle, GlobalContext};
use std::time::Duration;

const REQ_RESET: u8 = 0x00;
const REQ_SET_MODEM_CTRL: u8 = 0x01;
const REQ_SET_FLOW_CTRL: u8 = 0x02;
const REQ_SET_BAUDRATE: u8 = 0x03;
const REQ_SET_DATA: u8 = 0x04;
const REQ_SET_LATENCY: u8 = 0x09;

const TYPE_OUT: u8 = 0x40; // vendor, host-to-device, device recipient

/// Bulk endpoints of interface 0.
const EP_OUT: u8 = 0x02;
const EP_IN: u8 = 0x81;

/// Line-control word for 8 data bits, no parity, 1 stop bit.
const LINE_8N1: u16 = 0x0008;
/// Line-control bit that holds the line in the break state.
const LINE_BREAK: u16 = 0x4000;

/// USB packet size. Every IN packet starts with two modem-status bytes.
const PACKET: usize = 64;

/// Timeout for control requests and bulk writes.
const TIMEOUT: Duration = Duration::from_millis(300);

pub const DISABLE_FLOW_CTRL: u16 = 0x0000;

pub struct Ftdi {
    handle: DeviceHandle<GlobalContext>,
    /// Cached line-control word so the BREAK bit can be toggled without
    /// disturbing the data/parity/stop settings.
    data_cfg: u16,
}

/// Encode a baud rate as the FT232R's fractional divisor.
///
/// The part divides a 3 MHz clock by an integer plus one of eight fractions,
/// so most requested rates are only approximated. 1250000 becomes 1263158
/// (divisor 2.375), 1.05% high, which 8N1 framing absorbs without trouble.
fn encode_baud(baud: u32) -> (u16, u16, u32) {
    /// Encoding of the eighths: the part accepts 0, 1/8, 2/8 ... 7/8 but not
    /// in numeric order.
    const DIVFRAC: [u32; 8] = [0, 3, 2, 4, 1, 5, 6, 7];
    if baud == 0 {
        return (0, 0, 0);
    }
    let (encoded, actual) = if baud >= 3_000_000 {
        (0u32, 3_000_000u32) // divisor 0 means 3 MBaud exactly
    } else if baud >= 2_000_000 {
        (1u32, 2_000_000u32) // divisor 1 is a special case
    } else {
        // Divisor in eighths, rounded to nearest.
        let mut div8 = (24_000_000u32 + baud / 2) / baud;
        if div8 < 8 {
            div8 = 8; // clamp to divisor >= 1
        }
        let enc = (div8 >> 3) | (DIVFRAC[(div8 & 7) as usize] << 14);
        let act = (24_000_000u32 + div8 / 2) / div8;
        (enc, act)
    };
    ((encoded & 0xFFFF) as u16, (encoded >> 16) as u16, actual)
}

impl Ftdi {
    pub fn open(vid: u16, pid: u16) -> rusb::Result<Self> {
        let handle = rusb::open_device_with_vid_pid(vid, pid)
            .ok_or(rusb::Error::NoDevice)?;
        // Linux only; other platforms return NotSupported, which is ignored
        // because no kernel driver binds this product id there.
        match handle.set_auto_detach_kernel_driver(true) {
            Ok(()) | Err(rusb::Error::NotSupported) => {}
            Err(e) => return Err(e),
        }
        handle.claim_interface(0)?;
        Ok(Ftdi { handle, data_cfg: LINE_8N1 })
    }

    /// Explain an Access error in terms of what the platform needs, since the
    /// remedy differs and the bare errno says nothing useful.
    pub fn access_hint() -> &'static str {
        if cfg!(target_os = "linux") {
            "permission denied claiming the interface: install deploy/99-anrb.rules, or check that ftdi_sio has not bound the device (lsmod | grep ftdi_sio)"
        } else if cfg!(target_os = "macos") {
            "permission denied claiming the interface: a kernel extension holds it. Check with `ioreg -p IOUSB -l | grep -i ftdi`; if AppleUSBFTDI has bound the device it must be unloaded, as libusb cannot detach a driver on macOS"
        } else {
            "permission denied claiming the interface"
        }
    }

    pub fn reset_device(&mut self) -> rusb::Result<()> {
        self.handle.reset()
    }

    fn ctrl(&self, request: u8, value: u16, index: u16) -> rusb::Result<()> {
        self.handle
            .write_control(TYPE_OUT, request, value, index, &[], TIMEOUT)
            .map(|_| ())
    }

    pub fn reset(&self) -> rusb::Result<()> { self.ctrl(REQ_RESET, 0, 1) }
    fn purge_rx(&self) -> rusb::Result<()> { self.ctrl(REQ_RESET, 1, 1) }
    fn purge_tx(&self) -> rusb::Result<()> { self.ctrl(REQ_RESET, 2, 1) }
    pub fn purge(&self) -> rusb::Result<()> { self.purge_rx()?; self.purge_tx() }

    /// Returns the rate the part will produce, which is rarely the
    /// one asked for.
    pub fn set_baudrate(&self, baud: u32) -> rusb::Result<u32> {
        let (value, index, actual) = encode_baud(baud);
        self.ctrl(REQ_SET_BAUDRATE, value, index)?;
        Ok(actual)
    }

    /// 8 data bits, no parity, 1 stop bit.
    pub fn set_line_8n1(&mut self) -> rusb::Result<()> {
        self.data_cfg = LINE_8N1;
        self.ctrl(REQ_SET_DATA, self.data_cfg, 1)
    }

    /// Hold the line in the break state, or release it.
    pub fn set_break(&self, on: bool) -> rusb::Result<()> {
        let v = if on { self.data_cfg | LINE_BREAK } else { self.data_cfg };
        self.ctrl(REQ_SET_DATA, v, 1)
    }

    pub fn set_latency(&self, ms: u8) -> rusb::Result<()> {
        self.ctrl(REQ_SET_LATENCY, ms as u16, 1)
    }

    /// Must stay disabled: RTS# is wired to the MCU reset on this board, and
    /// letting the FTDI drive it automatically resets the receiver.
    pub fn set_flowctrl(&self, mode: u16) -> rusb::Result<()> {
        self.ctrl(REQ_SET_FLOW_CTRL, 0, mode | 1)
    }

    pub fn set_dtr(&self, on: bool) -> rusb::Result<()> {
        self.ctrl(REQ_SET_MODEM_CTRL, if on { 0x0101 } else { 0x0100 }, 1)
    }
    pub fn set_rts(&self, on: bool) -> rusb::Result<()> {
        self.ctrl(REQ_SET_MODEM_CTRL, if on { 0x0202 } else { 0x0200 }, 1)
    }

    /// Write to the device, waiting at most 300 ms.
    pub fn write(&self, data: &[u8]) -> rusb::Result<usize> {
        self.handle.write_bulk(EP_OUT, data, TIMEOUT)
    }

    /// Read payload bytes, stripping the two FTDI modem-status bytes that head
    /// every USB packet.
    pub fn read(&self, out: &mut Vec<u8>, timeout_ms: u64) -> rusb::Result<()> {
        let mut raw = [0u8; 4096];
        let n = match self.handle.read_bulk(EP_IN, &mut raw, Duration::from_millis(timeout_ms)) {
            Ok(n) => n,
            Err(rusb::Error::Timeout) => 0,
            Err(e) => return Err(e),
        };
        let mut i = 0usize;
        while i + 2 <= n {
            let chunk = (n - i).min(PACKET);
            if chunk > 2 {
                out.extend_from_slice(&raw[i + 2..i + chunk]);
            }
            i += chunk;
        }
        Ok(())
    }
}

impl Drop for Ftdi {
    fn drop(&mut self) {
        let _ = self.handle.release_interface(0);
    }
}
