//! Persistent device identity stored in the `USER_STORE` flash page.
//!
//! Layout is a single 28-byte fixed record at the very start of the
//! `USER_STORE` region. STM32F1 flash is erased to all-1s, so a blank
//! page reads as `magic == 0xFFFF_FFFF` which we explicitly reject in
//! `Slot::is_valid()`.
//!
//! Wear leveling: deferred until phase 5 (will move to two A/B slots in
//! the same page once write frequency exceeds discovery setup).

use crate::crc32_mpeg2::crc32_mpeg2;

/// ASCII `"USRC"` little-endian.
pub const USER_STORE_MAGIC: u32 = 0x4352_5355;
pub const USER_STORE_VERSION: u16 = 1;

/// Sentinel returned when no record has ever been written. Hosts treat
/// this as "factory state" and the firmware uses the embedded fallback
/// in [`Slot::factory_default`].
pub const SLOT_RAW_LEN: usize = 28;

/// Persistent identity record. Field order is part of the on-disk format
/// — never reorder, append-only.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Slot {
    pub magic:       u32,
    pub version:     u16,
    pub flags:       u16,
    pub seq:         u32,
    pub serial_no:   u32,
    pub assigned_id: u8,
    pub _pad:        [u8; 7],
    pub crc32:       u32,
}

impl Slot {
    /// Factory default when the user_store has never been programmed.
    /// Phase-3 firmware used this same constant hard-coded.
    pub const FACTORY_SERIAL: u32 = 0xCD00_0001;
    pub const UNASSIGNED_ID:  u8  = 0xFF;

    pub const fn factory_default() -> Self {
        Self {
            magic: USER_STORE_MAGIC,
            version: USER_STORE_VERSION,
            flags: 0,
            seq: 0,
            serial_no: Self::FACTORY_SERIAL,
            assigned_id: Self::UNASSIGNED_ID,
            _pad: [0; 7],
            crc32: 0,
        }
    }

    /// Returns true if `magic`, `version`, and stored CRC all check out.
    pub fn is_valid(&self) -> bool {
        if self.magic != USER_STORE_MAGIC { return false; }
        if self.version != USER_STORE_VERSION { return false; }
        let buf = self.to_bytes();
        let computed = crc32_mpeg2(&buf[0..SLOT_RAW_LEN - 4]);
        computed == self.crc32
    }

    /// Serialize to the wire/flash byte layout (little-endian).
    pub fn to_bytes(&self) -> [u8; SLOT_RAW_LEN] {
        let mut b = [0u8; SLOT_RAW_LEN];
        b[0..4].copy_from_slice(&self.magic.to_le_bytes());
        b[4..6].copy_from_slice(&self.version.to_le_bytes());
        b[6..8].copy_from_slice(&self.flags.to_le_bytes());
        b[8..12].copy_from_slice(&self.seq.to_le_bytes());
        b[12..16].copy_from_slice(&self.serial_no.to_le_bytes());
        b[16] = self.assigned_id;
        b[17..24].copy_from_slice(&self._pad);
        b[24..28].copy_from_slice(&self.crc32.to_le_bytes());
        b
    }

    /// Inverse of `to_bytes`. Does not validate CRC — call `is_valid()`.
    pub fn from_bytes(b: &[u8; SLOT_RAW_LEN]) -> Self {
        let mut pad = [0u8; 7];
        pad.copy_from_slice(&b[17..24]);
        Self {
            magic: u32::from_le_bytes(b[0..4].try_into().unwrap()),
            version: u16::from_le_bytes(b[4..6].try_into().unwrap()),
            flags: u16::from_le_bytes(b[6..8].try_into().unwrap()),
            seq: u32::from_le_bytes(b[8..12].try_into().unwrap()),
            serial_no: u32::from_le_bytes(b[12..16].try_into().unwrap()),
            assigned_id: b[16],
            _pad: pad,
            crc32: u32::from_le_bytes(b[24..28].try_into().unwrap()),
        }
    }

    /// Compute and store a fresh CRC, returning the byte image ready to
    /// be programmed into flash.
    pub fn finalize(mut self) -> [u8; SLOT_RAW_LEN] {
        // CRC computed over the leading 24 bytes (everything except the
        // CRC slot itself).
        let mut staging = self.to_bytes();
        self.crc32 = crc32_mpeg2(&staging[0..SLOT_RAW_LEN - 4]);
        staging[24..28].copy_from_slice(&self.crc32.to_le_bytes());
        staging
    }
}

/// Decode a raw flash page-prefix into a [`Slot`], if it parses and CRCs.
/// `bytes` must be at least [`SLOT_RAW_LEN`] bytes long.
pub fn parse(bytes: &[u8]) -> Option<Slot> {
    if bytes.len() < SLOT_RAW_LEN { return None; }
    let mut buf = [0u8; SLOT_RAW_LEN];
    buf.copy_from_slice(&bytes[..SLOT_RAW_LEN]);
    let slot = Slot::from_bytes(&buf);
    if slot.is_valid() { Some(slot) } else { None }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip() {
        let mut s = Slot::factory_default();
        s.seq = 7;
        s.assigned_id = 5;
        let bytes = s.finalize();
        let parsed = parse(&bytes).expect("must validate");
        assert_eq!(parsed.seq, 7);
        assert_eq!(parsed.assigned_id, 5);
        assert_eq!(parsed.serial_no, Slot::FACTORY_SERIAL);
    }

    #[test]
    fn blank_flash_rejected() {
        let blank = [0xFFu8; SLOT_RAW_LEN];
        assert!(parse(&blank).is_none());
    }

    #[test]
    fn corrupt_payload_rejected() {
        let mut bytes = Slot::factory_default().finalize();
        bytes[12] ^= 0x01; // flip a bit in serial_no, CRC mismatch
        assert!(parse(&bytes).is_none());
    }
}
