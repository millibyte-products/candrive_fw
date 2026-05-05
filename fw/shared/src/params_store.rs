//! Log-structured persistence for runtime motor parameters.
//!
//! Storage is the second half of the existing `USER_STORE` flash page
//! (1 KiB at 0x0800_3000). The first 128 bytes are reserved for the
//! identity record (28 bytes followed by zero-padding); the remaining
//! 896 bytes hold seven 128-byte parameter slots.
//!
//! Wear strategy: STM32F1 flash erases to all-1s and supports programming
//! any 16-bit half-word that is currently 0xFFFF. We therefore append
//! new parameter records into the first erased (all-0xFF) slot and only
//! erase the whole page once all seven slots are full. This gives
//! a 7× reduction in erase cycles compared to a naive
//! erase-then-write-at-base scheme. With a 10k-cycle/page endurance
//! budget that puts us at ~70k save calls before the page wears out.
//!
//! Layout of one slot (128 bytes, little-endian):
//! ```text
//!   off  size  field
//!     0    4   magic    = 0x4D52_4150 ('PARM')
//!     4    2   version  = current PARAMS_VERSION
//!     6    2   count    = number of populated f32 entries (<= MAX_PARAMS)
//!     8    4   seq      = monotonic u32 (wraps OK)
//!    12  104   values   = MAX_PARAMS f32 little-endian
//!   116    8   pad      = reserved 0
//!   124    4   crc32    = CRC-32/MPEG-2 over bytes 0..124
//! ```

use crate::crc32_mpeg2::crc32_mpeg2;
use crate::flash_layout::{USER_STORE_BASE, USER_STORE_SIZE};

/// ASCII `"PARM"` little-endian.
pub const PARAMS_MAGIC:   u32 = 0x4D52_4150;
pub const PARAMS_VERSION: u16 = 1;

/// Maximum number of f32 fields a single record can store. Bumped only
/// by versioning the slot layout — see `PARAMS_VERSION`.
pub const MAX_PARAMS: usize = 26;

/// Bytes occupied by one slot in flash. Power of two so the seven slots
/// pack neatly into the 1 KiB page after the 128-byte identity header.
pub const SLOT_LEN: usize = 128;

/// Number of parameter slots. The identity record consumes the first
/// `SLOT_LEN` bytes of `USER_STORE`, leaving room for this many.
pub const SLOT_COUNT: usize = 7;

/// Byte offset of the first param slot inside `USER_STORE`.
pub const SLOTS_OFFSET: u32 = SLOT_LEN as u32;

const _: () = {
    // Compile-time sanity: identity reserves SLOT_LEN, the params take
    // SLOT_COUNT * SLOT_LEN, and the total must fit inside USER_STORE.
    assert!(SLOT_LEN + SLOT_LEN * SLOT_COUNT <= USER_STORE_SIZE as usize);
};

/// One decoded parameter record.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Record {
    pub seq:    u32,
    pub count:  u16,
    pub values: [f32; MAX_PARAMS],
}

impl Record {
    /// Build an empty (count=0) record. Seq is whatever the caller wants
    /// to assign — typically `latest_seq.wrapping_add(1)`.
    pub const fn new(seq: u32) -> Self {
        Self { seq, count: 0, values: [0.0; MAX_PARAMS] }
    }

    /// Pack `values[..count]` into the record. Truncates silently to
    /// `MAX_PARAMS`.
    pub fn from_values(seq: u32, values: &[f32]) -> Self {
        let mut r = Self::new(seq);
        let n = values.len().min(MAX_PARAMS);
        r.count = n as u16;
        r.values[..n].copy_from_slice(&values[..n]);
        r
    }

    /// Serialize to the 128-byte flash image with a fresh CRC.
    pub fn to_bytes(&self) -> [u8; SLOT_LEN] {
        let mut b = [0u8; SLOT_LEN];
        b[0..4].copy_from_slice(&PARAMS_MAGIC.to_le_bytes());
        b[4..6].copy_from_slice(&PARAMS_VERSION.to_le_bytes());
        b[6..8].copy_from_slice(&self.count.to_le_bytes());
        b[8..12].copy_from_slice(&self.seq.to_le_bytes());
        for (i, v) in self.values.iter().enumerate() {
            let off = 12 + i * 4;
            b[off..off + 4].copy_from_slice(&v.to_bits().to_le_bytes());
        }
        // bytes 116..124 are pad (already 0).
        let crc = crc32_mpeg2(&b[0..SLOT_LEN - 4]);
        b[SLOT_LEN - 4..SLOT_LEN].copy_from_slice(&crc.to_le_bytes());
        b
    }

    /// Decode a 128-byte image. Returns `None` if magic / version / CRC
    /// do not check out, including the all-0xFF erased state.
    pub fn from_bytes(b: &[u8; SLOT_LEN]) -> Option<Self> {
        let magic = u32::from_le_bytes(b[0..4].try_into().ok()?);
        if magic != PARAMS_MAGIC { return None; }
        let version = u16::from_le_bytes(b[4..6].try_into().ok()?);
        if version != PARAMS_VERSION { return None; }
        let want = u32::from_le_bytes(b[SLOT_LEN - 4..SLOT_LEN].try_into().ok()?);
        let got  = crc32_mpeg2(&b[0..SLOT_LEN - 4]);
        if want != got { return None; }
        let count = u16::from_le_bytes(b[6..8].try_into().ok()?);
        let seq   = u32::from_le_bytes(b[8..12].try_into().ok()?);
        let mut values = [0.0f32; MAX_PARAMS];
        for (i, slot) in values.iter_mut().enumerate() {
            let off = 12 + i * 4;
            let bits = u32::from_le_bytes(b[off..off + 4].try_into().ok()?);
            *slot = f32::from_bits(bits);
        }
        Some(Self { seq, count, values })
    }
}

/// Result of [`scan`]. Bundles the latest valid record with bookkeeping
/// the writer needs (page-full detection, next sequence number).
#[derive(Clone, Copy, Debug)]
pub struct ScanResult {
    /// Most-recent valid record, if any was present.
    pub latest: Option<Record>,
    /// Index in `0..SLOT_COUNT` of the first slot whose magic word is
    /// erased (all 0xFF). `None` means every slot has been programmed
    /// at least once and the next save must erase the page.
    pub first_free: Option<u8>,
}

impl ScanResult {
    /// Sequence number to assign the next record. Wraps on u32 overflow.
    pub fn next_seq(&self) -> u32 {
        match self.latest {
            Some(r) => r.seq.wrapping_add(1),
            None    => 1,
        }
    }
}

/// Scan the seven param slots in `bytes` (which must cover the full
/// `USER_STORE` page) and return the latest record plus the first
/// erased slot index.
pub fn scan(page: &[u8]) -> ScanResult {
    let mut latest: Option<Record> = None;
    let mut first_free: Option<u8> = None;
    for i in 0..SLOT_COUNT {
        let off = (SLOTS_OFFSET as usize) + i * SLOT_LEN;
        if off + SLOT_LEN > page.len() { break; }
        let slice = &page[off..off + SLOT_LEN];
        // Detect erased: magic word == 0xFFFFFFFF. Cheap pre-check before
        // doing the full CRC scan.
        let magic = u32::from_le_bytes([slice[0], slice[1], slice[2], slice[3]]);
        if magic == 0xFFFF_FFFF {
            if first_free.is_none() { first_free = Some(i as u8); }
            continue;
        }
        let mut buf = [0u8; SLOT_LEN];
        buf.copy_from_slice(slice);
        if let Some(r) = Record::from_bytes(&buf) {
            latest = match latest {
                None    => Some(r),
                Some(p) => {
                    // Pick by max sequence (with wrap-aware compare).
                    if r.seq.wrapping_sub(p.seq) as i32 > 0 { Some(r) } else { Some(p) }
                }
            };
        }
        // Programmed-but-corrupt slots count as "used" for free-slot
        // accounting — we won't write into them without an erase. They
        // simply contribute nothing to `latest`.
    }
    ScanResult { latest, first_free }
}

/// Returns the absolute flash address of slot `i` (0..SLOT_COUNT).
pub const fn slot_addr(i: u8) -> u32 {
    USER_STORE_BASE + SLOTS_OFFSET + (i as u32) * (SLOT_LEN as u32)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_page(records: &[(u32, &[f32])]) -> [u8; USER_STORE_SIZE as usize] {
        let mut page = [0xFFu8; USER_STORE_SIZE as usize];
        for (i, (seq, vals)) in records.iter().enumerate() {
            let r = Record::from_values(*seq, vals);
            let off = (SLOTS_OFFSET as usize) + i * SLOT_LEN;
            page[off..off + SLOT_LEN].copy_from_slice(&r.to_bytes());
        }
        page
    }

    #[test]
    fn round_trip() {
        let r = Record::from_values(7, &[1.0, 2.0, 3.0, 4.0]);
        let parsed = Record::from_bytes(&r.to_bytes()).expect("valid");
        assert_eq!(parsed.seq, 7);
        assert_eq!(parsed.count, 4);
        assert_eq!(&parsed.values[..4], &[1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn erased_page_is_clean() {
        let page = [0xFFu8; USER_STORE_SIZE as usize];
        let s = scan(&page);
        assert!(s.latest.is_none());
        assert_eq!(s.first_free, Some(0));
        assert_eq!(s.next_seq(), 1);
    }

    #[test]
    fn picks_highest_seq() {
        let page = make_page(&[(1, &[1.0]), (5, &[5.0]), (3, &[3.0])]);
        let s = scan(&page);
        let r = s.latest.expect("some");
        assert_eq!(r.seq, 5);
        assert_eq!(r.values[0], 5.0);
        assert_eq!(s.first_free, Some(3));
    }

    #[test]
    fn full_page_no_free_slot() {
        let mut tuples: [(u32, &[f32]); SLOT_COUNT] = [(0, &[]); SLOT_COUNT];
        let vals: [f32; 1] = [42.0];
        for (i, t) in tuples.iter_mut().enumerate() {
            *t = ((i as u32) + 10, &vals);
        }
        let page = make_page(&tuples);
        let s = scan(&page);
        assert_eq!(s.first_free, None);
        assert_eq!(s.latest.expect("some").seq, 16);
    }

    #[test]
    fn corrupt_slot_ignored() {
        let mut page = make_page(&[(1, &[1.0])]);
        // Flip a byte in the first slot's CRC.
        let crc_off = (SLOTS_OFFSET as usize) + SLOT_LEN - 1;
        page[crc_off] ^= 0xFF;
        let s = scan(&page);
        assert!(s.latest.is_none());
        // Programmed-but-corrupt: not free, slot 1 is the first free.
        assert_eq!(s.first_free, Some(1));
    }
}
