//! Plain-data CAN frame. Mirrors a bxCAN mailbox payload but without
//! peripheral-specific bit packing — that conversion lives in the bxCAN
//! driver in `common`.

/// Maximum classic-CAN payload.
pub const CAN_MAX_DLEN: usize = 8;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct CanFrame {
    /// 11-bit standard ID. Extended IDs aren't used on the candrive bus.
    pub id: u16,
    /// Data length code (0..=8).
    pub len: u8,
    /// 1 = remote-transmit-request, 0 = data frame. We never send RTRs.
    pub rtr: u8,
    pub data: [u8; CAN_MAX_DLEN],
}

impl CanFrame {
    pub const fn new(id: u16, payload: &[u8]) -> Self {
        let mut data = [0u8; CAN_MAX_DLEN];
        let len = if payload.len() > CAN_MAX_DLEN { CAN_MAX_DLEN } else { payload.len() };
        let mut i = 0;
        while i < len {
            data[i] = payload[i];
            i += 1;
        }
        Self { id, len: len as u8, rtr: 0, data }
    }
}

/// 16-bit list-mode filter spec — accepts up to two STD IDs into FIFO0.
#[derive(Clone, Copy, Debug)]
#[repr(C)]
pub struct CanFilter {
    /// Filter bank number (0..=13 on F103).
    pub bank: u8,
    pub id1: u16,
    /// Set equal to `id1` to effectively disable the second slot.
    pub id2: u16,
}
