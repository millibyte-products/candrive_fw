//! Frozen-ABI function-pointer table that lives at exactly
//! [`flash_layout::COMMON_BASE`] (`0x0800_1000`).
//!
//! The bootloader and the application read this struct directly out of
//! flash via:
//!
//! ```ignore
//! use candrive_shared::common_api::COMMON_API;
//! unsafe { ((*COMMON_API).usart_init)(115_200) };
//! ```
//!
//! Rules for editing this struct (enforce when changing!):
//!  - Entries are append-only. Never reorder, never repurpose.
//!  - Bump [`Header::version`] when fields are added.
//!  - All function pointers MUST be `extern "C"` so the ABI is stable
//!    across compiler upgrades.
//!  - The `common` image must compile with no `.data`/`.bss` at link
//!    time (enforced by its linker script's `ASSERT`s) — therefore
//!    these functions must be reentrant and stateless.

use core::ptr;

use crate::can_frame::{CanFilter, CanFrame};
use crate::flash_layout::COMMON_BASE;

/// Magic bytes at `Header::magic`. ASCII `"CDRV"` little-endian.
pub const COMMON_API_MAGIC: u32 = 0x5652_4443;
pub const COMMON_API_VERSION: u16 = 1;

#[repr(C)]
pub struct Header {
    pub magic: u32,
    pub version: u16,
    pub flags: u16,
}

#[repr(C)]
pub struct CommonApi {
    pub header: Header,

    // ---- v1 entries (append-only) ----

    /// DWT-cycle-counter busy-wait. HCLK is locked at 64 MHz at boot.
    pub delay_us: extern "C" fn(u32),

    /// USART3 PB10/PB11 polled debug, 8N1.
    pub usart_init: extern "C" fn(baud: u32) -> i32,
    pub usart_putc: extern "C" fn(u8),
    pub usart_write: extern "C" fn(buf: *const u8, len: usize),

    /// bxCAN1 PA11/PA12 polled.
    pub can_init: extern "C" fn(bitrate: u32) -> i32,
    pub can_set_filter: extern "C" fn(*const CanFilter) -> i32,
    /// Returns 1 if accepted, 0 if all TX mailboxes are full, <0 on error.
    pub can_send: extern "C" fn(*const CanFrame) -> i32,
    /// Returns 1 if a frame was retrieved, 0 if FIFO0 is empty, <0 on error.
    pub can_recv: extern "C" fn(*mut CanFrame) -> i32,

    /// STM32 CRC peripheral (poly 0x04C11DB7, MSB-first, init 0xFFFFFFFF,
    /// no reflection). NOT identical to IEEE-802.3/zlib CRC32 — see
    /// software reference impl in `candrive_shared::crc32_stm`.
    pub crc32_reset: extern "C" fn(),
    pub crc32_update: extern "C" fn(buf: *const u8, len: usize) -> u32,
    pub crc32_compute: extern "C" fn(buf: *const u8, len: usize) -> u32,

    /// Flash erase/program. Refuses any write that would touch the
    /// bootloader (0x08000000-0x08000FFF) or common
    /// (0x08001000-0x08001FFF) regions. Returns 0 on success.
    pub flash_unlock: extern "C" fn() -> i32,
    pub flash_lock: extern "C" fn(),
    pub flash_erase_page: extern "C" fn(addr: u32) -> i32,
    pub flash_program: extern "C" fn(addr: u32, data: *const u8, len: usize) -> i32,
}

/// Pointer to the live API table in flash.
///
/// # Safety
/// Caller must ensure the `common.bin` image has been programmed at
/// [`COMMON_BASE`]. Use [`is_valid`] before invoking any function pointer.
pub const COMMON_API: *const CommonApi = COMMON_BASE as *const CommonApi;

/// Cheap runtime sanity check — verifies the magic word and version.
/// Caller-protected against an unprogrammed common region.
#[inline]
pub fn is_valid() -> bool {
    let header = unsafe { ptr::read_volatile(COMMON_BASE as *const Header) };
    header.magic == COMMON_API_MAGIC && header.version >= COMMON_API_VERSION
}

/// # Safety
/// `is_valid()` must have returned `true` before calling.
#[inline]
pub unsafe fn get() -> &'static CommonApi {
    unsafe { &*COMMON_API }
}
