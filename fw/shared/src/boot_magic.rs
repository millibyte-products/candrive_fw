//! Cross-reset signaling slot at the top of RAM (see `flash_layout::NOINIT_BASE`).
//!
//! Because each image (bootloader, app) has its own `memory.x`, both reserve
//! the same physical RAM word and access it directly via volatile pointers.
//! A `static` would land in `.bss` and be zeroed by the reset handler, which
//! is the opposite of what we want.

use core::ptr;

use crate::flash_layout::NOINIT_BASE;

/// Set by the running app to request the bootloader skip the jump-to-app on
/// the next reset (e.g. user requested a CAN firmware update).
pub const STAY_IN_BOOTLOADER: u32 = 0xB007_EDBA;

/// Set by the app on entry so a subsequent unrelated reset still hits the
/// app rather than getting stuck in the BL after garbage RAM contents.
pub const NORMAL_BOOT: u32 = 0xA999_9999;

/// # Safety
///
/// The pointer is to a fixed RAM address that no `static` can land on (each
/// image's `memory.x` shrinks RAM by 16 bytes to reserve this slot).
#[inline]
pub fn read() -> u32 {
    unsafe { ptr::read_volatile(NOINIT_BASE as *const u32) }
}

/// # Safety
///
/// See [`read`].
#[inline]
pub fn write(value: u32) {
    unsafe { ptr::write_volatile(NOINIT_BASE as *mut u32, value) }
}
