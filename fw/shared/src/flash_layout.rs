//! Authoritative flash layout. Mirrored in each crate's `memory.x` and the
//! common image's linker script — change one place, change them all.

/// Start of internal flash on STM32F103xB.
pub const FLASH_BASE: u32 = 0x0800_0000;
/// One flash erase page on F103 medium-density.
pub const FLASH_PAGE_SIZE: u32 = 1024;

pub const BOOTLOADER_BASE: u32 = 0x0800_0000;
pub const BOOTLOADER_SIZE: u32 = 8 * 1024;

/// Common code blob — the API table lives at exactly this address.
pub const COMMON_BASE: u32 = 0x0800_2000;
pub const COMMON_SIZE: u32 = 4 * 1024;

/// Persistent device config (serial number assignment, factory data).
pub const USER_STORE_BASE: u32 = 0x0800_3000;
pub const USER_STORE_SIZE: u32 = 1 * 1024;

/// Application image.
pub const APP_BASE: u32 = 0x0800_3400;
pub const APP_SIZE: u32 = 51 * 1024;

/// SRAM total (STM32F103xB has 20 KiB).
pub const RAM_BASE: u32 = 0x2000_0000;
pub const RAM_SIZE: u32 = 20 * 1024;

/// Reserved at the very top of RAM for cross-reset signaling between
/// bootloader and app. See `boot_magic`.
pub const NOINIT_SIZE: u32 = 16;
pub const NOINIT_BASE: u32 = RAM_BASE + RAM_SIZE - NOINIT_SIZE;
