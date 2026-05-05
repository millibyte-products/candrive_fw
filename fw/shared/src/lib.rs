//! Types and constants shared across every candrive image and the host tools.
//!
//! Rules for this crate:
//!  - `#![no_std]` always — bootloader and common are no-std environments.
//!  - No external dependencies — anything added here gets pulled into all
//!    three flash images plus the host CLI.
//!  - Pure data + `const fn`. No global state, no side-effecting code.

#![no_std]
#![deny(unsafe_op_in_unsafe_fn)]

pub mod flash_layout;
pub mod boot_magic;
pub mod can_frame;
pub mod common_api;
pub mod crc32_mpeg2;
pub mod identity;
pub mod params_store;
pub mod protocol;
pub mod user_store;
