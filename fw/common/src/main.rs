//! candrive common image.
//!
//! Constraints (enforced by `common.x` linker ASSERTs):
//!   - No `.data` / `.bss` (no statics) — every function stateless.
//!   - No `cortex-m-rt` (no vector table, no reset handler).
//!   - No panicking — `panic_handler` traps via UDF.

#![no_std]
#![no_main]

use candrive_shared::common_api::{CommonApi, Header, COMMON_API_MAGIC, COMMON_API_VERSION};

mod bxcan;
mod crc;
mod delay;
mod flash;
mod usart;

/// Frozen-ABI table at offset 0 of the COMMON region (`0x08001000`).
#[no_mangle]
#[link_section = ".api_table"]
pub static COMMON_API_V1: CommonApi = CommonApi {
    header: Header {
        magic: COMMON_API_MAGIC,
        version: COMMON_API_VERSION,
        flags: 0,
    },
    delay_us:         delay::delay_us,
    usart_init:       usart::init,
    usart_putc:       usart::putc,
    usart_write:      usart::write,
    can_init:         bxcan::init,
    can_set_filter:   bxcan::set_filter,
    can_send:         bxcan::send,
    can_recv:         bxcan::recv,
    crc32_reset:      crc::reset,
    crc32_update:     crc::update,
    crc32_compute:    crc::compute,
    flash_unlock:     flash::unlock,
    flash_lock:       flash::lock,
    flash_erase_page: flash::erase_page,
    flash_program:    flash::program,
};

/// Linker `ENTRY()` anchor — never executed at reset.
#[no_mangle]
pub extern "C" fn common_api_v1() -> ! {
    loop { cortex_m::asm::udf(); }
}

#[panic_handler]
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop { cortex_m::asm::udf(); }
}
