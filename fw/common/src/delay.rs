//! DWT cycle-counter busy-wait. Stateless — assumes the bootloader/app
//! has already enabled DWT in its `clocks` init before any common code
//! that delays runs. HCLK is hard-coded; if the clock tree changes, this
//! constant changes in lockstep across all images.

use cortex_m::peripheral::DWT;

const HCLK_HZ_LOCKED: u32 = 64_000_000;

#[inline]
pub extern "C" fn delay_us(us: u32) {
    // SAFETY: DWT->CYCCNT is a free-running 32-bit counter; we only read it.
    let dwt_cyccnt = 0xE000_1004 as *const u32;
    let start = unsafe { core::ptr::read_volatile(dwt_cyccnt) };
    let cycles = us.wrapping_mul(HCLK_HZ_LOCKED / 1_000_000);
    loop {
        let now = unsafe { core::ptr::read_volatile(dwt_cyccnt) };
        if now.wrapping_sub(start) >= cycles {
            break;
        }
    }
    let _ = DWT::PTR; // keep the cortex-m dep alive; lets the linker GC it cleanly
}
