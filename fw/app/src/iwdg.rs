//! Independent watchdog (IWDG) configuration.
//!
//! IWDG runs from the LSI (~40 kHz) and is independent of the main clock
//! tree, so a CPU lockup, brown-out blip, or interrupt-storm cannot stop
//! it from kicking the chip back to reset. Configured here for ≈500 ms
//! timeout (prescaler /64, reload 312 → 312 / (40 kHz / 64) ≈ 0.5 s).
//!
//! Call `init()` once at boot, then `pet()` periodically (the main loop
//! and the control ISR both reload via the same key register, which is
//! safe because each pet is a single 32-bit write).

use stm32f1::stm32f103 as pac;

const KEY_RELOAD:   u16 = 0xAAAA;
const KEY_UNLOCK:   u16 = 0x5555;
const KEY_START:    u16 = 0xCCCC;

/// Enable IWDG with ~500 ms timeout.
pub fn init() {
    let iwdg = unsafe { &*pac::IWDG::ptr() };

    // Start the watchdog. Once started it cannot be disabled until reset.
    iwdg.kr.write(|w| unsafe { w.key().bits(KEY_START) });

    // Unlock PR/RLR for write.
    iwdg.kr.write(|w| unsafe { w.key().bits(KEY_UNLOCK) });

    // Prescaler /64 → 40 kHz / 64 = 625 Hz tick.
    // 0b100 = /64 per RM0008.
    iwdg.pr.write(|w| w.pr().bits(0b100));

    // Reload value: 312 → ≈500 ms timeout.
    iwdg.rlr.write(|w| w.rl().bits(312));

    // Initial reload to apply the new period.
    iwdg.kr.write(|w| unsafe { w.key().bits(KEY_RELOAD) });
}

/// Reload the watchdog counter. Single 16-bit write, ISR-safe.
#[inline(always)]
pub fn pet() {
    let iwdg = unsafe { &*pac::IWDG::ptr() };
    iwdg.kr.write(|w| unsafe { w.key().bits(KEY_RELOAD) });
}
