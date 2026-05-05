//! ADC1 single-conversion driver for the two analog inputs A0/A1.
//!
//! Hardware:
//!   * PA0 → ADC1_IN0 → A0
//!   * PA1 → ADC1_IN1 → A1
//!
//! The protocol exposes these via `GetAnalog`, returning two raw
//! 12-bit ADC counts (0..4095). The values are unscaled — the host
//! is responsible for any voltage / current calibration.
//!
//! Conversions are software-triggered, on-demand, single-channel.
//! Each `read()` call performs two sequential conversions (~2 µs
//! each at 12-bit, 13.5-cycle sample time, ADC clock = PCLK2 / 6 =
//! 10.67 MHz).

use stm32f1::stm32f103 as pac;

pub fn init() {
    let rcc   = unsafe { &*pac::RCC::ptr() };
    let gpioa = unsafe { &*pac::GPIOA::ptr() };
    let adc1  = unsafe { &*pac::ADC1::ptr() };

    rcc.apb2enr.modify(|_, w| w.iopaen().set_bit().adc1en().set_bit());

    // ADC clock prescaler: PCLK2 / 6 = 64/6 ≈ 10.67 MHz (max 14 MHz).
    rcc.cfgr.modify(|_, w| w.adcpre().div6());

    // PA0, PA1: analog input mode. CRL nibbles 0 and 1 → 0x0 each
    // (CNF=00, MODE=00).
    gpioa.crl.modify(|r, w| unsafe {
        let mut v = r.bits();
        v &= !(0xF << (0 * 4));
        v &= !(0xF << (1 * 4));
        w.bits(v)
    });

    // Sample time: 13.5 cycles for both channels (SMP0/SMP1 = 010).
    adc1.smpr2.modify(|r, w| unsafe {
        let mut v = r.bits();
        v &= !(0b111 << (0 * 3));
        v |=  0b010 << (0 * 3);
        v &= !(0b111 << (1 * 3));
        v |=  0b010 << (1 * 3);
        w.bits(v)
    });

    // Single-conversion mode, software trigger, 12-bit, right-aligned.
    // CR2 fields: ADON=1, EXTTRIG=1, EXTSEL=111 (SWSTART), ALIGN=0,
    // CONT=0. Use a raw write to avoid PAC-version churn.
    adc1.cr2.write(|w| unsafe { w.bits(
          (1 << 0)              // ADON
        | (1 << 20)             // EXTTRIG
        | (0b111 << 17)         // EXTSEL = SWSTART
    ) });

    // Calibration: required after first ADON (RM0008 §11.12.3).
    cortex_m::asm::delay(800);  // tSTAB ~ 1 µs at 64 MHz
    adc1.cr2.modify(|_, w| w.cal().set_bit());
    while adc1.cr2.read().cal().bit_is_set() {}
}

/// Read both A0 and A1, returning raw 12-bit ADC counts.
pub fn read() -> (u16, u16) {
    (read_channel(0), read_channel(1))
}

fn read_channel(ch: u8) -> u16 {
    let adc1 = unsafe { &*pac::ADC1::ptr() };
    // 1-channel sequence with the requested channel.
    adc1.sqr1.write(|w| unsafe { w.bits(0) });          // L=0 → 1 conversion
    adc1.sqr3.write(|w| unsafe { w.bits(ch as u32 & 0x1F) });
    // SWSTART trigger: set bit 22 of CR2 (preserve the rest).
    adc1.cr2.modify(|r, w| unsafe { w.bits(r.bits() | (1 << 22)) });
    // Wait for EOC.
    while !adc1.sr.read().eoc().bit_is_set() {}
    let v = adc1.dr.read().data().bits() & 0x0FFF;
    // Clear EOC by reading DR (already done) — also clear by writing 0
    // explicitly for paranoia.
    adc1.sr.modify(|_, w| w.eoc().clear_bit());
    v
}
