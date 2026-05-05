//! Hobby-servo PWM driver on TIM4 CH1/CH2.
//!
//! Hardware:
//!   * PB6 → TIM4 CH1 → SRV0
//!   * PB7 → TIM4 CH2 → SRV1
//!
//! Standard hobby servo signaling: 50 Hz frame, 1000–2000 µs active
//! pulse with 1500 µs ≈ neutral. We expose the pulse width directly
//! to the host as a u16 microsecond count. A value of 0 disables the
//! channel (output held low → servo holds last commanded position
//! until it loses power).
//!
//! Timer configuration: TIM4 ticks at 1 MHz (PSC=63 from 64 MHz APB1
//! timer clock), ARR=19999 → 50 Hz frame. CCR = pulse width in µs.

use stm32f1::stm32f103 as pac;

pub const SRV0: u8 = 0x01;
pub const SRV1: u8 = 0x02;

pub const UPDATE_S0: u8 = 1 << 0;
pub const UPDATE_S1: u8 = 1 << 1;

const PSC: u16 = 63;
const ARR: u16 = 19_999;

static mut PULSE_S0: u16 = 0;
static mut PULSE_S1: u16 = 0;

pub fn init() {
    let rcc   = unsafe { &*pac::RCC::ptr() };
    let gpiob = unsafe { &*pac::GPIOB::ptr() };
    let tim4  = unsafe { &*pac::TIM4::ptr() };

    rcc.apb2enr.modify(|_, w| w.iopben().set_bit());
    rcc.apb1enr.modify(|_, w| w.tim4en().set_bit());

    // PB6, PB7: AF push-pull, 2 MHz (CNF=10, MODE=10 → 0xA per nibble).
    // CRL nibbles 6 and 7.
    gpiob.crl.modify(|r, w| unsafe {
        let mut v = r.bits();
        v &= !(0xF << (6 * 4));
        v |=  0xA << (6 * 4);
        v &= !(0xF << (7 * 4));
        v |=  0xA << (7 * 4);
        w.bits(v)
    });

    tim4.cr1.write(|w| w);
    tim4.psc.write(|w| w.psc().bits(PSC));
    tim4.arr.write(|w| w.arr().bits(ARR));
    tim4.ccr1().write(|w| w.ccr().bits(0));
    tim4.ccr2().write(|w| w.ccr().bits(0));

    tim4.ccmr1_output().write(|w| unsafe {
        w
            .cc1s().bits(0b00)
            .oc1m().bits(0b110)
            .oc1pe().set_bit()
            .cc2s().bits(0b00)
            .oc2m().bits(0b110)
            .oc2pe().set_bit()
    });
    // Active-high outputs.
    tim4.ccer.write(|w| w
        .cc1p().clear_bit().cc1e().set_bit()
        .cc2p().clear_bit().cc2e().set_bit()
    );
    tim4.cr1.write(|w| w.arpe().set_bit().cen().set_bit());
    tim4.egr.write(|w| w.ug().set_bit());
}

/// Set one channel's pulse width in microseconds (0..=ARR). 0
/// effectively turns the output off (CCR=0 → no pulse).
pub fn set(addr: u8, pulse_us: u16) {
    let p = pulse_us.min(ARR);
    let tim4 = unsafe { &*pac::TIM4::ptr() };
    match addr {
        SRV0 => { tim4.ccr1().write(|w| w.ccr().bits(p)); unsafe { PULSE_S0 = p; } }
        SRV1 => { tim4.ccr2().write(|w| w.ccr().bits(p)); unsafe { PULSE_S1 = p; } }
        _ => {}
    }
}

pub fn get(addr: u8) -> u16 {
    unsafe {
        match addr {
            SRV0 => PULSE_S0,
            SRV1 => PULSE_S1,
            _ => 0,
        }
    }
}

/// Apply a `SetServo` payload, honoring the `update_flag` mask.
pub fn apply_set(s0: u16, s1: u16, update_flag: u8) {
    if update_flag & UPDATE_S0 != 0 { set(SRV0, s0); }
    if update_flag & UPDATE_S1 != 0 { set(SRV1, s1); }
}
