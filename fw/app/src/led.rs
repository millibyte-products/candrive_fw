//! Indicator LEDs (SYS, STAT) on TIM3 PWM.
//!
//! Hardware:
//!   * PB4 → TIM3 CH1 → SYS  LED
//!   * PB5 → TIM3 CH2 → STAT LED
//!
//! Both are active-low (the LEDs are sourced from VCC and the MCU
//! sinks current — a TIM3 channel set to active-low polarity gives
//! `duty=100%` → mostly-on, `duty=0%` → off, matching the rest of
//! the protocol's expectations).
//! Duty cycle is exposed to the host as a 0..=100 percentage. The
//! timer ticks at 1 MHz (PSC=63 from 64 MHz APB1 timer clock) with
//! ARR=999, giving a 1 kHz PWM carrier and 1000-step internal
//! resolution. CCR = duty * 10. 1 kHz is fast enough to be
//! flicker-free, slow enough that the GPIO + LED transition cleanly,
//! and 1000 steps gives smoother apparent brightness than 100.
//!
//! PB4 is `JTRST` after reset. `motor_pwm::init` already programs
//! `AFIO_MAPR.SWJ_CFG = 010` (JTAG off, SWD on) which frees PB4 for
//! normal AF use, so this module assumes SWJ_CFG has been set before
//! `init` runs.

use stm32f1::stm32f103 as pac;

/// Address values used on the wire (`SetLed`/`GetLed` payloads).
pub const LED_STAT: u8 = 0x01;
pub const LED_SYS:  u8 = 0x02;

/// `update_flag` bits in the `SetLed` payload — bit 0 selects STAT,
/// bit 1 selects SYS. Set bits get the new duty value applied;
/// cleared bits are left untouched.
pub const UPDATE_STAT: u8 = 1 << 0;
pub const UPDATE_SYS:  u8 = 1 << 1;

const ARR: u16 = 999;
const PSC: u16 = 63;       // 64 MHz / 64 = 1 MHz tick → 1 kHz PWM with ARR=999.
const CCR_PER_PCT: u16 = 10;

static mut DUTY_SYS:  u8 = 0;
static mut DUTY_STAT: u8 = 0;

pub fn init() {
    let rcc   = unsafe { &*pac::RCC::ptr() };
    let afio  = unsafe { &*pac::AFIO::ptr() };
    let gpiob = unsafe { &*pac::GPIOB::ptr() };
    let tim3  = unsafe { &*pac::TIM3::ptr() };

    rcc.apb2enr.modify(|_, w| w.iopben().set_bit().afioen().set_bit());
    rcc.apb1enr.modify(|_, w| w.tim3en().set_bit());

    // TIM3_REMAP = 10 (partial remap): CH1 → PB4, CH2 → PB5.
    // Default mapping (00) would put CH1/CH2 on PA6/PA7 which are
    // the encoder MISO / MOSI lines — not what we want.
    afio.mapr.modify(|r, w| unsafe {
        let mut v = r.bits();
        v &= !(0b11 << 10);     // TIM3_REMAP mask
        v |=  0b10 << 10;       // partial remap
        w.bits(v)
    });

    // PB4, PB5: AF push-pull, 2 MHz (CNF=10, MODE=10 → 0xA per nibble).
    gpiob.crl.modify(|r, w| unsafe {
        let mut v = r.bits();
        v &= !(0xF << (4 * 4));
        v |=  0xA << (4 * 4);
        v &= !(0xF << (5 * 4));
        v |=  0xA << (5 * 4);
        w.bits(v)
    });

    tim3.cr1.write(|w| w);                      // disable + reset defaults
    tim3.psc.write(|w| w.psc().bits(PSC));
    tim3.arr.write(|w| w.arr().bits(ARR));
    tim3.ccr1().write(|w| w.ccr().bits(0));
    tim3.ccr2().write(|w| w.ccr().bits(0));

    // CH1 + CH2: PWM mode 1, preload enabled, output mode.
    tim3.ccmr1_output().write(|w| unsafe {
        w
            .cc1s().bits(0b00)
            .oc1m().bits(0b110)
            .oc1pe().set_bit()
            .cc2s().bits(0b00)
            .oc2m().bits(0b110)
            .oc2pe().set_bit()
    });
    // Active-low (LEDs are pulled to VCC; MCU sinks). Set CCxP to
    // invert the channel output so duty=100 → mostly sinking → bright.
    tim3.ccer.write(|w| w
        .cc1p().set_bit().cc1e().set_bit()
        .cc2p().set_bit().cc2e().set_bit()
    );
    tim3.cr1.write(|w| w.arpe().set_bit().cen().set_bit());
    tim3.egr.write(|w| w.ug().set_bit());
}

/// Set one LED's duty (`0..=100`). Out-of-range addresses are ignored.
pub fn set(addr: u8, duty: u8) {
    let d = duty.min(100);
    let tim3 = unsafe { &*pac::TIM3::ptr() };
    let ccr = (d as u16) * CCR_PER_PCT;
    match addr {
        LED_SYS  => {
            tim3.ccr1().write(|w| w.ccr().bits(ccr));
            unsafe { DUTY_SYS = d; }
        }
        LED_STAT => {
            tim3.ccr2().write(|w| w.ccr().bits(ccr));
            unsafe { DUTY_STAT = d; }
        }
        _ => {}
    }
}

/// Read back the most recently applied duty (`0..=100`). Unknown
/// addresses return 0.
pub fn get(addr: u8) -> u8 {
    unsafe {
        match addr {
            LED_SYS  => DUTY_SYS,
            LED_STAT => DUTY_STAT,
            _ => 0,
        }
    }
}

/// Apply a `SetLed` payload, honoring the `update_flag` mask.
pub fn apply_set(sys: u8, stat: u8, update_flag: u8) {
    if update_flag & UPDATE_SYS  != 0 { set(LED_SYS,  sys);  }
    if update_flag & UPDATE_STAT != 0 { set(LED_STAT, stat); }
}
