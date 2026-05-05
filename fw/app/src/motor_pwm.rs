//! 3-phase PWM driver + DRV gate-driver enable lines.
//!
//! Pin map (matches legacy `device.h`):
//!   PA15 — FOC_IN1_PWM (TIM2 CH1, partial remap)
//!   PB3  — FOC_IN3_PWM (TIM2 CH2, partial remap)
//!   PA2  — FOC_IN2_PWM (TIM2 CH3)
//!   PA3  — FOC_EN      (gate-driver enable, GPIO push-pull)
//!   PA8  — M_NRST      (gate-driver !RST, GPIO push-pull, drive HIGH = run)
//!   PA9  — M_NSLEEP    (gate-driver !SLEEP, GPIO push-pull, drive HIGH = run)
//!   PA10 — M_NFAULT    (gate-driver !FAULT, GPIO input pull-up)
//!
//! Notes
//! -----
//! * PA15 and PB3 are JTAG (JTDI/JTDO) by default. We disable the JTAG
//!   pin function (keep SWD) before remapping TIM2 so we can drive them
//!   as PWM. SWD on PA13/PA14 stays alive — STLink reflash continues to
//!   work.
//! * 50 kHz center-aligned PWM. APB1 timer clock = 64 MHz (PCLK1=32 MHz
//!   doubles per RM0008 §7.2 because APB1 prescaler != 1). Center-aligned
//!   period = TimerClk / (2*F_PWM) = 64e6 / (2*50e3) = 640 ARR ticks.
//! * Each channel uses PWM mode 1 with preload + output compare preload
//!   so updates apply atomically at the next ARR rollover. Update event
//!   ARRs everything cleanly on enable.
//! * `set_duty(a,b,c)` takes 0..ARR per phase. The naming is "high-side
//!   duty"; complementary low-side is the gate-driver's job (the DRV
//!   chip generates dead-time + low-side automatically — that's why
//!   we only need 3 PWM lines, not 6).

use stm32f1::stm32f103 as pac;

/// PWM period in timer ticks. 50 kHz center-aligned @ 64 MHz timer clock.
pub const PWM_ARR: u16 = 640;

/// Bring up GPIOs, gate driver enables, and TIM2 in 3-phase center-aligned
/// PWM mode. Outputs come up at 0% duty.
pub fn init() {
    let rcc   = unsafe { &*pac::RCC::ptr() };
    let afio  = unsafe { &*pac::AFIO::ptr() };
    let gpioa = unsafe { &*pac::GPIOA::ptr() };
    let gpiob = unsafe { &*pac::GPIOB::ptr() };
    let tim2  = unsafe { &*pac::TIM2::ptr() };

    // 1. Enable clocks: GPIOA, GPIOB, AFIO (APB2), TIM2 (APB1).
    rcc.apb2enr.modify(|_, w| w.iopaen().set_bit().iopben().set_bit().afioen().set_bit());
    rcc.apb1enr.modify(|_, w| w.tim2en().set_bit());

    // 2. Disable JTAG (keep SWD) and apply TIM2 partial remap so
    //    CH1=PA15, CH2=PB3, CH3=PA2 become the PWM outputs.
    //    SWJ_CFG = 010 = JTAG-DP disabled + SW-DP enabled.
    //    TIM2_REMAP = 01 = partial remap 1.
    afio.mapr.modify(|r, w| unsafe {
        let mut v = r.bits();
        v &= !(0b111 << 24);    // SWJ_CFG mask
        v |=  0b010 << 24;      // JTAG off, SWD on
        v &= !(0b11 << 8);      // TIM2_REMAP mask
        v |=  0b01 << 8;        // partial remap 1
        w.bits(v)
    });

    // 3. Drive enable lines low (safe state) before configuring outputs.
    //    Gate driver chips treat NRST/NSLEEP low as "off" — exactly what
    //    we want until PWM is happy and the user explicitly enables.
    //    PA3 EN, PA8 NRST, PA9 NSLEEP all start LOW.
    gpioa.bsrr.write(|w| w.br3().set_bit().br8().set_bit().br9().set_bit());

    // 4. GPIO modes.
    //    PA2 (CH3)  = AF push-pull 50 MHz  (0xB)
    //    PA3 (EN)   = GP push-pull 50 MHz  (0x3)
    //    CRL nibbles 2,3.
    gpioa.crl.modify(|r, w| unsafe {
        let mut v = r.bits();
        v &= !(0xF << (2 * 4));
        v |=  0xB << (2 * 4);
        v &= !(0xF << (3 * 4));
        v |=  0x3 << (3 * 4);
        w.bits(v)
    });

    //    PA8  (NRST)   = GP push-pull 50 MHz (0x3)
    //    PA9  (NSLEEP) = GP push-pull 50 MHz (0x3)
    //    PA10 (NFAULT) = input pull-up       (0x8) + ODR.PA10=1 for pull-up
    //    PA15 (CH1)    = AF push-pull 50 MHz (0xB)
    //    CRH nibbles 0..7 -> pins 8..15.
    gpioa.crh.modify(|r, w| unsafe {
        let mut v = r.bits();
        v &= !(0xF << ((8  - 8) * 4));
        v |=  0x3 << ((8  - 8) * 4);
        v &= !(0xF << ((9  - 8) * 4));
        v |=  0x3 << ((9  - 8) * 4);
        v &= !(0xF << ((10 - 8) * 4));
        v |=  0x8 << ((10 - 8) * 4);
        v &= !(0xF << ((15 - 8) * 4));
        v |=  0xB << ((15 - 8) * 4);
        w.bits(v)
    });
    // Enable PA10 pull-up.
    gpioa.bsrr.write(|w| w.bs10().set_bit());

    //    PB3 (CH2) = AF push-pull 50 MHz (0xB)
    gpiob.crl.modify(|r, w| unsafe {
        let mut v = r.bits();
        v &= !(0xF << (3 * 4));
        v |=  0xB << (3 * 4);
        w.bits(v)
    });

    // 5. TIM2: stop, reset, configure.
    tim2.cr1.write(|w| w);                      // disable + zero defaults
    tim2.psc.write(|w| w.psc().bits(0));        // no prescale → 64 MHz
    tim2.arr.write(|w| w.arr().bits(PWM_ARR));
    tim2.ccr1().write(|w| w.ccr().bits(0));
    tim2.ccr2().write(|w| w.ccr().bits(0));
    tim2.ccr3().write(|w| w.ccr().bits(0));
    tim2.ccr4().write(|w| w.ccr().bits(0));

    // CCMR1: CH1 + CH2 in PWM mode 1 (110), output, preload enabled.
    tim2.ccmr1_output().write(|w| unsafe {
        w
            .cc1s().bits(0b00)   // output
            .oc1m().bits(0b110)  // PWM mode 1
            .oc1pe().set_bit()   // preload enable
            .cc2s().bits(0b00)
            .oc2m().bits(0b110)
            .oc2pe().set_bit()
    });
    // CCMR2: CH3 in PWM mode 1, preload.
    tim2.ccmr2_output().write(|w| unsafe {
        w
            .cc3s().bits(0b00)
            .oc3m().bits(0b110)
            .oc3pe().set_bit()
    });
    // Active-high outputs, enable CH1/2/3.
    tim2.ccer.write(|w| w
        .cc1p().clear_bit().cc1e().set_bit()
        .cc2p().clear_bit().cc2e().set_bit()
        .cc3p().clear_bit().cc3e().set_bit()
    );

    // CR2 stays at reset (no special master mode needed for now).
    // CR1: ARR preload, center-aligned mode 1, counter enable.
    tim2.cr1.write(|w| w
        .arpe().set_bit()
        .cms().bits(0b01)        // center-aligned mode 1
        .dir().clear_bit()       // up-counting first half
        .cen().set_bit()
    );

    // Force an update event so PSC/ARR/CCR shadow registers latch now.
    tim2.egr.write(|w| w.ug().set_bit());
}

/// Update the three PWM duty cycles. Each value is clamped to `[0, PWM_ARR]`.
///
/// PCB pinout (TIM2 partial remap 1):
///   * PA15 → TIM2 CH1 → driver `FOC_IN1` (phase **A**)
///   * PB3  → TIM2 CH2 → driver `FOC_IN3` (phase **C**)
///   * PA2  → TIM2 CH3 → driver `FOC_IN2` (phase **B**)
///
/// CH2 and CH3 are cross-routed on this board, so we send phase B to
/// CCR3 and phase C to CCR2. Earlier revisions of this function had the
/// CCR2/CCR3 assignments swapped, which produced a mirrored α-β frame:
/// open-loop forced commutation appeared to work (rotor follows the
/// rotating field in the opposite physical direction, masked by
/// `direction=-1` in cal) but voltage-mode FOC locked at a stable null
/// π/2 elec from the cal-aligned position because applied Vq mapped
/// onto the rotor's d-axis instead of the q-axis.
#[inline]
pub fn set_duty(a: u16, b: u16, c: u16) {
    let tim2 = unsafe { &*pac::TIM2::ptr() };
    let max = PWM_ARR;
    let ca = a.min(max);
    let cb = b.min(max);
    let cc = c.min(max);
    tim2.ccr1().write(|w| w.ccr().bits(ca));  // CH1 = PA15 = FOC_IN1 = phase A
    tim2.ccr2().write(|w| w.ccr().bits(cc));  // CH2 = PB3  = FOC_IN3 = driver phase C
    tim2.ccr3().write(|w| w.ccr().bits(cb));  // CH3 = PA2  = FOC_IN2 = driver phase B
}

/// Drive the gate driver's enable lines.
///
/// `enable=true` brings the bridge live (NRST=1, NSLEEP=1, EN=1).
/// `enable=false` returns to safe (all three low).
pub fn set_enable(enable: bool) {
    let gpioa = unsafe { &*pac::GPIOA::ptr() };
    if enable {
        gpioa.bsrr.write(|w| w.bs8().set_bit().bs9().set_bit().bs3().set_bit());
    } else {
        gpioa.bsrr.write(|w| w.br8().set_bit().br9().set_bit().br3().set_bit());
    }
}

/// Read the gate driver's NFAULT line. `true` = OK, `false` = fault asserted.
#[inline]
pub fn fault_ok() -> bool {
    let gpioa = unsafe { &*pac::GPIOA::ptr() };
    gpioa.idr.read().idr10().bit_is_set()
}

/// Enable TIM2 update-event interrupt and unmask it in the NVIC. Call
/// once after `init()` and after the `#[interrupt] fn TIM2()` handler
/// has been linked into the vector table.
///
/// At 50 kHz center-aligned PWM (CMS=01) the update event fires once
/// per full period (≈20 µs). The handler is responsible for clearing
/// the UIF flag and dividing down to the desired control-loop rate.
pub fn enable_update_irq() {
    let tim2 = unsafe { &*pac::TIM2::ptr() };
    // Make sure no stale UIF is pending before unmasking, otherwise the
    // ISR fires immediately as soon as we unmask.
    tim2.sr.modify(|_, w| w.uif().clear_bit());
    tim2.dier.modify(|_, w| w.uie().set_bit());

    // Default NVIC priority (0 = highest) is fine: the control ISR is
    // short and we want it to pre-empt the main loop's CAN/USART work.
    unsafe {
        cortex_m::peripheral::NVIC::unmask(pac::Interrupt::TIM2);
    }
}

/// Disable the TIM2 update-event interrupt. Use to bracket sections
/// that call `encoder::read()` or write PWM directly from thread
/// context (e.g. `foc::calibrate`), so the ISR doesn't race on SPI1
/// or fight the thread for CCR ownership.
pub fn disable_update_irq() {
    cortex_m::peripheral::NVIC::mask(pac::Interrupt::TIM2);
}

/// Acknowledge the TIM2 update-event flag. Call from inside the
/// `#[interrupt] fn TIM2()` handler before doing any work, so a long
/// handler doesn't immediately re-enter on exit.
#[inline]
pub fn ack_update_irq() {
    let tim2 = unsafe { &*pac::TIM2::ptr() };
    tim2.sr.modify(|_, w| w.uif().clear_bit());
}
