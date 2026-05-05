//! Clock tree setup: HSI/2 × 16 = 64 MHz on HCLK.
//! APB1 = HCLK/2 = 32 MHz (CAN, USART2/3). APB2 = HCLK = 64 MHz.
//!
//! Lives in each application image (bootloader, app) — common can't have
//! statics so it relies on us to have set this up before any of its
//! peripheral drivers run.

use cortex_m::peripheral::Peripherals as CmPeripherals;
use stm32f1::stm32f103 as pac;

#[allow(dead_code)] // exported for cross-image reference / future use
pub const HCLK_HZ: u32 = 64_000_000;

pub fn init_64mhz() {
    let rcc   = unsafe { &*pac::RCC::ptr() };
    let flash = unsafe { &*pac::FLASH::ptr() };

    // 1. HSI on, switch SYSCLK to HSI so we can mess with the PLL.
    rcc.cr.modify(|_, w| w.hsion().set_bit());
    while rcc.cr.read().hsirdy().bit_is_clear() {}
    rcc.cfgr.modify(|_, w| w.sw().hsi());
    while !rcc.cfgr.read().sws().is_hsi() {}

    // 2. Two flash wait states + prefetch (required above 48 MHz).
    flash.acr.modify(|_, w| unsafe { w.latency().bits(0b010).prftbe().set_bit() });

    // 3. Prescalers: AHB/1, APB1/2, APB2/1, ADC/4.
    rcc.cfgr.modify(|_, w| w
        .hpre().div1()
        .ppre1().div2()
        .ppre2().div1()
        .adcpre().div4()
    );

    // 4. PLL: HSI/2 × 16 = 64 MHz.
    rcc.cr.modify(|_, w| w.pllon().clear_bit());
    while rcc.cr.read().pllrdy().bit_is_set() {}
    rcc.cfgr.modify(|_, w| w.pllsrc().clear_bit().pllmul().mul16());
    rcc.cr.modify(|_, w| w.pllon().set_bit());
    while rcc.cr.read().pllrdy().bit_is_clear() {}

    // 5. Switch SYSCLK -> PLL.
    rcc.cfgr.modify(|_, w| w.sw().pll());
    while !rcc.cfgr.read().sws().is_pll() {}

    // 6. Enable DWT cycle counter for delay_us in common.
    let mut cp = unsafe { CmPeripherals::steal() };
    cp.DCB.enable_trace();
    cp.DWT.enable_cycle_counter();
}
