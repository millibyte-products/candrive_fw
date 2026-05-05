//! USART3 polled @ 8N1, pins PB10 (TX) / PB11 (RX). PCLK1 = 32 MHz.

use stm32f1::stm32f103 as pac;

const PCLK1_HZ: u32 = 32_000_000;

pub extern "C" fn init(baud: u32) -> i32 {
    if baud == 0 { return -1; }

    // SAFETY: All peripheral pointers are MMIO; we don't escape the references.
    let rcc    = unsafe { &*pac::RCC::ptr() };
    let gpiob  = unsafe { &*pac::GPIOB::ptr() };
    let usart3 = unsafe { &*pac::USART3::ptr() };

    // Enable clocks: GPIOB, AFIO (APB2), USART3 (APB1).
    rcc.apb2enr.modify(|_, w| w.iopben().set_bit().afioen().set_bit());
    rcc.apb1enr.modify(|_, w| w.usart3en().set_bit());

    // PB10: AF push-pull 50 MHz (CNF=10, MODE=11 -> 0b1011 = 0xB)
    // PB11: floating input         (CNF=01, MODE=00 -> 0b0100 = 0x4)
    gpiob.crh.modify(|r, w| unsafe {
        let mut v = r.bits();
        v &= !(0xF << ((10 - 8) * 4));   // clear PB10 nibble
        v &= !(0xF << ((11 - 8) * 4));   // clear PB11 nibble
        v |= 0xB << ((10 - 8) * 4);
        v |= 0x4 << ((11 - 8) * 4);
        w.bits(v)
    });

    // BRR = PCLK1 / baud, with 4 fractional bits. The PAC lays it out as
    // mantissa[15:4] + fraction[3:0].
    let div = (PCLK1_HZ + baud / 2) / baud;
    usart3.brr.write(|w| unsafe { w.bits(div) });

    // 8N1, no flow control, TX+RX enabled, USART enabled.
    usart3.cr2.write(|w| w);
    usart3.cr3.write(|w| w);
    usart3.cr1.write(|w| w.te().set_bit().re().set_bit().ue().set_bit());
    0
}

pub extern "C" fn putc(byte: u8) {
    let usart3 = unsafe { &*pac::USART3::ptr() };
    while usart3.sr.read().txe().bit_is_clear() {}
    usart3.dr.write(|w| unsafe { w.bits(byte as u32) });
}

pub extern "C" fn write(buf: *const u8, len: usize) {
    if buf.is_null() { return; }
    // SAFETY: caller asserts buf is valid for `len`.
    let slice = unsafe { core::slice::from_raw_parts(buf, len) };
    for &b in slice {
        putc(b);
    }
}
