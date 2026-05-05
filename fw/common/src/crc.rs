//! STM32F1 hardware CRC unit. Word-input only.
//! Polynomial 0x04C11DB7, MSB-first, init 0xFFFFFFFF, no reflection, no XOR-out.
//! NOT identical to IEEE-802.3/zlib CRC32 — host tools must use the same convention.

use stm32f1::stm32f103 as pac;

#[inline]
fn ensure_clock() {
    let rcc = unsafe { &*pac::RCC::ptr() };
    if rcc.ahbenr.read().crcen().bit_is_clear() {
        rcc.ahbenr.modify(|_, w| w.crcen().set_bit());
    }
}

pub extern "C" fn reset() {
    ensure_clock();
    let crc = unsafe { &*pac::CRC::ptr() };
    crc.cr.write(|w| w.reset().set_bit());
}

pub extern "C" fn update(buf: *const u8, len: usize) -> u32 {
    ensure_clock();
    if buf.is_null() { return 0; }
    let crc = unsafe { &*pac::CRC::ptr() };
    let words = len / 4;
    // SAFETY: caller asserts buf is valid for `len` and 4-byte aligned.
    let p = buf as *const u32;
    for i in 0..words {
        let v = unsafe { core::ptr::read_unaligned(p.add(i)) };
        crc.dr.write(|w| w.bits(v));
    }
    crc.dr.read().bits()
}

pub extern "C" fn compute(buf: *const u8, len: usize) -> u32 {
    reset();
    update(buf, len)
}
