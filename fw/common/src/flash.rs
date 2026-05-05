//! STM32F1 embedded flash programming.
//! Self-protection: refuses any write that overlaps bootloader or common.

use candrive_shared::flash_layout::{
    APP_BASE, APP_SIZE, BOOTLOADER_BASE, BOOTLOADER_SIZE, COMMON_BASE, COMMON_SIZE,
    FLASH_BASE, FLASH_PAGE_SIZE,
};
use stm32f1::stm32f103 as pac;

const FPEC_KEY1: u32 = 0x4567_0123;
const FPEC_KEY2: u32 = 0xCDEF_89AB;

#[inline]
fn wait_done() -> i32 {
    let flash = unsafe { &*pac::FLASH::ptr() };
    while flash.sr.read().bsy().bit_is_set() {}
    let sr = flash.sr.read();
    if sr.pgerr().bit_is_set() || sr.wrprterr().bit_is_set() {
        // W1C: write back to clear all the sticky flags.
        flash.sr.modify(|_, w| w.eop().set_bit().pgerr().set_bit().wrprterr().set_bit());
        return -1;
    }
    flash.sr.modify(|_, w| w.eop().set_bit());
    0
}

fn region_writable(addr: u32, len: u32) -> bool {
    let end = match addr.checked_add(len) {
        Some(e) => e,
        None => return false,
    };
    if addr < FLASH_BASE { return false; }
    if addr < BOOTLOADER_BASE + BOOTLOADER_SIZE && end > BOOTLOADER_BASE { return false; }
    if addr < COMMON_BASE + COMMON_SIZE && end > COMMON_BASE { return false; }
    if end > APP_BASE + APP_SIZE { return false; }
    true
}

pub extern "C" fn unlock() -> i32 {
    let flash = unsafe { &*pac::FLASH::ptr() };
    if flash.cr.read().lock().bit_is_clear() { return 0; }
    flash.keyr.write(|w| unsafe { w.bits(FPEC_KEY1) });
    flash.keyr.write(|w| unsafe { w.bits(FPEC_KEY2) });
    if flash.cr.read().lock().bit_is_set() { -1 } else { 0 }
}

pub extern "C" fn lock() {
    let flash = unsafe { &*pac::FLASH::ptr() };
    flash.cr.modify(|_, w| w.lock().set_bit());
}

pub extern "C" fn erase_page(addr: u32) -> i32 {
    if !region_writable(addr, FLASH_PAGE_SIZE) { return -2; }
    if addr & (FLASH_PAGE_SIZE - 1) != 0       { return -3; }

    let flash = unsafe { &*pac::FLASH::ptr() };
    while flash.sr.read().bsy().bit_is_set() {}
    flash.cr.modify(|_, w| w.per().set_bit());
    flash.ar.write(|w| unsafe { w.bits(addr) });
    flash.cr.modify(|_, w| w.strt().set_bit());
    let rc = wait_done();
    flash.cr.modify(|_, w| w.per().clear_bit());
    rc
}

pub extern "C" fn program(addr: u32, data: *const u8, len: usize) -> i32 {
    if data.is_null()                   { return -3; }
    if !region_writable(addr, len as u32) { return -2; }
    if len & 1 != 0                     { return -3; }
    if addr & 1 != 0                    { return -3; }

    let flash = unsafe { &*pac::FLASH::ptr() };
    while flash.sr.read().bsy().bit_is_set() {}
    flash.cr.modify(|_, w| w.pg().set_bit());

    let halfwords = len / 2;
    for i in 0..halfwords {
        // SAFETY: caller asserts `data` is valid for `len` bytes and the
        // destination range is in flash and within a writable region.
        let src = unsafe { core::ptr::read_unaligned((data as *const u16).add(i)) };
        let dst = (addr as usize + i * 2) as *mut u16;
        unsafe { core::ptr::write_volatile(dst, src) };
        if wait_done() != 0 {
            flash.cr.modify(|_, w| w.pg().clear_bit());
            return -1;
        }
    }
    flash.cr.modify(|_, w| w.pg().clear_bit());
    0
}
