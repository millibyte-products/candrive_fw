//! bxCAN1 polled, no IRQs. Pins PA11 (RX) / PA12 (TX), no remap.
//!
//! Bit timing @ PCLK1 = 32 MHz, 1 Mbit/s:
//!   BRP=2, BS1=13, BS2=2, SJW=1; 16 quanta/bit; sample point 87.5 %.

use candrive_shared::can_frame::{CanFilter, CanFrame};
use stm32f1::stm32f103 as pac;

#[inline]
fn wait_set(reg_addr: usize, mask: u32, mut spins: u32) -> bool {
    let p = reg_addr as *const u32;
    while spins > 0 {
        if unsafe { core::ptr::read_volatile(p) } & mask != 0 { return true; }
        spins -= 1;
    }
    false
}

#[inline]
fn wait_clear(reg_addr: usize, mask: u32, mut spins: u32) -> bool {
    let p = reg_addr as *const u32;
    while spins > 0 {
        if unsafe { core::ptr::read_volatile(p) } & mask == 0 { return true; }
        spins -= 1;
    }
    false
}

pub extern "C" fn init(bitrate: u32) -> i32 {
    if bitrate != 1_000_000 { return -1; }

    let rcc   = unsafe { &*pac::RCC::ptr() };
    let gpioa = unsafe { &*pac::GPIOA::ptr() };
    let can   = unsafe { &*pac::CAN1::ptr() };

    // Clocks
    rcc.apb2enr.modify(|_, w| w.iopaen().set_bit().afioen().set_bit());
    rcc.apb1enr.modify(|_, w| w.canen().set_bit());

    // PA11: floating input  (CNF=01, MODE=00 -> 0x4)
    // PA12: AF push-pull 50 MHz (CNF=10, MODE=11 -> 0xB)
    gpioa.crh.modify(|r, w| unsafe {
        let mut v = r.bits();
        v &= !(0xF << ((11 - 8) * 4));
        v &= !(0xF << ((12 - 8) * 4));
        v |= 0x4 << ((11 - 8) * 4);
        v |= 0xB << ((12 - 8) * 4);
        w.bits(v)
    });

    // Reset CAN1 to a known state.
    rcc.apb1rstr.modify(|_, w| w.canrst().set_bit());
    rcc.apb1rstr.modify(|_, w| w.canrst().clear_bit());

    // Enter init mode.
    can.mcr.modify(|_, w| w.inrq().set_bit().sleep().clear_bit());
    let msr_addr = &can.msr as *const _ as usize;
    if !wait_set(msr_addr, 1 << 0 /* INAK */, 0x10000) { return -2; }

    // Clear unwanted MCR bits.
    can.mcr.modify(|_, w| w
        .ttcm().clear_bit()
        .abom().clear_bit()
        .awum().clear_bit()
        .nart().clear_bit()
        .rflm().clear_bit()
        .txfp().clear_bit()
    );

    // BTR: BRP-1=1, BS1-1=12, BS2-1=1, SJW-1=0.
    let btr_val: u32 = (1u32 << 0) | (12u32 << 16) | (1u32 << 20) | (0u32 << 24);
    can.btr.write(|w| unsafe { w.bits(btr_val) });

    // Leave init mode.
    can.mcr.modify(|_, w| w.inrq().clear_bit());
    if !wait_clear(msr_addr, 1 << 0, 0x10000) { return -3; }

    // Default filter: bank 0, 32-bit mask mode, accept-all -> FIFO0.
    can.fmr.modify(|_, w| w.finit().set_bit());
    can.fa1r.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << 0)) });
    can.fs1r.modify(|r, w| unsafe { w.bits(r.bits() |  (1 << 0)) });
    can.fm1r.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << 0)) });
    can.ffa1r.modify(|r, w| unsafe { w.bits(r.bits() & !(1 << 0)) });
    can.fb[0].fr1.write(|w| unsafe { w.bits(0) });
    can.fb[0].fr2.write(|w| unsafe { w.bits(0) });
    can.fa1r.modify(|r, w| unsafe { w.bits(r.bits() |  (1 << 0)) });
    can.fmr.modify(|_, w| w.finit().clear_bit());
    0
}

pub extern "C" fn set_filter(spec: *const CanFilter) -> i32 {
    if spec.is_null() { return -1; }
    let s = unsafe { core::ptr::read_volatile(spec) };
    if s.bank > 13 { return -1; }

    let can = unsafe { &*pac::CAN1::ptr() };
    let mask = 1u32 << s.bank;

    can.fmr.modify(|_, w| w.finit().set_bit());
    can.fa1r.modify(|r, w| unsafe { w.bits(r.bits() & !mask) });
    can.fs1r.modify(|r, w| unsafe { w.bits(r.bits() & !mask) }); // 16-bit scale
    can.fm1r.modify(|r, w| unsafe { w.bits(r.bits() |  mask) }); // list mode
    can.ffa1r.modify(|r, w| unsafe { w.bits(r.bits() & !mask) }); // FIFO0

    let a = ((s.id1 as u32) & 0x7FF) << 5;
    let b = ((s.id2 as u32) & 0x7FF) << 5;
    let bank = s.bank as usize;
    can.fb[bank].fr1.write(|w| unsafe { w.bits((b << 16) | a) });
    can.fb[bank].fr2.write(|w| unsafe { w.bits((b << 16) | a) });

    can.fa1r.modify(|r, w| unsafe { w.bits(r.bits() | mask) });
    can.fmr.modify(|_, w| w.finit().clear_bit());
    0
}

pub extern "C" fn send(frame: *const CanFrame) -> i32 {
    if frame.is_null() { return -1; }
    let f = unsafe { core::ptr::read_volatile(frame) };
    if f.len > 8 { return -1; }

    let can = unsafe { &*pac::CAN1::ptr() };
    let tsr = can.tsr.read().bits();
    let mb: usize = if tsr & (1 << 26) != 0 { 0 }       // TME0
              else if tsr & (1 << 27) != 0 { 1 }       // TME1
              else if tsr & (1 << 28) != 0 { 2 }       // TME2
              else { return 0; };

    let m = &can.tx[mb];
    let stid = (f.id as u32) & 0x7FF;
    let rtr  = if f.rtr != 0 { 1u32 << 1 } else { 0 };

    m.tdtr.write(|w| unsafe { w.bits(f.len as u32) });

    let mut low: u32 = 0;
    let mut high: u32 = 0;
    let n = f.len as usize;
    for i in 0..n.min(4)        { low  |= (f.data[i] as u32) << (8 * i); }
    for i in 4..n               { high |= (f.data[i] as u32) << (8 * (i - 4)); }
    m.tdlr.write(|w| unsafe { w.bits(low) });
    m.tdhr.write(|w| unsafe { w.bits(high) });

    // Set ID + TXRQ together (the request bit triggers transmission).
    m.tir.write(|w| unsafe { w.bits((stid << 21) | rtr | 1 /* TXRQ */) });
    1
}

pub extern "C" fn recv(out: *mut CanFrame) -> i32 {
    if out.is_null() { return -1; }
    let can = unsafe { &*pac::CAN1::ptr() };
    if can.rfr[0].read().fmp().bits() == 0 { return 0; }

    let m = &can.rx[0];
    let rir  = m.rir.read().bits();
    let rdtr = m.rdtr.read().bits();
    let rdlr = m.rdlr.read().bits();
    let rdhr = m.rdhr.read().bits();

    let mut frame = CanFrame::default();
    frame.id  = ((rir >> 21) & 0x7FF) as u16;
    frame.rtr = if rir & (1 << 1) != 0 { 1 } else { 0 };
    frame.len = (rdtr & 0xF) as u8;
    if frame.len > 8 { frame.len = 8; }
    let n = frame.len as usize;
    for i in 0..n.min(4)        { frame.data[i] = (rdlr >> (8 * i)) as u8; }
    for i in 4..n               { frame.data[i] = (rdhr >> (8 * (i - 4))) as u8; }

    unsafe { core::ptr::write_volatile(out, frame) };

    // Release FIFO0 slot.
    can.rfr[0].modify(|_, w| w.rfom().set_bit());
    1
}
