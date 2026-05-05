//! MT6701 magnetic encoder driver (SSI mode).
//!
//! Wiring (matches legacy hardware):
//!   PA4  – CS  (active-low, driven manually as GPIO)
//!   PA5  – SCK (SPI1_SCK, alternate function push-pull)
//!   PA6  – MISO (SPI1_MISO, input floating)
//!   PA7  – MOSI (SPI1_MOSI, AF push-pull; encoder ignores it)
//!
//! Frame format (24 bits, MSB first):
//!   [23..10] = 14-bit angle (0..16383, 0..2π)
//!   [9..6]   = 4-bit field/status (push-button, no-mag, weak-mag, strong-mag)
//!   [5..0]   = 6-bit CRC over the leading 18 bits (CRC-6, poly 0x43)
//!
//! Mode: SPI mode 0 (CPOL=0, CPHA=0), 1 MHz (matches legacy).
//! APB2 = 64 MHz so prescaler /64 yields exactly 1 MHz on SCK.
//!
//! Note: the MT6701 datasheet calls for SPI mode 2 with a 25-bit frame,
//! but mode 2 with the leading-dummy-bit interpretation produces angle
//! data with worse stability on this hardware (validated 6 May 2026).
//! Mode 0 yields good angle data; the CRC field consistently mismatches
//! (cosmetic only — the per-tick glitch filter in control::step rejects
//! the rare bad sample that does slip through).

use stm32f1::stm32f103 as pac;

const ANGLE_MASK: u32 = 0x3FFF; // 14 bits

/// One sample from the encoder.
#[derive(Copy, Clone, Default, Debug)]
pub struct Sample {
    /// 14-bit raw angle (0..16383). 0..2π wraps modulo 16384.
    pub angle: u16,
    /// 4-bit field status. Bits per MT6701 datasheet:
    ///   bit3 = push-button, bit2 = no-mag, bit1 = weak-mag, bit0 = strong-mag.
    pub status: u8,
    /// `true` if CRC-6 over [23..6] matches the trailing 6 bits.
    pub crc_ok: bool,
}

/// Configure GPIOA + SPI1 for MT6701 SSI reads. Idempotent enough for
/// app boot — we don't expect anyone else to be sharing SPI1.
pub fn init() {
    let rcc   = unsafe { &*pac::RCC::ptr() };
    let gpioa = unsafe { &*pac::GPIOA::ptr() };
    let spi1  = unsafe { &*pac::SPI1::ptr() };

    // 1. Clocks: GPIOA (APB2), AFIO (APB2), SPI1 (APB2).
    rcc.apb2enr.modify(|_, w| w.iopaen().set_bit().afioen().set_bit().spi1en().set_bit());

    // 2. PA4 (CS) = GPIO push-pull 50 MHz, idle high.
    //    PA5 (SCK) = AF push-pull 50 MHz.
    //    PA6 (MISO) = input floating.
    //    PA7 (MOSI) = AF push-pull 50 MHz (unused but driven by peripheral).
    //    CRL fields: 4 bits per pin, CNF[3:2] | MODE[1:0].
    //      output 50MHz push-pull        = 0b0011 (0x3)
    //      output 50MHz AF push-pull     = 0b1011 (0xB)
    //      input floating                = 0b0100 (0x4)
    gpioa.bsrr.write(|w| w.bs4().set_bit()); // CS high before reconfig.
    gpioa.crl.modify(|r, w| unsafe {
        let mut v = r.bits();
        // PA4
        v &= !(0xF << (4 * 4));
        v |=  0x3 << (4 * 4);
        // PA5
        v &= !(0xF << (5 * 4));
        v |=  0xB << (5 * 4);
        // PA6
        v &= !(0xF << (6 * 4));
        v |=  0x4 << (6 * 4);
        // PA7
        v &= !(0xF << (7 * 4));
        v |=  0xB << (7 * 4);
        w.bits(v)
    });

    // 3. SPI1: master, mode 0, BR=/64 (1 MHz @ 64 MHz APB2), 8-bit, MSB first,
    //    software NSS (we own PA4). MOSI is wired up but the encoder doesn't
    //    care what we send.
    spi1.cr1.write(|w| {
        w
            .bidimode().clear_bit()   // 2-line unidirectional
            .rxonly().clear_bit()
            .dff().clear_bit()        // 8-bit frames
            .lsbfirst().clear_bit()   // MSB first
            .ssm().set_bit()          // software slave management
            .ssi().set_bit()          // pretend NSS is high (we're master)
            .mstr().set_bit()         // master mode
            .br().bits(0b101)         // /64 → 1 MHz SCK
            .cpol().clear_bit()       // mode 0
            .cpha().clear_bit()
            .spe().set_bit()          // enable SPI
    });
    spi1.cr2.write(|w| w);            // no interrupts, no DMA, no NSS pulse
}

#[inline(always)]
fn cs_low() {
    let gpioa = unsafe { &*pac::GPIOA::ptr() };
    gpioa.bsrr.write(|w| w.br4().set_bit());
}

#[inline(always)]
fn cs_high() {
    let gpioa = unsafe { &*pac::GPIOA::ptr() };
    gpioa.bsrr.write(|w| w.bs4().set_bit());
}

#[inline(always)]
fn xfer_byte(b: u8) -> u8 {
    let spi1 = unsafe { &*pac::SPI1::ptr() };
    // Wait TXE.
    while spi1.sr.read().txe().bit_is_clear() {}
    spi1.dr.write(|w| w.dr().bits(b as u16));
    // Wait RXNE.
    while spi1.sr.read().rxne().bit_is_clear() {}
    spi1.dr.read().dr().bits() as u8
}

/// CRC-6 over 18 MSB-first bits, poly 0x43, init 0x00, no XOR-out
/// (matches MT6701 datasheet).
fn crc6(data18: u32) -> u8 {
    let mut crc: u32 = 0;
    for i in (0..18).rev() {
        let inb = (data18 >> i) & 1;
        let top = (crc >> 5) & 1;
        crc = (crc << 1) & 0x3F;
        if (inb ^ top) != 0 {
            crc ^= 0x03;
        }
    }
    (crc & 0x3F) as u8
}

/// Read one frame from the encoder. Returns `None` if the SPI transaction
/// produced an obviously-bogus all-zero or all-ones word.
pub fn read() -> Option<Sample> {
    cs_low();
    // Tiny CS setup time. The MT6701 needs ~tCS_lead before the first SCK
    // edge — a few PCLK cycles is plenty at 1 MHz SCK.
    cortex_m::asm::nop();
    cortex_m::asm::nop();
    let b0 = xfer_byte(0x00);
    let b1 = xfer_byte(0x00);
    let b2 = xfer_byte(0x00);
    cs_high();

    let raw: u32 = ((b0 as u32) << 16) | ((b1 as u32) << 8) | (b2 as u32);
    LAST_RAW32.store(raw, core::sync::atomic::Ordering::Relaxed);

    // Reject the obvious fault patterns: bus stuck high (no chip / no power)
    // or bus stuck low (no MISO connection).
    if raw == 0 || raw == 0x00FF_FFFF {
        BUS_FAULTS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
        return None;
    }

    let angle = ((raw >> 10) & ANGLE_MASK) as u16;
    let status = ((raw >> 6) & 0xF) as u8;
    let got_crc = (raw & 0x3F) as u8;
    let want_crc = crc6(raw >> 6);

    // Per-call diagnostic counters — health metrics for live host queries.
    // Status bits (per MT6701 datasheet):
    //   bit3 = push-button, bit2 = no-mag, bit1 = weak-mag, bit0 = strong-mag.
    SAMPLES.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    if got_crc != want_crc {
        CRC_ERRORS.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    }
    if status & 0b0100 != 0 {
        NO_MAG.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    }
    if status & 0b0010 != 0 {
        WEAK_MAG.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
    }

    let s = Sample { angle, status, crc_ok: got_crc == want_crc };
    // Cache the sample for cheap, race-free reads from contexts that
    // can't drive SPI safely (e.g. the main loop while a TIM2-driven
    // ISR also reads the encoder).
    //
    // Encoded as a single u32 written atomically:
    //   bits[31..16] = angle (14 bits used)
    //   bits[15..8]  = status
    //   bit[1]       = crc_ok
    //   bit[0]       = valid
    let packed: u32 =
        ((angle as u32) << 16)
            | ((status as u32) << 8)
            | (if s.crc_ok { 0b10 } else { 0 })
            | 0b01;
    LAST_SAMPLE.store(packed, core::sync::atomic::Ordering::Release);
    Some(s)
}

static LAST_SAMPLE: core::sync::atomic::AtomicU32 =
    core::sync::atomic::AtomicU32::new(0);

/// Most recent 32-bit raw SPI word from the encoder. Lets the host
/// inspect bit alignment without flashing new firmware. Top 24 bits
/// are what `read()` parses as the SSI frame.
static LAST_RAW32: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

pub fn last_raw32() -> u32 {
    LAST_RAW32.load(core::sync::atomic::Ordering::Relaxed)
}

/// Health counters. Wraparound is fine — the host computes deltas
/// over a measurement window.
static SAMPLES:    core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static CRC_ERRORS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static NO_MAG:     core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static WEAK_MAG:   core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static BUS_FAULTS: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Snapshot of all health counters (since boot). Wrapping u32s.
#[derive(Copy, Clone, Default, Debug)]
pub struct Health {
    pub samples: u32,
    pub crc_errors: u32,
    pub no_mag: u32,
    pub weak_mag: u32,
    pub bus_faults: u32,
}

pub fn health() -> Health {
    use core::sync::atomic::Ordering::Relaxed;
    Health {
        samples: SAMPLES.load(Relaxed),
        crc_errors: CRC_ERRORS.load(Relaxed),
        no_mag: NO_MAG.load(Relaxed),
        weak_mag: WEAK_MAG.load(Relaxed),
        bus_faults: BUS_FAULTS.load(Relaxed),
    }
}

/// Returns the most recent sample captured by `read()` from any
/// context. Safe to call from the main loop while an ISR is calling
/// `read()`. Returns `None` if no successful read has happened yet.
pub fn last_sample() -> Option<Sample> {
    let p = LAST_SAMPLE.load(core::sync::atomic::Ordering::Acquire);
    if p & 0b01 == 0 {
        return None;
    }
    Some(Sample {
        angle: (p >> 16) as u16 & 0x3FFF,
        status: ((p >> 8) & 0xF) as u8,
        crc_ok: (p & 0b10) != 0,
    })
}
