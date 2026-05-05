//! candrive bootloader.
//!
//! Phase 1 responsibilities:
//!   1. Clock to 64 MHz (HSI/2 × 16 PLL).
//!   2. Validate the common API table at 0x08001000.
//!   3. If `boot_magic != STAY_IN_BOOTLOADER` and the app's vector table
//!      looks sane, jump to the app.
//!   4. Otherwise, blink PB4 fast and idle (CAN firmware-update support
//!      lands in phase 2).

#![no_std]
#![no_main]

use candrive_shared::boot_magic::{self, STAY_IN_BOOTLOADER};
use candrive_shared::can_frame::CanFrame;
use candrive_shared::common_api::{self, CommonApi};
use candrive_shared::crc32_mpeg2::Crc32Mpeg2;
use candrive_shared::flash_layout::{
    APP_BASE, APP_SIZE, FLASH_PAGE_SIZE, RAM_BASE, RAM_SIZE,
};
use candrive_shared::identity::{self, build_reply, send_blocking, send_discovery_req};
use candrive_shared::protocol::{Command, Message, ProtocolData};
use candrive_shared::user_store::Slot;
use cortex_m::peripheral::SCB;
use cortex_m_rt::entry;
use panic_halt as _;
use stm32f1::stm32f103 as pac;
use stm32f1 as _;   // bring in PAC interrupt symbols for cortex-m-rt

mod clocks;

const LED_PIN: u8 = 4; // PB4 (SYS LED).
const UNASSIGNED_ID: u8 = Slot::UNASSIGNED_ID;
const SEND_SPIN_LIMIT: u32 = 200_000;

#[inline]
fn send(api: &CommonApi, msg: &Message) -> bool {
    send_blocking(api, msg, SEND_SPIN_LIMIT)
}

/// Erase every page that overlaps [APP_BASE, APP_BASE+stream_len).
/// The flash driver self-protects against erasing BL/common.
fn erase_app_range(api: &CommonApi, stream_len: u32) -> i32 {
    let len = stream_len.min(APP_SIZE);
    let pages = (len + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE;
    for i in 0..pages {
        let addr = APP_BASE + i * FLASH_PAGE_SIZE;
        let rc = (api.flash_erase_page)(addr);
        if rc != 0 { return rc; }
    }
    0
}

/// Bootloader-side CAN firmware-update state machine.
///
/// Sequence (host driven):
///   1. BL sends DiscoveryReq on 0x002 carrying the user_store serial.
///      Host replies with `DiscoveryAssign{ serial, assigned_id }`.
///   2. Host sends `Control{ cmd: StreamStart, data: { stream_length } }`.
///      BL erases the app pages, Acks.
///   3. Host sends a sequence of `Control{ cmd: StreamWrite, fragment[7] }`.
///      BL programs each fragment in order (with halfword carry buffer)
///      and Acks every frame.
///   4. After the last fragment, host sends
///      `Control{ cmd: StreamCommit, data: { crc32 } }`.
///      BL recomputes CRC-32/MPEG-2 over the bytes it received and:
///        - if it matches: clear boot_magic, Ack, sys_reset \u2192 next boot
///          lands in the new app.
///        - if it doesn't:  Error frame, drop session, leave the partially
///          programmed app in place (user_store still flags BL mode).
///
/// Bail-outs:
///   - `NetworkReset` or `RevokeConfig` -> abort, stay in BL.
///   - `WATCHDOG_LIMIT` loop iterations without progress -> abort.
fn run_update_mode(api: &CommonApi) -> ! {
    let banner = b"[bl] update mode\r\n";
    (api.usart_write)(banner.as_ptr(), banner.len());

    let identity = identity::load_identity(api);

    if (api.can_init)(1_000_000) != 0 {
        let msg = b"[bl] can_init FAILED\r\n";
        (api.usart_write)(msg.as_ptr(), msg.len());
        loop { led_toggle(); (api.delay_us)(80_000); }
    }
    let msg = b"[bl] can up @ 1Mbit/s\r\n";
    (api.usart_write)(msg.as_ptr(), msg.len());

    let mut my_id: u8 = UNASSIGNED_ID;
    let mut frame = CanFrame::default();
    let mut tick: u32 = 0;
    let mut blink: u32 = 0;
    let mut discovery_counter: u32 = 0;

    // Update-session state.
    let mut update_started = false;
    let mut stream_len: u32 = 0;
    let mut bytes_received: u32 = 0;     // bytes accepted from host
    let mut prog_offset: u32 = 0;        // bytes programmed to flash (halfword aligned)
    let mut pending: [u8; 8] = [0; 8];   // staging for halfword-aligned writes
    let mut pending_len: usize = 0;
    let mut crc = Crc32Mpeg2::new();
    let mut watchdog: u32 = 0;
    const WATCHDOG_LIMIT: u32 = 500_000; // ~5s @ 10us per loop pass

    if (api.flash_unlock)() != 0 {
        let msg = b"[bl] flash unlock FAILED\r\n";
        (api.usart_write)(msg.as_ptr(), msg.len());
    }

    send_discovery_req(api, &identity);

    loop {
        let rc = (api.can_recv)(&mut frame as *mut _);
        if rc == 1 {
            watchdog = 0;
            match Message::decode(&frame) {
                Ok(Some(Message::DiscoveryAssign { serial, assigned_id }))
                    if serial == identity.serial_no =>
                {
                    my_id = assigned_id;
                }
                Ok(Some(Message::Control { device_id, is_controller, cmd, data }))
                    if is_controller && my_id != UNASSIGNED_ID && device_id == my_id =>
                {
                    match (cmd, data) {
                        (Command::GetInfo, _) => {
                            send(api, &build_reply(my_id, Command::GetInfo,
                                ProtocolData::Info {
                                    serial: identity.serial_no,
                                    fw_major: 0, fw_minor: 4, fw_patch: 0,
                                }));
                        }
                        (Command::StreamStart, ProtocolData::StreamStart { stream_length }) => {
                            stream_len = stream_length.min(APP_SIZE);
                            bytes_received = 0;
                            prog_offset = 0;
                            pending_len = 0;
                            crc = Crc32Mpeg2::new();
                            let msg = b"[bl] stream start\r\n";
                            (api.usart_write)(msg.as_ptr(), msg.len());
                            let rc = erase_app_range(api, stream_len);
                            if rc != 0 {
                                send(api, &build_reply(my_id, Command::Error,
                                    ProtocolData::Error { code: 1, message: rc as u32 }));
                            } else {
                                update_started = true;
                                send(api, &build_reply(my_id, Command::Ack,
                                    ProtocolData::Empty));
                            }
                        }
                        (Command::StreamWrite, ProtocolData::StreamFragment(d))
                            if update_started =>
                        {
                            // Accept only as many fragment bytes as the host
                            // still owes us; the rest is filler the host added
                            // to round up to 7.
                            let remaining = stream_len.saturating_sub(bytes_received);
                            let take = (d.len() as u32).min(remaining) as usize;
                            crc.update(&d[..take]);
                            // Stage into `pending`, flush halfword-aligned chunks.
                            let mut rc: i32 = 0;
                            let mut i = 0;
                            while i < take {
                                let cap = pending.len() - pending_len;
                                let n = (take - i).min(cap);
                                pending[pending_len..pending_len + n]
                                    .copy_from_slice(&d[i..i + n]);
                                pending_len += n;
                                i += n;
                                let flush = pending_len & !1; // largest even
                                if flush > 0 {
                                    rc = (api.flash_program)(
                                        APP_BASE + prog_offset,
                                        pending.as_ptr(),
                                        flush,
                                    );
                                    if rc != 0 { break; }
                                    prog_offset += flush as u32;
                                    // shift residual byte (if any) to slot 0
                                    if pending_len > flush {
                                        pending[0] = pending[flush];
                                    }
                                    pending_len -= flush;
                                }
                            }
                            bytes_received = bytes_received.saturating_add(take as u32);
                            // If we've consumed the whole stream, flush a trailing
                            // odd byte by padding with 0xFF.
                            if rc == 0 && bytes_received >= stream_len && pending_len > 0 {
                                pending[1] = 0xFF;
                                rc = (api.flash_program)(
                                    APP_BASE + prog_offset,
                                    pending.as_ptr(),
                                    2,
                                );
                                prog_offset += 2;
                                pending_len = 0;
                            }
                            if rc != 0 {
                                send(api, &build_reply(my_id, Command::Error,
                                    ProtocolData::Error { code: 2, message: rc as u32 }));
                                update_started = false;
                            } else {
                                send(api, &build_reply(my_id, Command::Ack,
                                    ProtocolData::Empty));
                            }
                        }
                        (Command::StreamCommit, ProtocolData::StreamCommit { crc32: host_crc })
                            if update_started && bytes_received >= stream_len =>
                        {
                            let local = crc.finalize();
                            if local == host_crc {
                                let msg = b"[bl] commit OK, rebooting\r\n";
                                (api.usart_write)(msg.as_ptr(), msg.len());
                                send(api, &build_reply(my_id, Command::Ack,
                                    ProtocolData::Empty));
                                (api.flash_lock)();
                                (api.delay_us)(5_000);
                                boot_magic::write(0);
                                SCB::sys_reset();
                            } else {
                                let msg = b"[bl] commit CRC MISMATCH\r\n";
                                (api.usart_write)(msg.as_ptr(), msg.len());
                                send(api, &build_reply(my_id, Command::Error,
                                    ProtocolData::Error { code: 3, message: local }));
                                update_started = false;
                            }
                        }
                        (Command::NetworkReset, _) | (Command::RevokeConfig, _) => {
                            let msg = b"[bl] aborted by host\r\n";
                            (api.usart_write)(msg.as_ptr(), msg.len());
                            update_started = false;
                            my_id = UNASSIGNED_ID;
                        }
                        // Anything else: NACK with a typed Error so the host
                        // gets a definite signal instead of a phantom Ack.
                        _ => {
                            send(api, &build_reply(my_id, Command::Error,
                                ProtocolData::Error { code: 4, message: cmd.to_u8() as u32 }));
                        }
                    }
                }
                _ => {}
            }
        }

        tick = tick.wrapping_add(1);
        blink = blink.wrapping_add(1);
        if blink >= 12_500 {
            blink = 0;
            led_toggle();
        }

        if my_id == UNASSIGNED_ID {
            discovery_counter = discovery_counter.wrapping_add(1);
            if discovery_counter >= 200_000 {
                discovery_counter = 0;
                send_discovery_req(api, &identity);
            }
        } else if update_started {
            watchdog = watchdog.wrapping_add(1);
            if watchdog >= WATCHDOG_LIMIT {
                let msg = b"[bl] update timeout, abort\r\n";
                (api.usart_write)(msg.as_ptr(), msg.len());
                update_started = false;
                watchdog = 0;
            }
        }
        let _ = tick;
        (api.delay_us)(10);
    }
}

fn led_init() {
    let rcc   = unsafe { &*pac::RCC::ptr() };
    let gpiob = unsafe { &*pac::GPIOB::ptr() };
    let afio  = unsafe { &*pac::AFIO::ptr() };

    rcc.apb2enr.modify(|_, w| w.iopben().set_bit().afioen().set_bit());

    // JTAG occupies PB3/PB4 by default — disable JTAG, keep SWD.
    afio.mapr.modify(|_, w| unsafe { w.swj_cfg().bits(0b010) });

    // PB4 push-pull output, 2 MHz (CNF=00, MODE=10 -> 0x2)
    gpiob.crl.modify(|r, w| unsafe {
        let mut v = r.bits();
        v &= !(0xF << (LED_PIN * 4));
        v |=  0x2 << (LED_PIN * 4);
        w.bits(v)
    });
}

fn led_toggle() {
    let gpiob = unsafe { &*pac::GPIOB::ptr() };
    gpiob.odr.modify(|r, w| unsafe { w.bits(r.bits() ^ (1 << LED_PIN)) });
}

fn app_looks_valid() -> bool {
    let vec = APP_BASE as *const u32;
    let sp = unsafe { core::ptr::read_volatile(vec) };
    let reset = unsafe { core::ptr::read_volatile(vec.add(1)) };
    if sp < RAM_BASE || sp > RAM_BASE + RAM_SIZE { return false; }
    if reset & 1 == 0 { return false; }
    if reset < APP_BASE || reset >= APP_BASE + APP_SIZE { return false; }
    true
}

#[inline(never)]
unsafe fn jump_to_app() -> ! {
    let vec = APP_BASE as *const u32;
    let sp    = unsafe { core::ptr::read_volatile(vec) };
    let reset = unsafe { core::ptr::read_volatile(vec.add(1)) };

    cortex_m::interrupt::disable();

    // Disable+clear all NVIC IRQs so we hand the app a quiet system.
    let nvic_icer = 0xE000_E180 as *mut u32;
    let nvic_icpr = 0xE000_E280 as *mut u32;
    for i in 0..8 {
        unsafe {
            core::ptr::write_volatile(nvic_icer.add(i), 0xFFFF_FFFF);
            core::ptr::write_volatile(nvic_icpr.add(i), 0xFFFF_FFFF);
        }
    }

    // Point VTOR at the app and barrier.
    unsafe { (*SCB::PTR).vtor.write(APP_BASE) };
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    // Set MSP and branch to reset handler — never returns.
    unsafe { cortex_m::asm::bootstrap(sp as *const u32, reset as *const u32) }
}

#[entry]
fn main() -> ! {
    clocks::init_64mhz();

    let common_ok = common_api::is_valid();
    if common_ok {
        let api = unsafe { common_api::get() };
        (api.usart_init)(115_200);
        let banner = b"\r\n[bl] candrive bootloader\r\n";
        (api.usart_write)(banner.as_ptr(), banner.len());
    }

    led_init();

    let magic = boot_magic::read();
    boot_magic::write(0);

    if magic != STAY_IN_BOOTLOADER && app_looks_valid() {
        if common_ok {
            let api = unsafe { common_api::get() };
            let msg = b"[bl] jumping to app\r\n";
            (api.usart_write)(msg.as_ptr(), msg.len());
        }
        unsafe { jump_to_app() };
    }

    // Either the app explicitly asked us to stay (firmware update), or the
    // app image is invalid/erased. In both cases run the CAN update server.
    if !common_ok {
        // No common API -> can't talk CAN/USART. Fall back to LED blink.
        loop {
            led_toggle();
            for _ in 0..1_000_000 { cortex_m::asm::nop(); }
        }
    }
    let api = unsafe { common_api::get() };
    if magic == STAY_IN_BOOTLOADER {
        let msg = b"[bl] stay magic set, entering update mode\r\n";
        (api.usart_write)(msg.as_ptr(), msg.len());
    } else {
        let msg = b"[bl] no valid app, entering update mode\r\n";
        (api.usart_write)(msg.as_ptr(), msg.len());
    }
    run_update_mode(api);
}
