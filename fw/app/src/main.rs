//! candrive application.
//!
//! Phase 2: protocol responder. The app polls the CAN RX FIFO via the
//! common API, decodes incoming control frames, and replies to a small
//! initial set of commands:
//!   - `GetInfo`     -> firmware version + serial number
//!   - `GetInfoExt`  -> runtime flags + temperature (stubbed)
//!   - `GetStatus`   -> all-zero status bits for now
//!   - others        -> Ack
//!
//! Position/motor/FOC handling lands in phase 5 once the FOC driver is
//! up. For now this is enough to prove the protocol round-trips on real
//! hardware against `tools/candrive-cli`.

#![no_std]
#![no_main]

use candrive_shared::boot_magic::{self, NORMAL_BOOT, STAY_IN_BOOTLOADER};
use candrive_shared::can_frame::CanFrame;
use candrive_shared::common_api::{self, CommonApi};
use candrive_shared::flash_layout::APP_BASE;
use candrive_shared::identity::{self, build_reply};
use candrive_shared::protocol::{Command, Message, ProtocolData, StatusBits};
use candrive_shared::user_store::Slot;
use cortex_m::peripheral::SCB;
use cortex_m_rt::entry;
use panic_halt as _;
use stm32f1::stm32f103 as pac;
use stm32f1::stm32f103::interrupt;
use stm32f1 as _;

mod clocks;
mod control;
mod encoder;
mod foc;
mod iwdg;
mod led;
mod motor;
mod motor_pwm;
mod spin_test;

// Pre-PWM fallback heartbeat pin. Used only on the early-fault path
// where the common API is missing and we can't bring up TIM3 / CAN.
// Once `led::init` runs PB5 is reconfigured to TIM3 CH2 AF, replacing
// this plain-GPIO mode.
const LED_PIN: u8 = 5; // PB5 (STAT LED).
const FW_MAJOR: u8 = 0;
const FW_MINOR: u8 = 5;
const FW_PATCH: u8 = 0;

const UNASSIGNED_ID: u8 = Slot::UNASSIGNED_ID;

fn led_init() {
    let rcc   = unsafe { &*pac::RCC::ptr() };
    let gpiob = unsafe { &*pac::GPIOB::ptr() };
    rcc.apb2enr.modify(|_, w| w.iopben().set_bit());
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

/// Outcome from the message handler. We can't piggy-back the "reset into
/// bootloader" decision on the reply slot because we still want the Ack to
/// reach the host before we reboot.
struct Action {
    reply: Option<Message>,
    reboot_into_bl: bool,
    persist_id: bool,
    save_params: bool,
}

impl Action {
    fn nothing() -> Self { Self { reply: None, reboot_into_bl: false, persist_id: false, save_params: false } }
    fn reply(m: Message) -> Self { Self { reply: Some(m), reboot_into_bl: false, persist_id: false, save_params: false } }
}

/// Dispatch one received frame.
fn handle(api: &CommonApi, msg: Message, identity: &mut Slot) -> Action {
    match msg {
        Message::DiscoveryAssign { serial, assigned_id } => {
            if serial == identity.serial_no && identity.assigned_id != assigned_id {
                identity.assigned_id = assigned_id;
                Action {
                    reply: None,
                    reboot_into_bl: false,
                    persist_id: true,                    save_params: false,                }
            } else {
                Action::nothing()
            }
        }
        Message::Control { device_id, is_controller, cmd, data } => {
            if !is_controller || device_id != identity.assigned_id
                || identity.assigned_id == UNASSIGNED_ID
            {
                return Action::nothing();
            }
            // Capture `data` for branches that need it (motor params),
            // ignore it for the simpler get/ack handlers.
            let _ = &data;
            match cmd {
                Command::GetInfo => Action::reply(build_reply(identity.assigned_id, Command::GetInfo,
                    ProtocolData::Info {
                        serial: identity.serial_no,
                        fw_major: FW_MAJOR, fw_minor: FW_MINOR, fw_patch: FW_PATCH,
                    })),
                Command::GetInfoExt => Action::reply(build_reply(identity.assigned_id, Command::GetInfoExt,
                    ProtocolData::InfoExt { flags: 0, temperature: 0 })),
                Command::GetStatus => Action::reply(build_reply(identity.assigned_id, Command::GetStatus,
                    ProtocolData::Status(StatusBits::default()))),
                Command::GetPosition => {
                    // Single-turn 14-bit angle (0..16383). The
                    // multi-turn accumulator is exposed via
                    // GetMotorParam(132) for callers that need it.
                    let angle = encoder::last_sample().map(|s| s.angle).unwrap_or(0);
                    Action::reply(build_reply(identity.assigned_id, Command::GetPosition,
                        ProtocolData::Position { value: angle }))
                }
                Command::FirmwareUpdate => Action {
                    reply: Some(build_reply(identity.assigned_id, Command::Ack, ProtocolData::Empty)),
                    reboot_into_bl: true,
                    persist_id: false,
                    save_params: false,
                },
                Command::GetMotorParam => {
                    let idx = match data {
                        ProtocolData::MotorParam { index, .. } => index,
                        _ => 0,
                    };
                    let val = control::debug_get(idx)
                        .or_else(|| motor::read_field(idx))
                        .unwrap_or(f32::NAN);
                    Action::reply(build_reply(identity.assigned_id, Command::GetMotorParam,
                        ProtocolData::MotorParam { index: idx, value: val }))
                }
                Command::SetMotorParam => {
                    if let ProtocolData::MotorParam { index, value } = data {
                        let ok = motor::write_field(index, value);
                        let stored = if ok { value } else { f32::NAN };
                        Action::reply(build_reply(identity.assigned_id, Command::SetMotorParam,
                            ProtocolData::MotorParam { index, value: stored }))
                    } else {
                        Action::reply(build_reply(identity.assigned_id, Command::Ack, ProtocolData::Empty))
                    }
                }
                Command::RunCalibration => {
                    // Pole pairs come from motor_param[0] (host-set,
                    // authoritative). This routine only locks the rotor
                    // at theta_e=0 and latches the encoder reading,
                    // which it stores into motor_param[11]
                    // (electrical_zero_offset). The protocol still
                    // returns a `pole_pairs` field for host convenience —
                    // we just echo back the configured value.
                    let (dur_ms, valign_frac) = match data {
                        ProtocolData::CalibrationReq { dur_ms, valign_pct, .. } => {
                            (dur_ms as u32, (valign_pct.min(100) as f32) * 0.01)
                        }
                        _ => (1000_u32, 0.25_f32),
                    };
                    let vbus = motor::read_field(3).unwrap_or(12.0);
                    // foc::calibrate drives the bridge directly and
                    // calls encoder::read() from thread context.
                    // Mask the TIM2 ISR for the duration so it doesn't
                    // race on SPI1 / fight for CCR ownership.
                    motor_pwm::disable_update_irq();
                    let res = foc::calibrate(api.delay_us, vbus * valign_frac, dur_ms);
                    motor_pwm::enable_update_irq();
                    if !res.fault {
                        motor::write_field(11, res.zero_offset_counts as f32);
                        motor::write_field(13, res.direction as f32);
                    }
                    let mut flags = 0u8;
                    if res.fault { flags |= 0x01; }
                    if res.direction < 0 { flags |= 0x02; }
                    let pp = motor::read_field(0).unwrap_or(0.0);
                    Action::reply(build_reply(identity.assigned_id, Command::RunCalibration,
                        ProtocolData::CalibrationResult {
                            pole_pairs: pp,
                            zero_offset: res.zero_offset_counts,
                            flags,
                        }))
                }
                Command::SetMotorCommand => {
                    if let ProtocolData::MotorCommand { mode, target } = data {
                        control::set_command(mode, target);
                        Action::reply(build_reply(identity.assigned_id, Command::SetMotorCommand,
                            ProtocolData::MotorCommand { mode, target }))
                    } else {
                        Action::reply(build_reply(identity.assigned_id, Command::Ack, ProtocolData::Empty))
                    }
                }
                Command::SaveMotorParams => Action {
                    // Defer the actual flash op to the main loop where
                    // we can keep IWDG petted; reply Ack here.
                    reply: Some(build_reply(identity.assigned_id, Command::SaveMotorParams,
                        ProtocolData::Empty)),
                    reboot_into_bl: false,
                    persist_id: false,
                    save_params: true,
                },
                Command::SetLed => {
                    if let ProtocolData::Led { sys, stat, update_flag } = data {
                        led::apply_set(sys, stat, update_flag);
                    }
                    Action::reply(build_reply(identity.assigned_id, Command::SetLed,
                        ProtocolData::Led {
                            sys:  led::get(led::LED_SYS),
                            stat: led::get(led::LED_STAT),
                            update_flag: 0,
                        }))
                }
                Command::GetLed => Action::reply(build_reply(identity.assigned_id, Command::GetLed,
                    ProtocolData::Led {
                        sys:  led::get(led::LED_SYS),
                        stat: led::get(led::LED_STAT),
                        update_flag: 0,
                    })),
                _ => Action::reply(build_reply(identity.assigned_id, Command::Ack, ProtocolData::Empty)),
            }
        }
        _ => Action::nothing(),
    }
}

fn send_discovery_req(api: &CommonApi, identity: &Slot) {
    identity::send_discovery_req(api, identity);
}

#[entry]
fn main() -> ! {
    unsafe { (*SCB::PTR).vtor.write(APP_BASE) };
    cortex_m::asm::dsb();
    cortex_m::asm::isb();

    // Capture & clear reset-cause flags before anything else touches them.
    // Bits in RCC_CSR (RM0008 §7.3.10):
    //   31 LPWRRSTF, 30 WWDGRSTF, 29 IWDGRSTF, 28 SFTRSTF,
    //   27 PORRSTF,  26 PINRSTF,  25 RMVF
    let reset_csr: u32 = unsafe {
        let rcc = &*pac::RCC::ptr();
        let csr = rcc.csr.read().bits();
        // RMVF is bit 24 — write to clear all reset flags.
        rcc.csr.modify(|_, w| w.rmvf().set_bit());
        csr
    };

    clocks::init_64mhz();
    boot_magic::write(NORMAL_BOOT);
    led_init();
    // IWDG ASAP so any later hang gets caught. ~500 ms timeout.
    iwdg::init();

    if !common_api::is_valid() {
        // No common API → can't do USART or CAN. Fall back to LED blink.
        loop {
            led_toggle();
            for _ in 0..2_000_000 { cortex_m::asm::nop(); }
        }
    }
    let api = unsafe { common_api::get() };

    (api.usart_init)(115_200);
    let banner = b"\r\n[app] candrive v0.5.0\r\n";
    (api.usart_write)(banner.as_ptr(), banner.len());

    // Decode reset cause. The bootloader runs first after every reset
    // and re-enters here, so this captures the original system reset.
    let mut rbuf = [0u8; 64];
    rbuf[..7].copy_from_slice(b"[app] r");
    rbuf[7] = b'='; rbuf[8] = b'0'; rbuf[9] = b'x';
    let mut h = (reset_csr >> 24) as u8; // top byte holds all flags
    for i in 0..2 {
        let nib = (h >> 4) & 0xF;
        rbuf[10 + i] = if nib < 10 { b'0' + nib } else { b'a' + nib - 10 };
        h <<= 4;
    }
    rbuf[12] = b' ';
    let mut n: usize = 13;
    let tag = |buf: &mut [u8], n: &mut usize, s: &[u8]| {
        let avail = buf.len() - *n;
        let take = s.len().min(avail);
        buf[*n..*n + take].copy_from_slice(&s[..take]);
        *n += take;
    };
    if reset_csr & (1 << 31) != 0 { tag(&mut rbuf, &mut n, b"LPWR "); }
    if reset_csr & (1 << 30) != 0 { tag(&mut rbuf, &mut n, b"WWDG "); }
    if reset_csr & (1 << 29) != 0 { tag(&mut rbuf, &mut n, b"IWDG "); }
    if reset_csr & (1 << 28) != 0 { tag(&mut rbuf, &mut n, b"SFT  "); }
    if reset_csr & (1 << 27) != 0 { tag(&mut rbuf, &mut n, b"POR  "); }
    if reset_csr & (1 << 26) != 0 { tag(&mut rbuf, &mut n, b"PIN  "); }
    rbuf[n] = b'\r'; rbuf[n + 1] = b'\n';
    (api.usart_write)(rbuf.as_ptr(), n + 2);

    // Stash for read-back over CAN as debug param 200.
    control::set_reset_csr((reset_csr >> 24) as u8);

    let mut identity = identity::load_identity(api);

    // Restore previously-saved motor params (if any) before the control
    // loop spins up, so calibrated zero-offset / direction and any user
    // tunes survive a reset.
    if let Some(record) = identity::load_params(api) {
        let n = (record.count as usize).min(record.values.len());
        motor::apply_values(&record.values[..n]);
        let msg = b"[app] params: restored from flash\r\n";
        (api.usart_write)(msg.as_ptr(), msg.len());
    } else {
        let msg = b"[app] params: factory defaults\r\n";
        (api.usart_write)(msg.as_ptr(), msg.len());
    }

    if (api.can_init)(1_000_000) != 0 {
        let msg = b"[app] can_init FAILED\r\n";
        (api.usart_write)(msg.as_ptr(), msg.len());
    }

    encoder::init();
    motor_pwm::init();
    // motor_pwm::init programs SWJ_CFG=010 (JTAG off, SWD on) which
    // frees PB4 (JTRST) for AF use. led::init must run after that.
    led::init();
    // Synchronize encoder sampling + control step to the TIM2 update
    // event. The handler (#[interrupt] fn TIM2 below) divides the
    // 50 kHz PWM update rate down to a deterministic control-loop
    // rate, eliminating the variable phase that the free-running
    // main-loop polling produced (which we suspect was creating a
    // limit cycle invisible to a 1 kHz scope but enough to lock the
    // closed-loop FOC at the cal-aligned d-axis).
    motor_pwm::enable_update_irq();
    // The bootloader disables interrupts (PRIMASK=1) before jumping
    // here. cortex-m-rt's #[entry] does not re-enable them, so do it
    // explicitly now that the TIM2 ISR is wired up.
    unsafe { cortex_m::interrupt::enable(); }

    // FOC bring-up note: foc::apply_voltage and foc::calibrate exist in
    // app/src/foc.rs but are not run at boot. Pole-pair calibration is
    // fragile (rotor cogs into a detent at low voltage / low velocity)
    // so we expose it as an on-demand host-triggered command instead
    // of running it during startup. See foc.rs for details. Until that
    // command lands the bridge stays disabled and the app behaves as
    // a pure protocol responder + encoder reader.
    let _ = spin_test::run; // suppress dead_code while we iterate
    let _ = foc::calibrate; // ditto

    let mut frame = CanFrame::default();
    let mut hb_counter: u32 = 0;
    let mut discovery_counter: u32 = 0;
    let mut enc_log_counter: u32 = 0;
    let mut last_angle: u16 = 0;
    let mut last_status: u8 = 0;

    // Kick off discovery only if we don't already have an assigned id.
    if identity.assigned_id == UNASSIGNED_ID {
        send_discovery_req(api, &identity);
    }

    loop {
        // Service one inbound frame per pass.
        let rc = (api.can_recv)(&mut frame as *mut _);
        if rc == 1 {
            if let Ok(Some(m)) = Message::decode(&frame) {
                let act = handle(api, m, &mut identity);
                if act.persist_id {
                    let _ = identity::persist_identity(api, &mut identity);
                }
                if act.save_params {
                    let mut buf = [0.0f32; 32];
                    let n = motor::snapshot(&mut buf);
                    let _ = identity::persist_params(api, &buf[..n]);
                }
                if let Some(reply) = act.reply {
                    if let Ok(out) = reply.encode() {
                        let mut spins = 0u32;
                        while (api.can_send)(&out as *const _) == 0 && spins < 100_000 {
                            core::hint::spin_loop();
                            spins += 1;
                        }
                    }
                }
                if act.reboot_into_bl {
                    let msg = b"[app] entering BL for fw update\r\n";
                    (api.usart_write)(msg.as_ptr(), msg.len());
                    (api.delay_us)(5_000);
                    boot_magic::write(STAY_IN_BOOTLOADER);
                    cortex_m::peripheral::SCB::sys_reset();
                }
            }
        }

        // LED control is host-driven via SetLed; no firmware heartbeat
        // toggle here. The CAN GetStatus heartbeat (~1 Hz, below) plus
        // [enc] USART traffic are the in-firmware liveness signals.

        // Encoder sampling and control::step() now run from the TIM2
        // update ISR (see below). Main loop just observes the cached
        // last sample for diagnostics.
        if let Some(s) = encoder::last_sample() {
            last_angle = s.angle;
            last_status = s.status;
        }

        enc_log_counter = enc_log_counter.wrapping_add(1);
        if enc_log_counter >= 100_000 {
            enc_log_counter = 0;
            // "[enc] a=NNNNN st=N\r\n"
            let mut buf = [0u8; 24];
            buf[..6].copy_from_slice(b"[enc] ");
            buf[6] = b'a'; buf[7] = b'=';
            // 14-bit angle fits in 5 decimal digits.
            let mut a = last_angle as u32;
            let mut digits = [0u8; 5];
            for i in (0..5).rev() {
                digits[i] = b'0' + (a % 10) as u8;
                a /= 10;
            }
            buf[8..13].copy_from_slice(&digits);
            buf[13] = b' '; buf[14] = b's'; buf[15] = b't'; buf[16] = b'=';
            buf[17] = b'0' + (last_status & 0xF);
            buf[18] = b'\r'; buf[19] = b'\n';
            (api.usart_write)(buf.as_ptr(), 20);
        }

        if identity.assigned_id == UNASSIGNED_ID {
            // Re-issue DiscoveryReq every ~2s until assigned.
            discovery_counter = discovery_counter.wrapping_add(1);
            if discovery_counter >= 200_000 {
                discovery_counter = 0;
                send_discovery_req(api, &identity);
            }
        } else {
            // CAN heartbeat ~1 Hz once we have an ID.
            hb_counter = hb_counter.wrapping_add(1);
            if hb_counter >= 100_000 {
                hb_counter = 0;
                let hb = build_reply(identity.assigned_id, Command::GetStatus,
                    ProtocolData::Status(StatusBits::default()));
                if let Ok(out) = hb.encode() {
                    let _ = (api.can_send)(&out as *const _);
                }
            }
        }
        // Pet the watchdog at the bottom of every main-loop pass.
        iwdg::pet();
        (api.delay_us)(10);
    }
}

/// TIM2 update-event ISR. Fires every PWM period (≈20 µs at 50 kHz
/// center-aligned). We divide down to ~10 kHz for the control loop:
/// that gives a 100 µs budget per tick, comfortably above the ~24 µs
/// MT6701 SSI read plus FOC math, while staying well above the
/// motor's mechanical bandwidth.
///
/// Running encoder sampling + apply_voltage from this single deterministic
/// context replaces the prior free-running main-loop polling, which had
/// variable phase relative to the PWM update event and was the suspected
/// root cause of the closed-loop FOC lock.
const CONTROL_DIVIDER: u32 = 5; // 50 kHz / 5 = 10 kHz control rate.

#[interrupt]
fn TIM2() {
    static mut TICK: u32 = 0;

    motor_pwm::ack_update_irq();

    *TICK = TICK.wrapping_add(1);
    if *TICK >= CONTROL_DIVIDER {
        *TICK = 0;
        // control::step() is self-contained: reads the encoder, runs
        // the active mode's math, writes the new CCRs. The CCRs latch
        // at the next TIM2 update event (one PWM period from now),
        // giving a deterministic 20 µs sample-to-output delay.
        control::step();
    }
}
