//! Runtime-tunable motor parameters.
//!
//! Defaults target the **GM3506** gimbal motor, but every field is
//! settable at runtime via the `SetMotorParam` protocol command and
//! readable via `GetMotorParam`. The host references each scalar by
//! a stable index — never reorder; only append.
//!
//! Persistence
//! -----------
//! These live in RAM only for now. Once the production-tooling milestone
//! lands we'll back them with a second `user_store` slot so they survive
//! reflash. The protocol API is unchanged when that happens — only the
//! load/save plumbing in `app/src/main.rs`.
//!
//! GM3506 reference values (from datasheet + the legacy firmware):
//!
//! | Index | Field             | Units    | GM3506 default |
//! |------:|-------------------|----------|---------------:|
//! |   0   | `pole_pairs`      | count    | 11             |
//! |   1   | `phase_resistance`| ohm      | 5.65           |
//! |   2   | `kv`              | rpm/V    | 100            |
//! |   3   | `voltage_supply`  | V        | 12.0           |
//! |   4   | `voltage_limit`   | V        | 2.0            |
//! |   5   | `current_limit`   | A        | 1.0            |
//! |   6   | `velocity_limit`  | rad/s    | 600·2π         |
//! |   7   | `pid_vel_p`       | —        | 0.5            |
//! |   8   | `pid_vel_i`       | —        | 10.0           |
//! |   9   | `pid_vel_d`       | —        | 0.0            |
//! |  10   | `pid_pos_p`       | —        | 20.0           |
//! |  11   | `electrical_zero_offset` | counts (raw 14-bit) | 0 |
//! |  12   | `pid_pos_d`       | V/(counts/tick) | 0.05     |
//! |  13   | `direction`       | ±1.0     | +1.0           |
//! |  14   | `pid_pos_i`       | V/(rad·s) | 2.0           |
//! |  15   | `hold_voltage_limit` | V    | 1.5            |
//! |  16   | `traj_v_max`      | rad/s    | 10.0           |
//! |  17   | `traj_a_max`      | rad/s²   | 50.0           |
//! |  18   | `traj_follow_err` | rad      | 0.3            |
//!
//! `direction` is the sign of (encoder-counts-per-mech-radian) relative
//! to the phase-rotation direction commanded by FOC. It is auto-detected
//! during `RunCalibration` and stored alongside `electrical_zero_offset`.
//! `electrical_zero_offset` is the raw encoder reading at electrical
//! angle 0. It is rotor-mounting-dependent so it must be calibrated
//! once per assembled unit (see `RunCalibration` host command). All
//! other fields are nameplate / tuning values that the host sets
//! authoritatively; the device never auto-derives them.

#[derive(Clone, Copy, Debug)]
pub struct MotorParams {
    pub pole_pairs:       f32, // stored as f32 for uniform protocol; cast to int when used
    pub phase_resistance: f32,
    pub kv:               f32,
    pub voltage_supply:   f32,
    pub voltage_limit:    f32,
    pub current_limit:    f32,
    pub velocity_limit:   f32,
    pub pid_vel_p:        f32,
    pub pid_vel_i:        f32,
    pub pid_vel_d:        f32,
    pub pid_pos_p:        f32,
    pub electrical_zero_offset: f32, // raw 14-bit encoder counts at theta_e=0
    pub pid_pos_d:        f32, // V per (rad/s) damping on position loop
    pub direction:        f32, // ±1.0; auto-detected during calibration
    pub pid_pos_i:        f32, // V/(rad·s); bounded integrator for hold droop
    pub hold_voltage_limit: f32, // sustained heat budget (V); clamps pos_i
    pub traj_v_max:       f32, // trajectory cruise speed (rad/s)
    pub traj_a_max:       f32, // trajectory accel limit (rad/s²)
    pub traj_follow_err:  f32, // freeze traj when |angle_acc-traj_setpoint|>this (rad)
}

impl MotorParams {
    /// GM3506 factory defaults.
    ///
    /// `pole_pairs = 22` was determined empirically: a slow open-loop
    /// `theta_e` sweep at 0.5 rad/s electrical produced a 22.5:1 ratio
    /// between commanded electrical advance and observed mechanical
    /// advance, and FOC commutation only stays locked when the math
    /// uses `theta_e = mech * 22 * dir`. This is consistent either
    /// with the rotor having 44 magnets (22 PP true), or with the
    /// rotor having 22 magnets (11 PP) plus a 2-PP shaft magnet on
    /// the MT6701 encoder reading angle at 2× true mechanical rate.
    /// Either way, 22 is the value that makes the closed-loop math
    /// self-consistent for this motor + encoder pairing.
    pub const fn gm3506() -> Self {
        Self {
            pole_pairs:       22.0,
            phase_resistance: 5.65,
            kv:               100.0,
            voltage_supply:   12.0,
            // Conservative cap: GM3506 phase R ~5.65 Ω, so 2.0 V ≈
            // 0.35 A stall ≈ 0.7 W per phase. User can raise via
            // SetMotorParam(4, ...) once thermal headroom is known.
            voltage_limit:    2.0,
            current_limit:    1.0,
            // Mech rad/s. The cascaded position loop uses this both as
            // an absolute clamp on the velocity command and as the
            // basis for sizing the position integrator clamp. Default
            // is sized for typical hands-on, low-speed actuator use;
            // raise via SetMotorParam(6, …) for high-speed apps.
            // Position/velocity feedback is in *true mechanical rad*
            // (8192 enc counts/rev → τ rad/rev). Earlier gains assumed
            // the encoder-counts-per-rev = 16384 case, so all gains
            // touching angle or velocity are halved here vs the legacy
            // values to keep loop response identical.
            velocity_limit:   15.0,
            pid_vel_p:        0.25,
            pid_vel_i:        5.0,
            pid_vel_d:        0.0,
            pid_pos_p:        10.0,
            electrical_zero_offset: 0.0,
            pid_pos_d:        0.025,
            direction:        1.0,
            pid_pos_i:        1.0,
            hold_voltage_limit: 1.5,
            traj_v_max:       5.0,
            traj_a_max:       25.0,
            traj_follow_err:  0.3,
        }
    }

    /// Returns `Some(value)` for known indices, `None` otherwise.
    pub fn get(&self, index: u8) -> Option<f32> {
        Some(match index {
            0  => self.pole_pairs,
            1  => self.phase_resistance,
            2  => self.kv,
            3  => self.voltage_supply,
            4  => self.voltage_limit,
            5  => self.current_limit,
            6  => self.velocity_limit,
            7  => self.pid_vel_p,
            8  => self.pid_vel_i,
            9  => self.pid_vel_d,
            10 => self.pid_pos_p,
            11 => self.electrical_zero_offset,
            12 => self.pid_pos_d,
            13 => self.direction,
            14 => self.pid_pos_i,
            15 => self.hold_voltage_limit,
            16 => self.traj_v_max,
            17 => self.traj_a_max,
            18 => self.traj_follow_err,
            _  => return None,
        })
    }

    /// Returns true if the index was recognised and the value stored.
    pub fn set(&mut self, index: u8, value: f32) -> bool {
        match index {
            0  => self.pole_pairs       = value,
            1  => self.phase_resistance = value,
            2  => self.kv               = value,
            3  => self.voltage_supply   = value,
            4  => self.voltage_limit    = value,
            5  => self.current_limit    = value,
            6  => self.velocity_limit   = value,
            7  => self.pid_vel_p        = value,
            8  => self.pid_vel_i        = value,
            9  => self.pid_vel_d        = value,
            10 => self.pid_pos_p        = value,
            11 => self.electrical_zero_offset = value,
            12 => self.pid_pos_d        = value,
            13 => self.direction        = if value >= 0.0 { 1.0 } else { -1.0 },
            14 => self.pid_pos_i        = value,
            15 => self.hold_voltage_limit = value,
            16 => self.traj_v_max       = value,
            17 => self.traj_a_max       = value,
            18 => self.traj_follow_err  = value,
            _  => return false,
        }
        true
    }
}

/// Number of distinct field indices recognised by `get`/`set`. Bump
/// when adding fields. Mirrored by `params_store::MAX_PARAMS` (which
/// must remain >= this value); the field count is also written into
/// each persisted record so older records load correctly after a
/// firmware update that adds fields.
pub const FIELD_COUNT: u8 = 19;

/// Shared motor parameter state. Single-threaded access from the main
/// loop only — no atomics needed.
static mut PARAMS: MotorParams = MotorParams::gm3506();

/// SAFETY: callers run on the single main-loop thread. Do not call from
/// interrupts (none configured today). Returns a copy.
pub fn read() -> MotorParams {
    unsafe { core::ptr::read(core::ptr::addr_of!(PARAMS)) }
}

/// SAFETY: same single-thread invariant as `read()`.
pub fn read_field(index: u8) -> Option<f32> {
    read().get(index)
}

/// Returns true on a known index. SAFETY: same single-thread invariant.
pub fn write_field(index: u8, value: f32) -> bool {
    let mut params = read();
    let ok = params.set(index, value);
    if ok {
        unsafe { core::ptr::write(core::ptr::addr_of_mut!(PARAMS), params) };
    }
    ok
}

/// Snapshot every field into `out[0..FIELD_COUNT]`. Returns the number
/// of fields written (== `FIELD_COUNT`). Used by the persistence layer
/// when packing a fresh flash record.
pub fn snapshot(out: &mut [f32]) -> usize {
    let p = read();
    let n = (FIELD_COUNT as usize).min(out.len());
    for i in 0..n {
        out[i] = p.get(i as u8).unwrap_or(0.0);
    }
    n
}

/// Apply every field in `values` (in field-index order) into the live
/// parameter state. Used at boot to restore from flash. Unknown fields
/// past `FIELD_COUNT` are silently ignored so older firmware loading a
/// record written by newer firmware degrades cleanly.
pub fn apply_values(values: &[f32]) {
    let mut params = read();
    for (i, v) in values.iter().enumerate() {
        if !v.is_finite() { continue; }
        let _ = params.set(i as u8, *v);
    }
    unsafe { core::ptr::write(core::ptr::addr_of_mut!(PARAMS), params) };
}
