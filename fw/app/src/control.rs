//! Closed-loop control state machine.
//!
//! Single-thread state, polled from the main loop. Modes:
//!
//!   * `Idle`     — bridge disabled, nothing applied.
//!   * `Voltage`  — voltage-mode FOC. `target` is Vq in volts (signed);
//!                   Vd = 0. Useful for torque demos and verifying the
//!                   FOC signal chain end-to-end.
//!   * `Velocity` — velocity-mode FOC. `target` is mechanical angular
//!                   velocity in rad/s. PI on (target - vel) with I-term
//!                   clamp anti-windup. Output is Vq.
//!   * `Position` — multi-turn position loop, cascaded into the velocity
//!                   loop. `target` is mechanical angle in radians;
//!                   actual angle is accumulated across encoder wraps so
//!                   commands of any magnitude work.
//!
//! Velocity uses the DWT cycle counter (already enabled by `clocks::init`)
//! for dt, so `vel` is in rad/s — not counts/tick.
//!
//! Electrical angle:
//!     elec_counts = ((mech - offset) * pp · direction) mod 16384
//!     theta_e     = elec_counts * (2π / 16384)

use core::ptr::{addr_of, addr_of_mut};

use crate::encoder;
use crate::foc;
use crate::motor;
use crate::motor_pwm;

const TAU: f32 = core::f32::consts::TAU;

/// Encoder counts per mechanical revolution (14-bit MT6701).
const COUNTS_PER_REV: i32 = 16384;
/// Radians per count.
const RAD_PER_COUNT: f32 = TAU / 16384.0;
/// HCLK frequency, locked at 64 MHz by `clocks::init`.
const HCLK_HZ: f32 = 64_000_000.0;

/// DWT->CYCCNT register (free-running 32-bit cycle counter).
const DWT_CYCCNT: *const u32 = 0xE000_1004 as *const u32;

#[inline]
fn cyccnt() -> u32 {
    // SAFETY: read-only volatile load of a memory-mapped peripheral.
    unsafe { core::ptr::read_volatile(DWT_CYCCNT) }
}

#[derive(Copy, Clone, Debug)]
pub enum Mode {
    Idle,
    Voltage,
    Velocity,
    Position,
    /// Diagnostic open-loop sweep. `target` = commanded electrical
    /// angular velocity (rad/s). Applies (Vd=vlim, Vq=0) at an
    /// integrated `theta_cmd`. Used to verify FOC alignment / cal
    /// quality: if cal is correct, the rotor follows theta_cmd in
    /// lockstep and the *measured* theta_e = theta_cmd (mod 2π).
    OpenLoop,
}

#[derive(Copy, Clone)]
struct State {
    mode:        Mode,
    target:      f32,
    /// Last raw mech angle in counts; `i32::MIN` = uninitialised.
    last_counts: i32,
    /// Last DWT cycle-counter sample.
    last_cycles: u32,
    /// Multi-turn mechanical position in encoder counts (i32, signed,
    /// already in user-positive direction). Each tick we compute a
    /// wrapped, glitch-filtered single-turn delta and add it here, so
    /// position is tracked exactly with no float drift. The float
    /// `angle_acc` is derived from this every tick.
    mech_total:  i32,
    /// Cumulative mechanical angle (rad). Derived each tick from
    /// `mech_total` so float accumulation drift is impossible.
    angle_acc:   f32,
    /// Filtered angular velocity (rad/s), 1-pole IIR, updated only
    /// when the velocity sampling window expires (see `vel_acc_dt`).
    vel:         f32,
    /// Velocity-loop integrator output (volts), clamped each tick.
    vel_i:       f32,
    /// Position-loop integrator (volts). Bounded by `hold_voltage_limit`
    /// for sustained-heat safety. Only integrates when |traj_v| is small
    /// (i.e. during hold), to avoid winding up during transit.
    pos_i:       f32,
    /// Trajectory generator state: current commanded setpoint (rad).
    /// Drives the PD inner loop. Slewed toward `target` at
    /// `traj_a_max` accel, capped at `traj_v_max` cruise.
    traj_setpoint: f32,
    /// Trajectory generator state: current commanded setpoint velocity
    /// (rad/s, signed).
    traj_v:      f32,
    /// Accumulated mech-count delta within the current velocity window.
    vel_acc_d:   i32,
    /// Accumulated time within the current velocity window (s).
    vel_acc_dt:  f32,
    /// Open-loop integrated electrical angle command (rad), wrapped to
    /// [0, 2π). Only used by `Mode::OpenLoop`.
    theta_cmd:   f32,
}

impl State {
    const fn idle() -> Self {
        Self {
            mode: Mode::Idle, target: 0.0,
            last_counts: i32::MIN, last_cycles: 0,
            mech_total: 0,
            angle_acc: 0.0, vel: 0.0, vel_i: 0.0,
            pos_i: 0.0, traj_setpoint: 0.0, traj_v: 0.0,
            vel_acc_d: 0, vel_acc_dt: 0.0,
            theta_cmd: 0.0,
        }
    }
}

static mut STATE: State = State::idle();

/// Last applied Vq (for diagnostics).
static mut LAST_VQ: f32 = 0.0;
/// Last raw mech_counts read from encoder (for diagnostics).
static mut LAST_MECH: i32 = 0;
/// Last theta_e (for diagnostics).
static mut LAST_THETA_E: f32 = 0.0;
/// step() invocation count (for measuring control-loop rate).
static mut STEP_COUNTER: u32 = 0;

/// Encoder dcounts diagnostics.
static DCOUNT_GLITCHES: core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);
static MAX_DCOUNTS:     core::sync::atomic::AtomicU32 = core::sync::atomic::AtomicU32::new(0);

/// Read internal control state by index. Used for live tuning/diagnostics
/// via the GetMotorParam protocol path. Indices are intentionally far
/// above the param-table range to avoid collision.
pub fn debug_get(index: u8) -> Option<f32> {
    let st = unsafe { core::ptr::read(addr_of!(STATE)) };
    Some(match index {
        100 => st.angle_acc,
        101 => st.vel,
        102 => st.target,
        103 => unsafe { core::ptr::read(addr_of!(LAST_VQ)) },
        104 => unsafe { core::ptr::read(addr_of!(LAST_MECH)) as f32 },
        105 => unsafe { core::ptr::read(addr_of!(LAST_THETA_E)) },
        106 => match st.mode {
            Mode::Idle => 0.0, Mode::Voltage => 1.0,
            Mode::Velocity => 2.0, Mode::Position => 3.0,
            Mode::OpenLoop => 4.0,
        },
        107 => st.theta_cmd,
        108 => unsafe { core::ptr::read(addr_of!(STEP_COUNTER)) as f32 },
        109 => st.traj_setpoint,
        110 => st.traj_v,
        111 => st.pos_i,
        // 120-124: encoder health counters (since boot, wrapping u32→f32).
        120 => encoder::health().samples as f32,
        121 => encoder::health().crc_errors as f32,
        122 => encoder::health().no_mag as f32,
        123 => encoder::health().weak_mag as f32,
        124 => encoder::health().bus_faults as f32,
        // 125 = last raw status byte (bit3 push, bit2 no-mag, bit1 weak, bit0 strong)
        125 => encoder::last_sample().map(|s| s.status as f32).unwrap_or(f32::NAN),
        // 130 = total dcount glitches rejected; 131 = max |dcounts| ever observed.
        130 => DCOUNT_GLITCHES.load(core::sync::atomic::Ordering::Relaxed) as f32,
        131 => MAX_DCOUNTS.load(core::sync::atomic::Ordering::Relaxed) as f32,
        // 132 = integer multi-turn count (mech_total, signed) cast to f32.
        //       Loses precision above 2^24 counts (~1024 revs). Use only
        //       for diagnostic visibility, not as the loop's source of truth.
        132 => st.mech_total as f32,
        // 140/141 = top/bottom 16 bits of the most recent raw 32-bit
        //          encoder SPI word. Used to inspect SSI bit alignment.
        140 => ((encoder::last_raw32() >> 16) & 0xFFFF) as f32,
        141 => (encoder::last_raw32() & 0xFFFF) as f32,
        // 200 = RCC_CSR top byte from last reset (set by main on boot).
        200 => unsafe { core::ptr::read(addr_of!(RESET_CSR)) as f32 },
        _ => return None,
    })
}

/// Last RCC_CSR (bits 24..31). Stored as u8 by main on boot.
static mut RESET_CSR: u8 = 0;

/// One-shot setter called from main during boot.
pub fn set_reset_csr(top_byte: u8) {
    unsafe { core::ptr::write(addr_of_mut!(RESET_CSR), top_byte); }
}

#[inline]
fn clamp(x: f32, lo: f32, hi: f32) -> f32 {
    if x < lo { lo } else if x > hi { hi } else { x }
}

/// Update the control mode + setpoint. Mode codes match the protocol:
///
///   0 = Idle, 1 = Voltage, 2 = Velocity, 3 = Position (raw multi-turn),
///   4 = OpenLoop, 5 = PositionAbsShortest, 6 = PositionAbsForward,
///   7 = PositionAbsBackward, 8 = PositionRelative.
///
/// Modes 5–8 dispatch into [`Mode::Position`] after resolving the
/// user-supplied `target` into an absolute multi-turn target appropriate
/// for the requested motion semantics:
///
///   * **AbsShortest** — wrap target into (−π, π], then choose the
///     resolved multi-turn target so that the move distance Δ from the
///     current accumulated angle has the smallest |Δ|.
///   * **AbsForward**  — same wrap, but Δ > 0 (rotate in the +ve
///     direction; if the wrapped target equals current position, Δ=0).
///   * **AbsBackward** — same wrap, but Δ < 0.
///   * **Relative**    — target is interpreted directly as a delta
///     from the current accumulated angle.
///
/// Modes 5–8 are normalised here into a `(Mode::Position, resolved_target)`
/// pair before falling through into the standard same-mode/cross-mode
/// state-machine logic. The control loop only ever sees `Mode::Position`.
///
/// Unknown codes route to Idle.
pub fn set_command(mode_code: u8, target: f32) {
    let prev = unsafe { core::ptr::read(addr_of!(STATE)) };

    // Resolve target-resolver modes (5–8) into Mode::Position with an
    // adjusted target so the rest of the state machine is identical to
    // raw mode-3 semantics. See doc-comment above for resolution rules.
    let (mode, target) = match mode_code {
        1 => (Mode::Voltage,  target),
        2 => (Mode::Velocity, target),
        3 => (Mode::Position, target),
        4 => (Mode::OpenLoop, target),
        5..=8 => {
            // Wrap target into (−π, π] so user input outside that range
            // still produces a well-defined absolute mech angle.
            let half = TAU * 0.5;
            let mut t = target;
            while t >  half { t -= TAU; }
            while t <= -half { t += TAU; }

            // Same wrap on the current accumulated angle to compare them
            // on the same modular footing.
            let mut cur = prev.angle_acc;
            while cur >  half { cur -= TAU; }
            while cur <= -half { cur += TAU; }

            // Smallest delta in (−π, π].
            let mut delta = t - cur;
            while delta >  half { delta -= TAU; }
            while delta <= -half { delta += TAU; }

            let resolved_delta = match mode_code {
                5 => delta,                                // AbsShortest
                6 => if delta > 0.0 { delta } else if delta == 0.0 { 0.0 } else { delta + TAU },
                7 => if delta < 0.0 { delta } else if delta == 0.0 { 0.0 } else { delta - TAU },
                8 => target,                               // Relative: raw delta
                _ => delta,
            };

            (Mode::Position, prev.angle_acc + resolved_delta)
        }
        _ => (Mode::Idle, target),
    };

    let same_mode = matches!(
        (prev.mode, mode),
        (Mode::Voltage,  Mode::Voltage)  |
        (Mode::Velocity, Mode::Velocity) |
        (Mode::Position, Mode::Position) |
        (Mode::OpenLoop, Mode::OpenLoop)
    );

    let new_state = if same_mode {
        // Only the setpoint changes; preserve integrators & accumulator.
        // For Position, also re-anchor the trajectory generator to the
        // *current* angle so a new command starts a fresh smooth move
        // from where the rotor actually is, rather than instant-stepping.
        // We also clear both `pos_i` and `vel_i` so the new move starts
        // from a clean integrator state — leftover wind-up from the
        // previous hold can otherwise drive Vq the wrong way during
        // the initial part of the new move (observed as same-mode
        // direction-asymmetric stalls).
        match mode {
            Mode::Position => State {
                target,
                traj_setpoint: prev.angle_acc,
                traj_v: 0.0,
                pos_i: 0.0,
                vel_i: 0.0,
                ..prev
            },
            _ => State { target, ..prev },
        }
    } else {
        // Mode transition: reset timing/integrators. Position mode treats
        // `target` as a delta from the moment the mode is entered, so we
        // seed `angle_acc = 0` (and continue accumulating from there).
        State {
            mode,
            target,
            last_counts: i32::MIN,
            last_cycles: cyccnt(),
            mech_total: 0,
            angle_acc: 0.0,
            vel:   0.0,
            vel_i: 0.0,
            pos_i: 0.0,
            traj_setpoint: 0.0,
            traj_v: 0.0,
            vel_acc_d: 0,
            vel_acc_dt: 0.0,
            theta_cmd: 0.0,
        }
    };

    unsafe { core::ptr::write(addr_of_mut!(STATE), new_state); }
    match mode {
        Mode::Idle => foc::shutdown(),
        _          => motor_pwm::set_enable(true),
    }
}

/// One control-loop tick. Cheap to call every main-loop iteration.
pub fn step() {
    // Step counter (diagnostic) — incremented every tick regardless of
    // mode, so the host can verify the ISR cadence even while idle.
    unsafe {
        let c = core::ptr::read(addr_of!(STEP_COUNTER));
        core::ptr::write(addr_of_mut!(STEP_COUNTER), c.wrapping_add(1));
    }

    let mut st = unsafe { core::ptr::read(addr_of!(STATE)) };

    let s = match encoder::read() {
        Some(s) => s,
        None    => return,
    };

    let p = motor::read();
    let mech_counts = s.angle as i32;
    // Always update theta_e diagnostic, even in Idle, so the host can
    // correlate encoder reading vs computed electrical angle.
    {
        let offset    = p.electrical_zero_offset as i32;
        let dir_dbg   = if p.direction >= 0.0 { 1i32 } else { -1i32 };
        // theta_e = (mech - offset) * pp * (2π/CPR) * dir.
        // pp is the motor's true pole-pair count (verified
        // empirically: GM3506 sample has pp=22 — each commanded
        // electrical revolution drags the rotor by 1/22 of a
        // mechanical revolution under Vd-aligned drag, measured at
        // ~729 counts/elec_rev = CPR/22). `dir` corrects encoder
        // mount sign.
        let pp_signed = (p.pole_pairs as i32) * dir_dbg;
        let mut e = (mech_counts - offset) % COUNTS_PER_REV;
        if e < 0 { e += COUNTS_PER_REV; }
        let mut elec = (e * pp_signed) % COUNTS_PER_REV;
        if elec < 0 { elec += COUNTS_PER_REV; }
        let theta_e_dbg = (elec as f32) * RAD_PER_COUNT;
        unsafe {
            core::ptr::write(addr_of_mut!(LAST_MECH), mech_counts);
            core::ptr::write(addr_of_mut!(LAST_THETA_E), theta_e_dbg);
        }
    }

    let mode = match st.mode {
        Mode::Idle => return,
        m => m,
    };

    if !motor_pwm::fault_ok() {
        unsafe { core::ptr::write(addr_of_mut!(STATE), State::idle()) };
        foc::shutdown();
        return;
    }

    let offset      = p.electrical_zero_offset as i32;
    let dir         = if p.direction >= 0.0 { 1i32 } else { -1i32 };
    // theta_e = (mech - offset) * pp * (2π/CPR) * dir.
    // See debug branch above for pp rationale (motor has pp=22).
    let pp_signed   = (p.pole_pairs as i32) * dir;

    // Wrap (mech - offset) into [0, COUNTS_PER_REV), then multiply by pp.
    let mut e = (mech_counts - offset) % COUNTS_PER_REV;
    if e < 0 { e += COUNTS_PER_REV; }
    let mut elec = (e * pp_signed) % COUNTS_PER_REV;
    if elec < 0 { elec += COUNTS_PER_REV; }
    let theta_e = (elec as f32) * RAD_PER_COUNT;

    // ---- timing & velocity --------------------------------------------------
    let now = cyccnt();
    let (dt, dcounts) = if st.last_counts == i32::MIN {
        (0.0_f32, 0_i32)
    } else {
        // DWT wraps every ~67 s at 64 MHz; wrapping_sub gets correct delta.
        let dcyc = now.wrapping_sub(st.last_cycles);
        let dt   = (dcyc as f32) / HCLK_HZ;
        let mut d = mech_counts - st.last_counts;
        if d >  COUNTS_PER_REV / 2 { d -= COUNTS_PER_REV; }
        if d < -COUNTS_PER_REV / 2 { d += COUNTS_PER_REV; }
        // Glitch filter: at a >1 kHz tick, any |d| > 1/4 rev (4096 counts ≈
        // ~1570 rad/s mech) is implausible and almost certainly a noisy
        // encoder sample, not real rotor motion. Reject as zero so a
        // single bad SPI read can't push angle_acc forward by ~π.
        const MAX_PLAUSIBLE_D: i32 = COUNTS_PER_REV / 4;
        if d.abs() > MAX_PLAUSIBLE_D {
            DCOUNT_GLITCHES.fetch_add(1, core::sync::atomic::Ordering::Relaxed);
            let prev_max = MAX_DCOUNTS.load(core::sync::atomic::Ordering::Relaxed);
            if d.abs() as u32 > prev_max {
                MAX_DCOUNTS.store(d.abs() as u32, core::sync::atomic::Ordering::Relaxed);
            }
            d = 0;
        } else {
            let prev_max = MAX_DCOUNTS.load(core::sync::atomic::Ordering::Relaxed);
            if d.abs() as u32 > prev_max {
                MAX_DCOUNTS.store(d.abs() as u32, core::sync::atomic::Ordering::Relaxed);
            }
        }
        // Multiply by `dir` so dcounts is positive when rotor rotates
        // in the user-positive direction (= cal Phase-2 forward step).
        // Velocity and angle_acc are in user-positive radians.
        (dt, d * dir)
    };
    st.last_counts = mech_counts;
    st.last_cycles = now;

    // Integer multi-turn accumulator (counts). Float `angle_acc` is
    // derived fresh each tick — no float drift, exact across millions
    // of revolutions (i32 overflows at ~131k revs).
    st.mech_total = st.mech_total.wrapping_add(dcounts);
    st.angle_acc  = (st.mech_total as f32) * RAD_PER_COUNT;

    // Windowed velocity: accumulate counts/dt across many ticks, only
    // recompute vel when the window is wide enough that a single-count
    // jitter doesn't dominate. With a 1 ms window and 14-bit encoder,
    // resolution is ~0.4 rad/s — well below normal closed-loop speeds.
    st.vel_acc_d  += dcounts;
    st.vel_acc_dt += dt;
    const VEL_WINDOW_S: f32 = 0.001;
    if st.vel_acc_dt >= VEL_WINDOW_S {
        let v_inst = (st.vel_acc_d as f32) * RAD_PER_COUNT / st.vel_acc_dt;
        st.vel = st.vel + 0.25 * (v_inst - st.vel);
        st.vel_acc_d  = 0;
        st.vel_acc_dt = 0.0;
    }

    let vlim    = p.voltage_limit;
    let vbus    = p.voltage_supply;
    let vel_lim = p.velocity_limit;

    let vq = match mode {
        Mode::Voltage => clamp(st.target, -vlim, vlim),

        Mode::Velocity => {
            let target_vel = clamp(st.target, -vel_lim, vel_lim);
            run_velocity_loop(&mut st, target_vel, dt, &p, vlim)
        }

        Mode::Position => {
            // Cascaded outer position / inner velocity loop. The
            // outer loop generates a velocity command from the
            // trajectory generator + position correction; the inner
            // loop (existing velocity PI) produces Vq. This rate-
            // limits the bridge naturally — the velocity loop never
            // sees a step-change demand, so it cannot pole-slip from
            // saturated Vq.
            //
            // Architecture:
            //   1. Trapezoidal trajectory generator slews `traj_setpoint`
            //      from current toward `target` at ±`traj_v_max`,
            //      ±`traj_a_max`, producing a smooth velocity profile
            //      `traj_v`.
            //   2. Outer P + bounded I on position error generates a
            //      velocity command:
            //        v_cmd = traj_v (feedforward)
            //              + pid_pos_p · pos_err
            //              + pos_i      (bounded)
            //      Position integrator removes steady-state droop
            //      under constant load. It only accumulates when the
            //      trajectory is near zero (holding or end-of-move),
            //      and is bounded to a fraction of `velocity_limit`.
            //   3. Inner velocity PI tracks `v_cmd` and outputs Vq.
            //      Vq is naturally bounded by `voltage_limit`.
            //
            // The legacy `pid_pos_d` is unused in cascade — derivative
            // damping is supplied by the inner velocity loop's
            // `pid_vel_d`. Param is retained for back-compat with
            // existing tuning UIs.

            // ---- trajectory generator ---------------------------------
            let v_max = p.traj_v_max.max(0.0);
            let a_max = p.traj_a_max.max(0.0);
            let remaining = st.target - st.traj_setpoint;

            let target_v = if a_max > 0.0 {
                // Distance needed to brake from current traj_v to 0:
                //   d_brake = v² / (2 a)   (with sign of v)
                let brake_dist = (st.traj_v * st.traj_v) / (2.0 * a_max)
                                 * if st.traj_v >= 0.0 { 1.0 } else { -1.0 };
                let same_sign = remaining * st.traj_v > 0.0
                             || st.traj_v == 0.0;
                if same_sign && remaining.abs() <= brake_dist.abs() {
                    0.0
                } else if remaining >= 0.0 {
                    v_max
                } else {
                    -v_max
                }
            } else {
                st.traj_setpoint = st.target;
                0.0
            };

            // Trajectory feed-following with hard freeze. The original
            // soft-attenuation scheme allowed the trajectory to keep
            // advancing (just at reduced speed) when the rotor lagged
            // behind. For long moves under heavy load this still
            // lets the setpoint reach the target while the rotor is
            // far behind, growing pos_err past what Kp · pos_err can
            // command without saturating Vq and pole-slipping.
            //
            // Hard-freeze instead: when |lag| exceeds `traj_follow_err`
            // we hold the setpoint and zero the trajectory velocity
            // until the rotor catches up. This bounds pos_err to
            // ~`traj_follow_err`, keeping `Kp · pos_err` well below the
            // inner-loop's available velocity budget.
            // `traj_follow_err = 0` disables freeze (legacy behaviour).
            let lag = st.traj_setpoint - st.angle_acc;
            let follow_lim = p.traj_follow_err.abs();
            let frozen = follow_lim > 0.0 && lag.abs() > follow_lim;

            if a_max > 0.0 && dt > 0.0 && !frozen {
                let dv_max = a_max * dt;
                let dv = clamp(target_v - st.traj_v, -dv_max, dv_max);
                st.traj_v += dv;
                st.traj_v = clamp(st.traj_v, -v_max, v_max);
                st.traj_setpoint += st.traj_v * dt;
            } else if frozen {
                // Bleed traj_v so when we unfreeze we don't snap forward.
                st.traj_v = 0.0;
            }

            // Snap when arrived: prevents creeping past target due
            // to float drift and gives the integrator a clean ref.
            const POS_EPS: f32 = 0.002;     // ~0.1°
            const VEL_EPS: f32 = 0.05;      // rad/s
            if (st.target - st.traj_setpoint).abs() < POS_EPS
                && st.traj_v.abs() < VEL_EPS
            {
                st.traj_setpoint = st.target;
                st.traj_v = 0.0;
            }

            // ---- outer position loop: P + bounded I on pos_err -------
            let pos_err = st.traj_setpoint - st.angle_acc;

            // Position integrator: adds a velocity bias that removes
            // steady-state droop under constant load. Active only
            // when the move has effectively settled — trajectory
            // velocity is small AND position error is small. This
            // prevents wind-up during transit and during stall
            // (where pos_err is large and growing pos_i would only
            // worsen the velocity-command saturation).
            const INT_VEL_THRESH: f32 = 0.5; // rad/s
            const INT_ERR_THRESH: f32 = 0.3; // rad
            let i_active = dt > 0.0
                && st.traj_v.abs() < INT_VEL_THRESH
                && pos_err.abs()   < INT_ERR_THRESH;
            if i_active {
                st.pos_i += p.pid_pos_i * pos_err * dt;
                // Bound to a fraction of vel_lim so the integrator
                // can't dominate the velocity command and can't push
                // the inner loop into V_q saturation.
                let i_lim = (vel_lim * 0.25).max(1.0);
                st.pos_i = clamp(st.pos_i, -i_lim, i_lim);
            } else {
                // Bleed integrator during transit / stall so any
                // leftover bias from the previous hold doesn't
                // perturb a fresh move and a stalled-out wind-up
                // doesn't hold over once the rotor catches up.
                st.pos_i *= 0.95;
            }

            // Velocity command = trajectory feedforward + P-correction
            // + bounded I. Clamp to ±traj_v_max so the inner loop
            // never sees a setpoint above the user-configured cruise
            // speed — vel_lim is an absolute hardware ceiling, not a
            // tracking goal, and demanding it during a transient is
            // what previously caused pole-slip on long moves.
            let v_cmd_lim = if v_max > 0.0 { v_max } else { vel_lim };
            let v_cmd = clamp(
                st.traj_v + p.pid_pos_p * pos_err + st.pos_i,
                -v_cmd_lim, v_cmd_lim,
            );

            // ---- inner velocity loop produces Vq ---------------------
            run_velocity_loop(&mut st, v_cmd, dt, &p, vlim)
        }

        Mode::OpenLoop => {
            // Diagnostic: integrate theta_cmd at the requested electrical
            // rad/s and apply (Vd=vlim, Vq=0). Returns 0 vq; we override
            // the apply_voltage call below.
            st.theta_cmd += st.target * dt;
            // Wrap to [0, 2π) to keep the float well-conditioned.
            while st.theta_cmd >= TAU { st.theta_cmd -= TAU; }
            while st.theta_cmd <  0.0 { st.theta_cmd += TAU; }
            let _ = vel_lim;
            0.0
        }

        Mode::Idle => 0.0,
    };

    unsafe { core::ptr::write(addr_of_mut!(STATE), st); }
    unsafe {
        core::ptr::write(addr_of_mut!(LAST_VQ), vq);
        core::ptr::write(addr_of_mut!(LAST_MECH), mech_counts);
        core::ptr::write(addr_of_mut!(LAST_THETA_E), theta_e);
    }

    match mode {
        Mode::OpenLoop => foc::apply_voltage(st.theta_cmd, 0.0, vlim, vbus),
        _              => foc::apply_voltage(theta_e, 0.0, vq, vbus),
    }
}

/// Velocity PI with I-term anti-windup. Returns Vq in volts (clamped).
fn run_velocity_loop(
    st:         &mut State,
    target_vel: f32,
    dt:         f32,
    p:          &motor::MotorParams,
    vlim:       f32,
) -> f32 {
    let err = target_vel - st.vel;

    if dt > 0.0 {
        st.vel_i += p.pid_vel_i * err * dt;
        // Standard clamp-the-state anti-windup: integrator can't push
        // past what the bridge can deliver.
        st.vel_i = clamp(st.vel_i, -vlim, vlim);
    }

    let v = p.pid_vel_p * err + st.vel_i - p.pid_vel_d * st.vel;
    clamp(v, -vlim, vlim)
}
