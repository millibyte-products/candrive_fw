//! FOC math and pole-pair / electrical-zero calibration.
//!
//! Pipeline (closed-loop, eventually):
//!
//!   theta_e = mech_angle * pole_pairs - zero_offset
//!   (Vd, Vq)              ─┐
//!     │  inverse Park       │
//!     ▼                     │
//!   (Valpha, Vbeta)         │
//!     │  inverse Clarke     │
//!     ▼                     │
//!   (Va, Vb, Vc)            │
//!     │  SVPWM common-mode  │
//!     ▼                     │
//!   set_duty(da, db, dc) ───┘
//!
//! For phase 5 first-light we only need open-loop forced commutation
//! (Vd > 0, Vq = 0, theta_e = command) so the rotor follows the
//! electrical angle. That's enough to:
//!
//!   * verify pole-pair count by comparing electrical-rev count to
//!     mechanical-angle delta (encoder counts).
//!   * record the encoder reading at theta_e=0 (electrical zero offset).
//!
//! Both feed into closed-loop FOC later.

use libm::{cosf, sinf};

use crate::encoder;
use crate::iwdg;
use crate::motor;
use crate::motor_pwm::{self, PWM_ARR};

const SQRT3_2: f32 = 0.8660254;

#[inline]
fn clamp01(x: f32) -> f32 {
    if x < 0.0 { 0.0 } else if x > 1.0 { 1.0 } else { x }
}

/// Apply a voltage vector `(vd, vq)` at electrical angle `theta_e` (rad).
/// `vbus` is the supply rail; outputs are normalised to it.
///
/// Uses inverse Park → inverse Clarke → SVPWM (min-max common-mode
/// injection). Duties saturate at 0..PWM_ARR.
pub fn apply_voltage(theta_e: f32, vd: f32, vq: f32, vbus: f32) {
    let c = cosf(theta_e);
    let s = sinf(theta_e);

    // Inverse Park: (Vd, Vq) → (Valpha, Vbeta)
    let v_alpha = vd * c - vq * s;
    let v_beta  = vd * s + vq * c;

    // Inverse Clarke (amplitude-invariant): (Valpha, Vbeta) → (Va, Vb, Vc)
    let mut va = v_alpha;
    let mut vb = -0.5 * v_alpha + SQRT3_2 * v_beta;
    let mut vc = -0.5 * v_alpha - SQRT3_2 * v_beta;

    // SVPWM via common-mode injection (subtract midpoint of max/min).
    let vmax = if va > vb { va } else { vb };
    let vmax = if vmax > vc { vmax } else { vc };
    let vmin = if va < vb { va } else { vb };
    let vmin = if vmin < vc { vmin } else { vc };
    let mid = 0.5 * (vmax + vmin);
    va -= mid;
    vb -= mid;
    vc -= mid;

    // Convert to duty centred on 0.5: d = 0.5 + V/Vbus.
    let inv_vbus = if vbus > 0.001 { 1.0 / vbus } else { 0.0 };
    let da = clamp01(0.5 + va * inv_vbus);
    let db = clamp01(0.5 + vb * inv_vbus);
    let dc = clamp01(0.5 + vc * inv_vbus);
    let arr = PWM_ARR as f32;
    motor_pwm::set_duty(
        (da * arr) as u16,
        (db * arr) as u16,
        (dc * arr) as u16,
    );
}

/// Disable the bridge and command zero on all three duty registers.
pub fn shutdown() {
    motor_pwm::set_duty(0, 0, 0);
    motor_pwm::set_enable(false);
}

/// Result of an alignment / electrical-zero calibration.
#[derive(Copy, Clone, Debug)]
pub struct CalResult {
    /// Encoder reading (raw 14-bit counts) at theta_e=0 after the rotor
    /// has settled into electrical alignment.
    pub zero_offset_counts: u16,
    /// Sign of (encoder counts) vs (commanded electrical-angle advance).
    /// `+1` if mech counts increase when theta_e is stepped forward;
    /// `-1` otherwise. Used to make closed-loop torque stabilizing.
    pub direction: i8,
    /// True if NFAULT was asserted at any point during the run.
    pub fault: bool,
}

/// Electrical-zero alignment + direction probe via voltage hold.
///
/// On a cogging-prone gimbal motor we cannot trust a slow sweep
/// regression: the rotor captures cogging detents and the regression
/// intercept is biased by tens of mech counts (== tens of degrees
/// electrical). Instead we apply a strong stator field at theta_e=0
/// and let the rotor PARK there.
///
/// Sequence:
///   1. Pre-kick at theta_e=+pi/4 to break any detent at theta_e=0.
///   2. Direction probe: lock at theta_e=0 (settle), then theta_e=+pi/2
///      (probe), measure mech delta, lock back at theta_e=0 (return).
///      The probe does not span a full electrical cycle, so the rotor
///      cannot capture the next detent — it always returns to the
///      same rest position.
///   3. Final park at theta_e=0 with average mech sample.
///
/// `delay_us` is the common-API microsecond delay (1 µs ticks).
pub fn calibrate(
    delay_us: extern "C" fn(u32),
    valign_volts: f32,
    align_ms: u32,
) -> CalResult {
    use core::f32::consts::FRAC_PI_2;

    let p = motor::read();
    let vbus = p.voltage_supply;

    motor_pwm::set_enable(true);

    let mut fault = false;

    // Helper: apply theta_e for `dur_ms` ms; returns false on fault.
    let hold = |theta: f32, dur_ms: u32, fault: &mut bool| -> bool {
        for _ in 0..dur_ms {
            apply_voltage(theta, valign_volts, 0.0, vbus);
            (delay_us)(1000);
            iwdg::pet();
            if !motor_pwm::fault_ok() { *fault = true; return false; }
        }
        true
    };

    // ---- Phase 1: bidirectional sweep to break any prior detent ----------
    // A single static kick can fail when the rotor is sitting in a deep
    // detent that happens to be near our kick angle. Sweeping the field
    // ±π electrical from theta_e=0 drags the rotor across any nearby
    // detent in BOTH directions, guaranteeing it ends up driven by the
    // field rather than captured by cogging.
    let kick_ms = 200u32.max(align_ms / 8);
    if !fault {
        for i in 0..kick_ms {
            let frac = (i as f32) / (kick_ms as f32);
            // Triangle: 0 → +π → -π → 0 over kick_ms.
            let t = frac * 4.0; // 0..4
            let theta = if t < 1.0 {
                t * core::f32::consts::PI
            } else if t < 3.0 {
                (2.0 - t) * core::f32::consts::PI
            } else {
                (t - 4.0) * core::f32::consts::PI
            };
            apply_voltage(theta, valign_volts, 0.0, vbus);
            (delay_us)(1000);
            iwdg::pet();
            if !motor_pwm::fault_ok() { fault = true; break; }
        }
    }

    // ---- Phase 2: settle at theta_e=0 -------------------------------------
    let settle_ms = align_ms.max(800);
    hold(0.0, settle_ms, &mut fault);

    // ---- Phase 3: small forward probe for direction sign ------------------
    // Use pi/2 — large enough to overcome static friction reliably,
    // small enough that the rotor cannot slip into the next detent.
    let probe_ms: u32 = 200;
    let mut probe_prev: i32 =
        encoder::read().map(|s| s.angle as i32).unwrap_or(0);
    let mut probe_un: f32 = 0.0;
    if !fault {
        // Step the field to +pi/2 and let rotor settle.
        for i in 0..probe_ms {
            let frac = (i as f32) / (probe_ms as f32);
            apply_voltage(frac * FRAC_PI_2, valign_volts, 0.0, vbus);
            (delay_us)(1000);
            iwdg::pet();
            if !motor_pwm::fault_ok() { fault = true; break; }
            let m_raw = encoder::read().map(|s| s.angle as i32).unwrap_or(probe_prev);
            let mut d = m_raw - probe_prev;
            if d >  8192 { d -= 16384; }
            if d < -8192 { d += 16384; }
            probe_un += d as f32;
            probe_prev = m_raw;
        }
    }
    let direction: i8 = if probe_un >= 0.0 { 1 } else { -1 };

    // ---- Phase 4: return to theta_e=0 -------------------------------------
    // Same length as probe so the rotor reverses cleanly back to its
    // original rest position. After this phase the rotor sits exactly
    // where it sat before the probe — no per-cal drift.
    if !fault {
        for i in 0..probe_ms {
            let frac = (i as f32) / (probe_ms as f32);
            apply_voltage((1.0 - frac) * FRAC_PI_2, valign_volts, 0.0, vbus);
            (delay_us)(1000);
            iwdg::pet();
            if !motor_pwm::fault_ok() { fault = true; break; }
        }
    }

    // ---- Phase 5: final settle + sample mech ------------------------------
    // Hold at theta_e=0 with a longer settle, then average a window of
    // encoder reads to smooth sub-count jitter.
    let final_settle_ms = (align_ms / 2).max(400);
    hold(0.0, final_settle_ms, &mut fault);

    let sample_ms: u32 = 80;
    let mut prev_raw: i32 =
        encoder::read().map(|s| s.angle as i32).unwrap_or(0);
    let mut mech_un: f32 = prev_raw as f32;
    let mut sum: f32 = 0.0;
    let mut n: u32 = 0;
    for _ in 0..sample_ms {
        apply_voltage(0.0, valign_volts, 0.0, vbus);
        (delay_us)(1000);
        iwdg::pet();
        if !motor_pwm::fault_ok() { fault = true; break; }
        let m_raw = encoder::read().map(|s| s.angle as i32).unwrap_or(prev_raw);
        let mut d = m_raw - prev_raw;
        if d >  8192 { d -= 16384; }
        if d < -8192 { d += 16384; }
        mech_un += d as f32;
        prev_raw = m_raw;
        sum += mech_un;
        n += 1;
    }
    let off_avg = if n > 0 { sum / (n as f32) } else { mech_un };

    let cpr_f = 16384.0f32;
    let mut off_f = off_avg;
    while off_f < 0.0    { off_f += cpr_f; }
    while off_f >= cpr_f { off_f -= cpr_f; }
    let zero_offset_counts = off_f as u16;

    apply_voltage(0.0, 0.0, 0.0, vbus);
    shutdown();

    CalResult { zero_offset_counts, direction, fault }
}
