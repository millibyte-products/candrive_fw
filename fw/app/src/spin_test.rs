//! One-shot open-loop spin test.
//!
//! Applies a rotating sinusoidal voltage vector to the 3-phase bridge
//! at low modulation depth for 2 seconds, then disables the gate driver
//! and parks duties at center. This is bring-up only — it lets us
//! confirm wiring and rotation direction before enabling closed-loop FOC.
//!
//! Safety
//! ------
//! * Modulation amplitude capped at 12% of bus voltage (≈1.4 V on a 12 V
//!   supply). At MOTOR_RESISTANCE ~5 Ω this draws ~0.3 A — well under
//!   any reasonable winding limit.
//! * If NFAULT asserts at any point, immediately disable enables and
//!   abort.
//! * Electrical frequency 2 Hz so a tester can visually confirm rotation.
//! * Hard-wired total duration of 2 s, then guaranteed disable.

use crate::motor_pwm;
use libm::cosf;

const TWO_PI: f32 = 6.283_185_3;
/// Electrical frequency, Hz. Two full cycles per second of the voltage
/// vector.
const F_ELEC_HZ: f32 = 2.0;
/// Modulation amplitude as a fraction of (ARR/2). 0.12 = 12% of bus.
const AMP_FRAC: f32 = 0.12;
/// Test duration in microseconds (hard cap).
const DURATION_US: u32 = 2_000_000;
/// Update period in microseconds. 5 kHz update rate is plenty.
const STEP_US: u32 = 200;

/// Run the open-loop spin test. Blocks for ~2 seconds, then returns.
///
/// `delay_us` is taken from `CommonApi.delay_us`.
/// Returns `true` on completion, `false` if NFAULT tripped (test aborted).
pub fn run(delay_us: extern "C" fn(u32)) -> bool {
    let arr = motor_pwm::PWM_ARR as f32;
    let half = arr * 0.5;
    let amp = half * AMP_FRAC;

    // Park at center duty before enabling, so the bridge sees zero
    // differential voltage at the instant the gates come on.
    motor_pwm::set_duty(motor_pwm::PWM_ARR / 2, motor_pwm::PWM_ARR / 2, motor_pwm::PWM_ARR / 2);
    motor_pwm::set_enable(true);

    let total_steps = DURATION_US / STEP_US;
    // dtheta per step in radians.
    let dtheta = TWO_PI * F_ELEC_HZ * (STEP_US as f32) * 1e-6;
    let mut theta: f32 = 0.0;

    for _ in 0..total_steps {
        if !motor_pwm::fault_ok() {
            motor_pwm::set_enable(false);
            motor_pwm::set_duty(motor_pwm::PWM_ARR / 2,
                                motor_pwm::PWM_ARR / 2,
                                motor_pwm::PWM_ARR / 2);
            return false;
        }

        // Three cosines 120° apart, scaled to ±amp around half-ARR.
        let a = half + amp * cosf(theta);
        let b = half + amp * cosf(theta - TWO_PI / 3.0);
        let c = half + amp * cosf(theta + TWO_PI / 3.0);

        motor_pwm::set_duty(a as u16, b as u16, c as u16);

        theta += dtheta;
        if theta >= TWO_PI { theta -= TWO_PI; }

        delay_us(STEP_US);
    }

    // Park: zero modulation, then disable the gates.
    motor_pwm::set_duty(motor_pwm::PWM_ARR / 2,
                        motor_pwm::PWM_ARR / 2,
                        motor_pwm::PWM_ARR / 2);
    motor_pwm::set_enable(false);
    true
}
