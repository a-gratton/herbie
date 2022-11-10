use crate::app::turning_test_task;
use core::fmt::Write;
use cortex_m::asm;
use libm::{fabsf, fminf};
use pid;
use rtic::Mutex;
use systick_monotonic::fugit::Duration;

const YAW_TOLERANCE_DEG: f32 = 2.0;
const MAX_SPEED_DEG_PER_S: f32 = 2640.0;
const TURNING_SPEED_SLOPE: f32 = 5.2; //could go a little less


// assume IMU is mounted upside down
// yaw values range from -180.0 to +180.0
// return value sign:
// - error means measured_yaw is to the left of desired_yaw
// + error means measured_yaw is to the right of desired_yaw
fn compute_yaw_error(measured_yaw: f32, desired_yaw: f32) -> f32 {
    let error: f32 = measured_yaw - desired_yaw;
    if fabsf(error) < 360.0 - fabsf(error) {
        error
    } else if error > 0.0 {
        error - 360.0
    } else {
        error + 360.0
    }
}

// trapezoid speed profile
fn turning_speed_profile(yaw_error: f32) -> f32 {
    let mut speed: f32 = fminf(MAX_SPEED_DEG_PER_S, TURNING_SPEED_SLOPE * fabsf(yaw_error));
    if yaw_error < 0.0 {
        speed *= -1.0;
    }
    return speed;
}

pub fn turning_test_task(mut cx: turning_test_task::Context) {
    let mut yaw: f32 = 0.0;
    cx.shared
        .imu_filter
        .lock(|imu_filter| (_, _, yaw) = imu_filter.filtered().unwrap_or((0.0, 0.0, 0.0))); // IMU is mounted sideways -> swap pitch and roll

    let mut base_speed: f32 = 0.0;
    let mut yaw_error: f32 = 0.0;

    if *cx.local.currently_turning {
        // check for button press
        if cx.local.button.is_low() {
            while cx.local.button.is_low() {
                asm::nop();
            }
            *cx.local.currently_turning = false;
            cx.shared.motor_setpoints.lock(|motor_setpoints| {
                motor_setpoints.f_right = 0.0;
                motor_setpoints.r_right = 0.0;
                motor_setpoints.f_left = 0.0;
                motor_setpoints.r_left = 0.0;
            });
        } else {
            // check if yaw set point is reached
            yaw_error = compute_yaw_error(yaw, *cx.local.desired_yaw);
            if fabsf(yaw_error) < YAW_TOLERANCE_DEG {
                *cx.local.currently_turning = false;
                cx.shared.motor_setpoints.lock(|motor_setpoints| {
                    motor_setpoints.f_right = 0.0;
                    motor_setpoints.r_right = 0.0;
                    motor_setpoints.f_left = 0.0;
                    motor_setpoints.r_left = 0.0;
                });
            } else {
                *cx.local.num_samples_within_yaw_tolerance = 0;
                // compute right and left side speed set points
                // base_speed = cx.local.turning_pid.next_control_output(yaw_error).output;
                base_speed = turning_speed_profile(yaw_error);
                let (right_side_speed, left_side_speed): (f32, f32) = (base_speed, -base_speed);
                // set motor set points
                cx.shared.motor_setpoints.lock(|motor_setpoints| {
                    motor_setpoints.f_right = right_side_speed;
                    motor_setpoints.r_right = right_side_speed;
                    motor_setpoints.f_left = left_side_speed;
                    motor_setpoints.r_left = left_side_speed;
                });
            }
        }
    } else if cx.local.button.is_low() {
        while cx.local.button.is_low() {
            asm::nop();
        }
        *cx.local.currently_turning = true;
        cx.shared.imu_filter.lock(|imu_filter| imu_filter.reset());
        *cx.local.desired_yaw = 90.0;
    }
    cx.shared.tx.lock(|tx| {
        writeln!(
            tx,
            "measured_yaw {yaw} desired_yaw {} base_speed {} yaw_error {yaw_error}",
            *cx.local.desired_yaw,
            base_speed / 10.0
        )
        .unwrap()
    });
    // run at 100 Hz
    turning_test_task::spawn_after(Duration::<u64, 1, 1000>::millis(10)).unwrap();
}
