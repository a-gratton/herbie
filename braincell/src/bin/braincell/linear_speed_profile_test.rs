use crate::app::linear_speed_profile_test_task;
use core::fmt::Write;
use cortex_m::asm;
use libm::{fabsf, fminf};
use rtic::Mutex;
use systick_monotonic::fugit::Duration;

const DISTANCE_TARGET_MM: i32 = 150;
const DISTANCE_TOLERANCE_MM: i32 = 20;
const MAX_TOF_READING_MM: i32 = 5000;
const STEADY_STATE_NUM_SAMPLES: usize = 10;

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

pub fn linear_speed_profile_test_task(mut cx: linear_speed_profile_test_task::Context) {
    let mut yaw: f32 = 0.0;
    cx.shared
        .imu_filter
        .lock(|imu_filter| (_, _, yaw) = imu_filter.filtered().unwrap_or((0.0, 0.0, 0.0))); // IMU is mounted sideways -> swap pitch and roll
    let mut distance: i32 = MAX_TOF_READING_MM;
    cx.shared.tof_front_filter.lock(|tof_front_filter| {
        distance = tof_front_filter.filtered().unwrap_or(MAX_TOF_READING_MM)
    });

    if *cx.local.currently_running {
        // check for button press
        if cx.local.button.is_low() {
            while cx.local.button.is_low() {
                asm::nop();
            }
            *cx.local.currently_running = false;
        } else if fabsf((distance - DISTANCE_TARGET_MM) as f32) < DISTANCE_TOLERANCE_MM as f32 {
            *cx.local.num_samples_within_distance_tolerance += 1;
            if *cx.local.num_samples_within_distance_tolerance == STEADY_STATE_NUM_SAMPLES {
                // set motor set points
                cx.shared.motor_setpoints.lock(|motor_setpoints| {
                    motor_setpoints.f_right = 0.0;
                    motor_setpoints.r_right = 0.0;
                    motor_setpoints.f_left = 0.0;
                    motor_setpoints.r_left = 0.0;
                });
                *cx.local.currently_running = false;
                *cx.local.currently_turning = true;
                *cx.local.desired_yaw = 90.0;
            }
        } else {
            *cx.local.num_samples_within_distance_tolerance = 0;
            // compute right and left side speed set points
            let yaw_error: f32 = compute_yaw_error(yaw, *cx.local.desired_yaw);
            *cx.local.base_speed = -cx
                .local
                .distance_pid
                .next_control_output(distance as f32)
                .output;
            let yaw_alpha: f32 = cx
                .local
                .yaw_compensation_pid
                .next_control_output(fabsf(yaw_error))
                .output;
            let (right_side_speed, left_side_speed): (f32, f32) = {
                if yaw_error < 0.0 {
                    // current heading to the right of center
                    (
                        *cx.local.base_speed,
                        *cx.local.base_speed * (1.0 - yaw_alpha),
                    )
                } else {
                    // current heading to the left of center
                    (
                        *cx.local.base_speed * (1.0 - yaw_alpha),
                        *cx.local.base_speed,
                    )
                }
            };
            // set motor set points
            cx.shared.motor_setpoints.lock(|motor_setpoints| {
                motor_setpoints.f_right = right_side_speed;
                motor_setpoints.r_right = right_side_speed;
                motor_setpoints.f_left = left_side_speed;
                motor_setpoints.r_left = left_side_speed;
            });
        }
    } else if *cx.local.currently_turning {
        // check if yaw set point is reached
        let yaw_error = compute_yaw_error(yaw, *cx.local.desired_yaw);
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
            let base_speed = turning_speed_profile(yaw_error);
            let (right_side_speed, left_side_speed): (f32, f32) = (base_speed, -base_speed);
            // set motor set points
            cx.shared.motor_setpoints.lock(|motor_setpoints| {
                motor_setpoints.f_right = right_side_speed;
                motor_setpoints.r_right = right_side_speed;
                motor_setpoints.f_left = left_side_speed;
                motor_setpoints.r_left = left_side_speed;
            });
        }
    } else if cx.local.button.is_low() {
        while cx.local.button.is_low() {
            asm::nop();
        }
        cx.shared.imu_filter.lock(|imu_filter| imu_filter.reset());
        *cx.local.currently_running = true;
        *cx.local.desired_yaw = 0.0;
        *cx.local.num_samples_within_distance_tolerance = 0;
        cx.local.distance_pid.setpoint = DISTANCE_TARGET_MM as f32;
    }
    cx.shared
        .tx
        .lock(|tx| writeln!(tx, "yaw {yaw} distance {distance}").unwrap());
    // run at 100 Hz
    linear_speed_profile_test_task::spawn_after(Duration::<u64, 1, 1000>::millis(10)).unwrap();
}
