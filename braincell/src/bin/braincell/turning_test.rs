use crate::app::turning_test_task;
use cortex_m::asm;
use libm::fabsf;
use rtic::Mutex;
use systick_monotonic::fugit::Duration;

const YAW_TOLERANCE_DEG: f32 = 3.0;
const STEADY_STATE_NUM_SAMPLES: u32 = 100;

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

pub fn turning_test_task(mut cx: turning_test_task::Context) {
    let mut yaw: f32 = 0.0;
    cx.shared
        .imu_filter
        .lock(|imu_filter| (_, _, yaw) = imu_filter.filtered().unwrap_or((0.0, 0.0, 0.0))); // IMU is mounted sideways -> swap pitch and roll

    if *cx.local.currently_turning {
        // check for button press
        if cx.local.button.is_low() {
            while cx.local.button.is_low() {
                asm::nop();
            }
            *cx.local.currently_turning = false;
        } else {
            // check if yaw set point is reached
            let yaw_error: f32 = compute_yaw_error(yaw, *cx.local.desired_yaw);
            if fabsf(yaw_error) < YAW_TOLERANCE_DEG {
                // check if steady state is reached
                *cx.local.num_samples_within_yaw_tolerance += 1;
                if *cx.local.num_samples_within_yaw_tolerance == STEADY_STATE_NUM_SAMPLES {
                    *cx.local.currently_turning = false;
                    cx.shared.motor_setpoints.lock(|motor_setpoints| {
                        motor_setpoints.f_right = 0.0;
                        motor_setpoints.r_right = 0.0;
                        motor_setpoints.f_left = 0.0;
                        motor_setpoints.r_left = 0.0;
                    });
                }
            } else {
                *cx.local.num_samples_within_yaw_tolerance = 0;
                // compute right and left side speed set points
                let base_speed: f32 = cx.local.turning_pid.next_control_output(yaw_error).output;
                let (right_side_speed, left_side_speed): (f32, f32) = (-base_speed, base_speed);
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
        *cx.local.desired_yaw = yaw + 90.0;
        if *cx.local.desired_yaw > 180.0 {
            *cx.local.desired_yaw -= 360.0;
        }
    }
    // run at 100 Hz
    turning_test_task::spawn_after(Duration::<u64, 1, 1000>::millis(10)).unwrap();
}
