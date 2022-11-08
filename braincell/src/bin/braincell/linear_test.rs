use crate::app::linear_test_task;
use crate::app::monotonics;
use cortex_m::asm;
use libm::fabsf;
use rtic::Mutex;
use crate::config::sys_config;
use systick_monotonic::fugit::Duration;

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

pub fn linear_test_task(mut cx: linear_test_task::Context) {
    let mut yaw: f32 = 0.0;
    cx.shared
        .imu_filter
        .lock(|imu_filter| (_, _, yaw) = imu_filter.filtered().unwrap_or((0.0, 0.0, 0.0))); // IMU is mounted sideways -> swap pitch and roll

    if *cx.local.currently_running {
        // check for button press
        if cx.local.button.is_low() {
            while cx.local.button.is_low() {
                asm::nop();
            }
            *cx.local.currently_running = false;
        } else if (monotonics::now().ticks() - *cx.local.start_ticks) as f32 * sys_config::SECONDS_PER_TICK > 3.0 {
            // set motor set points
            cx.shared.motor_setpoints.lock(|motor_setpoints| {
                motor_setpoints.f_right = 0.0;
                motor_setpoints.r_right = 0.0;
                motor_setpoints.f_left = 0.0;
                motor_setpoints.r_left = 0.0;
            });
            *cx.local.currently_running = false;
        } else {
            // compute right and left side speed set points
            let yaw_error: f32 = compute_yaw_error(yaw, *cx.local.desired_yaw);
            let base_speed: f32 = 2000.0;
            let yaw_compensated_speed_offset: f32 = cx
                .local
                .yaw_compensation_pid
                .next_control_output(fabsf(yaw_error))
                .output;
            let (right_side_speed, left_side_speed): (f32, f32) = {
                if yaw_error > 0.0 {
                    // current heading to the right of center
                    (base_speed, base_speed - yaw_compensated_speed_offset)
                } else {
                    // current heading to the left of center
                    (base_speed - yaw_compensated_speed_offset, base_speed)
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
    } else if cx.local.button.is_low() {
        while cx.local.button.is_low() {
            asm::nop();
        }
        *cx.local.currently_running = true;
        *cx.local.start_ticks = monotonics::now().ticks();
        *cx.local.desired_yaw = yaw;
    }
    // run at 100 Hz
    linear_test_task::spawn_after(Duration::<u64, 1, 1000>::millis(10)).unwrap();
}