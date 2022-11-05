use crate::config::sys_config;
use core::fmt::Write;
use libm::{absf, expf};
use pid::Pid;
use rtic::Mutex;
use systick_monotonic::fugit::Duration;

pub enum State {
    Idle,
    Linear,
    Turning,
}

fn within_range<T>(val: T, lower_bound: T, upper_bound: T) -> T {
    val > lower_bound && val < upper_bound
}

// takes distance in mm
// returns speed in deg/s
fn linear_speed_profile(distance: f32) -> f32 {
    sys_config::MAX_MOTOR_SPEED_DPS
        * (1 - expf(-distance / sys_config::LINEAR_SPEED_PROFILE_TAU_MM))
}

// assume IMU is mounted upside down
// yaw values range from -180.0 to +180.0
// return value sign:
// - error means measured_yaw is to the left of desired_yaw
// + error means measured_yaw is to the right of desired_yaw
fn compute_yaw_error(measured_yaw: f32, desired_yaw: f32) -> f32 {
    let error: f32 = measured_yaw - desired_yaw;
    if absf(error) < 360.0 - absf(error) {
        error
    } else if error > 0 {
        error - 360.0
    } else {
        error + 360.0
    }
}

pub fn supervisor(mut cx: supervisor::Context) {
    // get TOF and IMU readings
    let distance: f32 = 0.0;
    cx.shared
        .tof_front_filter
        .lock(|tof_front_filter| distance = tof_front_filter.filtered().unwrap_or(0.0));
    let (roll, pitch, yaw): (f32, f32, f32) = (0.0, 0.0, 0.0);
    cx.shared
        .imu_filter
        .lock(|imu_filter| (pitch, roll, yaw) = imu_filter.filtered().unwrap_or((0.0, 0.0, 0.0))); // IMU is mounted sideways -> swap pitch and roll

    match cx.local.state {
        State::Idle => {
            // set motor set points to 0
            cx.shared.motor_setpoints.lock(|motor_setpoints| {
                motor_setpoints
                    .into_inter()
                    .for_each(|setpoint| setpoint = 0.0)
            });
            // check for button press and release
            if cx.local.button.is_low() {
                while (cx.local.button.is_low()) {
                    asm::nop();
                }
                cx.local.curr_leg = 1;
                cx.local.state = State::Linear;
            }
        }
        State::Linear => {
            // check for button press and release
            if cx.local.button.is_low() {
                while (cx.local.button.is_low()) {
                    asm::nop();
                }
                cx.local.curr_leg = 0;
                cx.local.state = State::Idle;
            }
            // check if linear leg is complete
            else if distance
                < sys_config::DISTANCE_TO_WALL_THRESHOLDS_MM[curr_leg]
                    - sys_config::ROBOT_CENTER_TO_TOF_MM
                && within_range(
                    pitch,
                    sys_config::PITCH_LOWER_BOUND_DEG,
                    sys_config::PITCH_HIGHER_BOUND_DEG,
                )
            {
                if cx.local.curr_leg == sys_config::NUM_LEGS_IN_RACE {
                    // race is complete
                    cx.local.curr_leg = 0;
                    cx.local.state = State::Idle;
                } else {
                    cx.local.curr_leg += 1;
                    cx.local.state = State::Turning;
                    cx.local.turning_pid = Pid::new(); // TODO: need to reset the turning pid
                    cx.local.turning_pid.setpoint = 0.0; // set point for yaw error
                    cx.local.num_samples_within_yaw_tolerance = 0;
                }
            } else {
                // compute right and left side speed set points
                let yaw_error: f32 =
                    compute_yaw_error(yaw, sys_config::YAW_SET_POINTS_DEG[cx.local.curr_leg]);
                let base_speed: f32 = linear_speed_profile(distance);
                let (right_side_speed, left_side_speed): (f32, f32) = {
                    if yaw_error > 0.0 {
                        // current heading to the right of center
                        (
                            base_speed,
                            base_speed - sys_config::YAW_CORRECTION_GAIN * absf(yaw_error),
                        )
                    } else {
                        // current heading to the left of center
                        (
                            base_speed - sys_config::YAW_CORRECTION_GAIN * absf(yaw_error),
                            base_speed,
                        )
                    }
                };
                // set motor set points
                motor_setpoints.f_right = right_side_speed;
                motor_setpoints.r_right = right_side_speed;
                motor_setpoints.f_left = left_side_speed;
                motor_setpoints.r_left = left_side_speed;
            }
        }
        State::Turning => {
            // check for button press and release
            if cx.local.button.is_low() {
                while (cx.local.button.is_low()) {
                    asm::nop();
                }
                cx.local.curr_leg = 0;
                cx.local.state = State::Idle;
            } else {
                // check if yaw set point is reached
                let yaw_error: f32 =
                    compute_yaw_error(yaw, sys_config::YAW_SET_POINTS_DEG[cx.local.curr_leg]);
                if absf(yaw_error) < sys_config::YAW_TOLERANCE_DEG {
                    // check if steady state is reached
                    cx.local.num_samples_within_yaw_tolerance += 1;
                    if cx.local.num_samples_within_yaw_tolerance
                        == sys_config::STEADY_STATE_NUM_SAMPLES
                    {
                        cx.local.state = State::Linear;
                    }
                } else {
                    cx.local.num_samples_within_yaw_tolerance = 0;
                    // compute right and left side speed set points
                    let base_speed: f32 =
                        cx.local.turning_pid.next_control_output(yaw_error).output;
                    let (right_side_speed, left_side_speed): (f32, f32) = (-base_speed, base_speed);
                    // set motor set points
                    motor_setpoints.f_right = right_side_speed;
                    motor_setpoints.r_right = right_side_speed;
                    motor_setpoints.f_left = left_side_speed;
                    motor_setpoints.r_left = left_side_speed;
                }
            }
        }
    }
    // Run at 100Hz
    supervisor::spawn_after(Duration::<u64, 1, 1000>::millis(10)).unwrap();
}