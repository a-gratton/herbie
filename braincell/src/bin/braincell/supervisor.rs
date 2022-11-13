use crate::app::supervisor_task;
use crate::config::tuning::{PITCH_LOWER_BOUND_DEG, PITCH_UPPER_BOUND_DEG};
use crate::config::{sys_config, tuning};
use cortex_m::asm;
use libm::{fabsf, fminf};
use rtic::Mutex;
use systick_monotonic::fugit::Duration;

pub enum State {
    Idle,
    Linear,
    Turning,
}

fn within_range<T: core::cmp::PartialOrd>(val: T, lower_bound: T, upper_bound: T) -> bool {
    val > lower_bound && val < upper_bound
}

fn turning_speed_profile(yaw_error: f32) -> f32 {
    let mut speed: f32 = fminf(
        tuning::MAX_TURNING_SPEED_DPS,
        tuning::TURNING_SPEED_SLOPE * fabsf(yaw_error) + 50.0,
    );
    if yaw_error < 0.0 {
        speed *= -1.0;
    }
    return speed;
}

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

fn compute_dist_error(measured_dist: i32, desired_dist: i32) -> i32 {
    measured_dist - desired_dist
}

// pause the system if button is pressed
fn block_if_button_pressed<P>(button: &P)
where
    P: embedded_hal::digital::v2::InputPin,
{
    if button.is_low().unwrap_or(false) {
        while button.is_low().unwrap_or(false) {
            asm::nop();
        }
    }
}

pub fn supervisor_task(mut cx: supervisor_task::Context) {
    // get TOF and IMU readings
    let mut front_distance: i32 = *cx.local.prev_front_distance;
    let mut left_distance: i32 = sys_config::MIN_TOF_DISTANCE_MM;
    cx.shared.tof_front_filter.lock(|tof_front_filter| {
        front_distance = tof_front_filter
            .filtered()
            .unwrap_or(*cx.local.prev_front_distance)
    });
    cx.shared.tof_left_filter.lock(|tof_left_filter| {
        left_distance = tof_left_filter
            .filtered()
            .unwrap_or(sys_config::MIN_TOF_DISTANCE_MM)
    });
    let (mut roll, mut pitch, mut yaw): (f32, f32, f32) = (0.0, 0.0, 0.0);
    cx.shared
        .imu_filter
        .lock(|imu_filter| (pitch, roll, yaw) = imu_filter.filtered().unwrap_or((0.0, 0.0, 0.0))); // IMU is mounted sideways -> swap pitch and roll

    let mut max_base_speed: f32 = tuning::MAX_LINEAR_SPEED_DPS;
    // if Herbie is tilted in pitch, take "distance" to be previous measured distance when Herbie was level with the ground
    if !within_range(
        pitch,
        tuning::PITCH_LOWER_BOUND_DEG,
        tuning::PITCH_UPPER_BOUND_DEG,
    ) {
        front_distance = *cx.local.prev_front_distance;
    } else if !*cx.local.in_drop {
        // we are level and not in the drop trap, set speed to max
        max_base_speed = tuning::MAX_LINEAR_SPEED_DPS;
    }

    // detect entering the drop trap, set lower speed max
    if !*cx.local.in_drop && pitch < PITCH_LOWER_BOUND_DEG {
        *cx.local.in_drop = true;
        max_base_speed = tuning::MAX_LINEAR_SPEED_IN_DROP_DPS;
    }
    // detect leaving the drop trap, speed will be reset once we level out
    if *cx.local.in_drop && pitch > PITCH_UPPER_BOUND_DEG {
        *cx.local.in_drop = false;
    }

    match cx.local.state {
        State::Idle => {
            // set motor set points to 0
            cx.shared.motor_setpoints.lock(|motor_setpoints| {
                motor_setpoints.f_left = 0.0;
                motor_setpoints.r_left = 0.0;
                motor_setpoints.f_right = 0.0;
                motor_setpoints.r_right = 0.0;
            });
            // check for button press and release
            if cx.local.button.is_low() {
                while cx.local.button.is_low() {
                    asm::nop();
                }
                *cx.local.curr_leg = 0;
                *cx.local.state = State::Linear;
                cx.local.side_dist_compensation_pid.setpoint = 0.0; // set point for yaw error
                cx.local.distance_pid.setpoint =
                    sys_config::FRONT_DISTANCE_TARGETS_MM[*cx.local.curr_leg] as f32;
                cx.shared
                    .tof_front_filter
                    .lock(|tof_front_filter| tof_front_filter.reset());
                cx.shared.imu_filter.lock(|imu_filter| imu_filter.reset());
            }
        }
        State::Linear => {
            block_if_button_pressed(cx.local.button);
            // check if linear leg is complete
            if fabsf(
                (compute_dist_error(
                    front_distance,
                    sys_config::FRONT_DISTANCE_TARGETS_MM[*cx.local.curr_leg],
                )) as f32,
            ) < tuning::DISTANCE_TOLERANCE_MM as f32
                && within_range(
                    pitch,
                    tuning::PITCH_LOWER_BOUND_DEG,
                    tuning::PITCH_UPPER_BOUND_DEG,
                )
            {
                *cx.local.num_samples_within_tolerance += 1;
                if *cx.local.curr_leg == sys_config::NUM_LEGS_IN_RACE - 1 {
                    // race is complete
                    *cx.local.curr_leg = 0;
                    *cx.local.state = State::Idle;
                } else if *cx.local.num_samples_within_tolerance >= tuning::STEADY_STATE_NUM_SAMPLES
                {
                    *cx.local.curr_leg += 1;
                    *cx.local.state = State::Turning;
                }
            } else {
                *cx.local.num_samples_within_tolerance = 0;
                // compute right and left side speed set points
                let left_dist_error: i32 = compute_dist_error(
                    left_distance,
                    sys_config::LEFT_DISTANCE_TARGETS_MM[*cx.local.curr_leg],
                );
                cx.local.distance_pid.output_limit = max_base_speed;
                let base_speed = cx
                    .local
                    .distance_pid
                    .next_control_output(front_distance as f32)
                    .output;
                let dist_comp_alpha: f32 = cx
                    .local
                    .side_dist_compensation_pid
                    .next_control_output(fabsf(left_dist_error as f32))
                    .output;
                let (right_side_speed, left_side_speed): (f32, f32) = {
                    if left_dist_error < 0 {
                        // too far from wall, move left
                        (base_speed, base_speed * (1.0 - dist_comp_alpha))
                    } else {
                        // too close to wall, move right
                        (base_speed * (1.0 - dist_comp_alpha), base_speed)
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
        }
        State::Turning => {
            block_if_button_pressed(cx.local.button);
            // check if yaw set point is reached
            let yaw_error: f32 = compute_yaw_error(yaw, 90.0);
            if fabsf(yaw_error) < tuning::YAW_TOLERANCE_DEG {
                *cx.local.state = State::Linear;
                *cx.local.num_samples_within_tolerance = 0;
                cx.local.side_dist_compensation_pid.reset_integral_term();
                cx.local.distance_pid.reset_integral_term();
                cx.local.distance_pid.setpoint =
                    sys_config::FRONT_DISTANCE_TARGETS_MM[*cx.local.curr_leg] as f32;
                cx.shared
                    .tof_front_filter
                    .lock(|tof_front_filter| tof_front_filter.reset());
                cx.shared.imu_filter.lock(|imu_filter| imu_filter.reset());
            } else {
                // compute right and left side speed set points
                let base_speed = turning_speed_profile(yaw_error);
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
    }
    *cx.local.prev_front_distance = front_distance;
    // Run at 100Hz
    supervisor_task::spawn_after(Duration::<u64, 1, 1000>::millis(10)).unwrap();
}
