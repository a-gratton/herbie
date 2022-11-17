use crate::app::supervisor_task;
use crate::config::{sys_config, tuning};
use cortex_m::asm;
use embedded_hal::digital::v2::InputPin;
use libm::{fabsf, fminf};
use num_traits::float::FloatCore;
use pid::Pid;
use rtic::Mutex;
use systick_monotonic::fugit::Duration;

pub struct Data<P: InputPin, FloatT: FloatCore> {
    button: P,
    state: State,
    curr_leg: usize,
    smooth_accel_samples: usize,
    detection_samples_within_tolerance: usize,
    distance_pid: pid::Pid<FloatT>,
    side_dist_compensation_pid: pid::Pid<FloatT>,
    prev_front_distance: i32,
    in_drop: bool,
    max_base_speed: f32,
}
impl<P, FloatT> Data<P, FloatT>
where
    P: InputPin,
    FloatT: FloatCore,
{
    pub fn new(button: P, distance_pid: Pid<FloatT>, side_dist_comp_pid: Pid<FloatT>) -> Self {
        Data {
            button: button,
            state: State::Idle,
            curr_leg: 0,
            smooth_accel_samples: 0,
            detection_samples_within_tolerance: 0,
            distance_pid: distance_pid,
            side_dist_compensation_pid: side_dist_comp_pid,
            prev_front_distance: sys_config::MAX_TOF_DISTANCE_MM,
            in_drop: false,
            max_base_speed: tuning::MAX_LINEAR_SPEED_DPS,
        }
    }
}

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
    let mut front_distance: i32 = cx.local.supervisor_state.prev_front_distance;
    let mut left_distance: i32 = sys_config::MIN_TOF_DISTANCE_MM;
    cx.shared.tof_front_filter.lock(|tof_front_filter| {
        front_distance = tof_front_filter
            .filtered()
            .unwrap_or(cx.local.supervisor_state.prev_front_distance)
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

    // cx.shared
    //     .tx
    //     .lock(|tx| writeln!(tx, "pitch {pitch} yaw {yaw}"));

    // if Herbie is tilted in pitch, take "distance" to be previous measured distance when Herbie was level with the ground
    if !within_range(
        pitch,
        tuning::TOF_PITCH_LOWER_BOUND_DEG,
        tuning::TOF_PITCH_UPPER_BOUND_DEG,
    ) {
        front_distance = cx.local.supervisor_state.prev_front_distance;
    }
    if within_range(
        pitch,
        tuning::DETECTION_PITCH_LOWER_BOUND_DEG,
        tuning::DETECTION_PITCH_UPPER_BOUND_DEG,
    ) && !cx.local.supervisor_state.in_drop
    {
        cx.local.supervisor_state.detection_samples_within_tolerance += 1;

        if cx.local.supervisor_state.detection_samples_within_tolerance
            == tuning::DETECTION_NUM_SAMPLES
        {
            // we are level and not in the drop trap, set speed to max
            cx.local.supervisor_state.max_base_speed = tuning::MAX_LINEAR_SPEED_DPS;
            cx.local.supervisor_state.detection_samples_within_tolerance = 0;
        }
    }

    // detect entering the drop trap, set lower speed max
    if !cx.local.supervisor_state.in_drop && pitch < tuning::DETECTION_PITCH_LOWER_BOUND_DEG {
        cx.local.supervisor_state.in_drop = true;
        cx.local.supervisor_state.max_base_speed = tuning::MAX_LINEAR_SPEED_IN_DROP_DPS;
    }
    // detect leaving the drop trap, speed will be reset once we level out
    if cx.local.supervisor_state.in_drop && pitch > tuning::DETECTION_PITCH_UPPER_BOUND_DEG {
        cx.local.supervisor_state.in_drop = false;
    }

    match cx.local.supervisor_state.state {
        State::Idle => {
            // set motor set points to 0
            cx.shared.motor_setpoints.lock(|motor_setpoints| {
                motor_setpoints.f_left = 0.0;
                motor_setpoints.r_left = 0.0;
                motor_setpoints.f_right = 0.0;
                motor_setpoints.r_right = 0.0;
            });
            // check for button press and release
            if cx.local.supervisor_state.button.is_low() {
                while cx.local.supervisor_state.button.is_low() {
                    asm::nop();
                }
                asm::delay(4_000_000);
                cx.local.supervisor_state.curr_leg = 0;
                cx.local.supervisor_state.state = State::Linear;
                cx.local
                    .supervisor_state
                    .side_dist_compensation_pid
                    .setpoint = 0.0; // set point for yaw error
                cx.local.supervisor_state.distance_pid.setpoint =
                    sys_config::FRONT_DISTANCE_TARGETS_MM[cx.local.supervisor_state.curr_leg]
                        as f32;
                cx.shared
                    .tof_front_filter
                    .lock(|tof_front_filter| tof_front_filter.reset());
                cx.shared
                    .tof_left_filter
                    .lock(|tof_left_filter| tof_left_filter.reset());
                cx.shared.imu_filter.lock(|imu_filter| imu_filter.reset());
            }
        }
        State::Linear => {
            block_if_button_pressed(&cx.local.supervisor_state.button);
            // check if linear leg is complete
            if fabsf(
                (compute_dist_error(
                    front_distance,
                    sys_config::FRONT_DISTANCE_TARGETS_MM[cx.local.supervisor_state.curr_leg],
                )) as f32,
            ) < tuning::DISTANCE_TOLERANCE_MM as f32
                && within_range(
                    pitch,
                    tuning::TOF_PITCH_LOWER_BOUND_DEG,
                    tuning::TOF_PITCH_UPPER_BOUND_DEG,
                )
            {
                if cx.local.supervisor_state.curr_leg == sys_config::NUM_LEGS_IN_RACE - 1 {
                    // race is complete
                    cx.local.supervisor_state.curr_leg = 0;
                    cx.local.supervisor_state.state = State::Idle;
                } else {
                    cx.shared.motor_setpoints.lock(|motor_setpoints| {
                        motor_setpoints.f_right = 0.0;
                        motor_setpoints.r_right = 0.0;
                        motor_setpoints.f_left = 0.0;
                        motor_setpoints.r_left = 0.0;
                    });
                    cx.local.supervisor_state.curr_leg += 1;
                    cx.local.supervisor_state.state = State::Turning;
                    asm::delay(100_000);
                }
            } else {
                cx.local.supervisor_state.smooth_accel_samples += 1;
                // compute right and left side speed set points
                let left_dist_error: i32 = compute_dist_error(
                    left_distance,
                    sys_config::LEFT_DISTANCE_TARGETS_MM[cx.local.supervisor_state.curr_leg],
                );
                cx.local.supervisor_state.distance_pid.output_limit =
                    cx.local.supervisor_state.max_base_speed;
                cx.local.supervisor_state.distance_pid.p_limit =
                    cx.local.supervisor_state.max_base_speed;
                cx.local.supervisor_state.distance_pid.i_limit =
                    cx.local.supervisor_state.max_base_speed;
                cx.local.supervisor_state.distance_pid.d_limit =
                    cx.local.supervisor_state.max_base_speed;
                let base_speed = if cx.local.supervisor_state.in_drop {
                    -cx.local.supervisor_state.max_base_speed
                } else {
                    cx.local
                        .supervisor_state
                        .distance_pid
                        .next_control_output(front_distance as f32)
                        .output
                };
                if cx.local.supervisor_state.smooth_accel_samples
                    == tuning::SMOOTH_ACCEL_NUM_SAMPLES
                {
                    cx.local.supervisor_state.distance_pid.reset_integral_term();
                }
                let dist_comp_alpha: f32 = cx
                    .local
                    .supervisor_state
                    .side_dist_compensation_pid
                    .next_control_output(fabsf(left_dist_error as f32))
                    .output;
                let (right_side_speed, left_side_speed): (f32, f32) = {
                    let left_accel_comp = fminf(
                        cx.local.supervisor_state.smooth_accel_samples as f32
                            / tuning::SMOOTH_ACCEL_NUM_SAMPLES as f32,
                        1.0,
                    );
                    if left_dist_error < 0 {
                        // too far from wall, move left
                        (
                            base_speed,
                            base_speed * (1.0 - dist_comp_alpha) * left_accel_comp,
                        )
                    } else {
                        // too close to wall, move right
                        (
                            base_speed * (1.0 - dist_comp_alpha),
                            base_speed * left_accel_comp,
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
        }
        State::Turning => {
            block_if_button_pressed(&cx.local.supervisor_state.button);
            // check if yaw set point is reached
            let yaw_error: f32 = compute_yaw_error(
                yaw,
                sys_config::TURNING_YAW_TARGETS_DEG[cx.local.supervisor_state.curr_leg],
            );
            if fabsf(yaw_error) < tuning::YAW_TOLERANCE_DEG {
                cx.shared.motor_setpoints.lock(|motor_setpoints| {
                    motor_setpoints.f_right = 0.0;
                    motor_setpoints.r_right = 0.0;
                    motor_setpoints.f_left = 0.0;
                    motor_setpoints.r_left = 0.0;
                });
                asm::delay(100_000);
                cx.local.supervisor_state.smooth_accel_samples = 0;
                cx.local
                    .supervisor_state
                    .side_dist_compensation_pid
                    .reset_integral_term();
                cx.local.supervisor_state.distance_pid.reset_integral_term();
                cx.local.supervisor_state.distance_pid.setpoint =
                    sys_config::FRONT_DISTANCE_TARGETS_MM[cx.local.supervisor_state.curr_leg]
                        as f32;
                cx.shared
                    .tof_front_filter
                    .lock(|tof_front_filter| tof_front_filter.reset());
                cx.shared
                    .tof_left_filter
                    .lock(|tof_left_filter| tof_left_filter.reset());
                cx.shared.imu_filter.lock(|imu_filter| imu_filter.reset());
                cx.local.supervisor_state.state = State::Linear;
                cx.local.supervisor_state.prev_front_distance =
                    sys_config::FRONT_STARTING_DISTANCES_MM[cx.local.supervisor_state.curr_leg];
                cx.local.supervisor_state.in_drop = false;
                cx.local.supervisor_state.max_base_speed = tuning::MAX_LINEAR_SPEED_DPS;
                cx.local.supervisor_state.detection_samples_within_tolerance = 0;
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
    cx.local.supervisor_state.prev_front_distance = front_distance;
    // Run at 100Hz
    supervisor_task::spawn_after(Duration::<u64, 1, 1000>::millis(10)).unwrap();
}
