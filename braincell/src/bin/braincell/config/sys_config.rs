use braincell::controller::motor::{Direction, MotorDirections};
pub const TOF_FRONT_ADDRESS: u8 = 0x42;
// pub const TOF_LEFT_ADDRESS: u8 = 0x69;

pub const IMU_SMA_FILTER_SIZE: usize = 5;
pub const IMU_GYRO_BIAS_DPS: (f32, f32, f32) = (0.319, 0.034, 0.21);
pub const IMU_USE_MAG: bool = false;

pub const SECONDS_PER_TICK: f32 = 0.001;

pub const MOTOR_DIRECTIONS: MotorDirections = MotorDirections {
    f_left: Direction::Forward,
    r_left: Direction::Forward,
    f_right: Direction::Backward,
    r_right: Direction::Backward,
};
pub const NUM_LEGS_IN_RACE: usize = 11;
pub const MAX_MOTOR_SPEED_DPS: f32 = 0.0;

pub const ROBOT_CENTER_TO_TOF_MM: u16 = 100;
pub const DESIRED_OFFSET_TO_WALL_MM: u16 = 150;
pub const DISTANCE_TO_WALL_THRESHOLDS_MM: [u16; NUM_LEGS_IN_RACE] =
    [150, 150, 150, 450, 450, 450, 450, 750, 750, 750, 750];
pub const MAX_DISTANCE_MM: u16 = 1800;

pub const PITCH_LOWER_BOUND_DEG: f32 = -20.0;
pub const PITCH_UPPER_BOUND_DEG: f32 = 20.0;

// Linear speed profile function: v(x) = v_max * (1 - exp(-(x-offset)/tau))
// where tau is the time constant, offset is the desired offset between the
// robot center and the wall, and v_max is the max motor speed
pub const LINEAR_SPEED_PROFILE_TAU_MM: f32 = 100.0;

pub const YAW_TOLERANCE_DEG: f32 = 2.0;
pub const YAW_SET_POINTS_DEG: [f32; NUM_LEGS_IN_RACE] = [
    0.0, -90.0, 180.0, 90.0, 0.0, -90.0, 180.0, 90.0, 0.0, -90.0, 180.0,
];

pub const STEADY_STATE_NUM_SAMPLES: usize = 10;

// Turning PID params
pub const TURNING_PID_KP: f32 = 0.0;
pub const TURNING_PID_KI: f32 = 0.0;
pub const TURNING_PID_KD: f32 = 0.0;
pub const TURNING_PID_OUTPUT_LIMIT: f32 = MAX_MOTOR_SPEED_DPS;

// Yaw compensation PID params
pub const YAW_COMPENSATION_PID_KP: f32 = 0.0;
pub const YAW_COMPENSATION_PID_KI: f32 = 0.0;
pub const YAW_COMPENSATION_PID_KD: f32 = 0.0;
pub const YAW_COMPENSATION_PID_OUTPUT_LIMIT: f32 = MAX_MOTOR_SPEED_DPS;
