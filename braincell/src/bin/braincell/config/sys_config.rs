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

pub const ROBOT_CENTER_TO_TOF_MM: f32 = 100.0;
pub const DISTANCE_TO_WALL_THRESHOLDS_MM: [f32; NUM_LEGS_IN_RACE] = [
    150.0, 150.0, 150.0, 450.0, 450.0, 450.0, 450.0, 750.0, 750.0, 750.0, 750.0,
];

pub const PITCH_LOWER_BOUND_DEG: f32 = -10.0;
pub const PITCH_UPPER_BOUND_DEG: f32 = 10.0;

pub const LINEAR_SPEED_PROFILE_TAU_MM: f32 = 1.0;

pub const YAW_TOLERANCE_DEG: f32 = 4.0;
pub const YAW_CORRECTION_GAIN: f32 = 1.0;
pub const YAW_SET_POINTS_DEG: [f32; NUM_LEGS_IN_RACE] = [
    0.0, -90.0, 180.0, 90.0, 0.0, -90.0, 180.0, 90.0, 0.0, -90.0, 180.0,
];

pub const STEADY_STATE_NUM_SAMPLES: usize = 10;
