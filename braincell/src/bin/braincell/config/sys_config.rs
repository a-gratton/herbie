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
pub const DESIRED_OFFSET_TO_WALL_MM: f32 = 150.0;
pub const DISTANCE_TO_WALL_THRESHOLDS_MM: [u16; NUM_LEGS_IN_RACE] =
    [150, 150, 150, 450, 450, 450, 450, 750, 750, 750, 750];
pub const MAX_DISTANCE_MM: u16 = 1800;

pub const PITCH_LOWER_BOUND_DEG: f32 = -20.0;
pub const PITCH_UPPER_BOUND_DEG: f32 = 20.0;

pub const YAW_TOLERANCE_DEG: f32 = 2.0;
pub const YAW_SET_POINTS_DEG: [f32; NUM_LEGS_IN_RACE] = [
    0.0, -90.0, 180.0, 90.0, 0.0, -90.0, 180.0, 90.0, 0.0, -90.0, 180.0,
];
