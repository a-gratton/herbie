use braincell::controller::motor::{Direction, MotorDirections};
pub const TOF_FRONT_ADDRESS: u8 = 0x42;
// pub const TOF_LEFT_ADDRESS: u8 = 0x69;

pub const IMU_SMA_FILTER_SIZE: usize = 5;
pub const CALIBRATE_IMU: bool = true;
pub const IMU_CALIBRATION_NUM_SAMPLES: u32 = 10000;
pub const DEFAULT_IMU_GYRO_BIAS_DPS: (f32, f32, f32) = (0.26560888, 0.29839727, 0.2897151);

pub const SECONDS_PER_TICK: f32 = 0.001;

pub const MOTOR_DIRECTIONS: MotorDirections = MotorDirections {
    f_left: Direction::Forward,
    r_left: Direction::Forward,
    f_right: Direction::Backward,
    r_right: Direction::Backward,
};
