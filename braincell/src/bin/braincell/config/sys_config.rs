use braincell::controller::motor::{Direction, MotorDirections};
pub const TOF_FRONT_ADDRESS: u8 = 0x42;
// pub const TOF_LEFT_ADDRESS: u8 = 0x69;

pub const SECONDS_PER_TICK: f32 = 0.001;

pub const MOTOR_DIRECTIONS: MotorDirections = MotorDirections {
    f_left: Direction::Forward,
    r_left: Direction::Forward,
    f_right: Direction::Backward,
    r_right: Direction::Backward,
};

pub const NUM_LEGS_IN_RACE: usize = 11;

pub const ROBOT_CENTER_TO_TOF_MM: i32 = 100;
pub const DISTANCE_TARGETS_MM: [i32; NUM_LEGS_IN_RACE] =
    [150, 150, 150, 450, 450, 450, 450, 750, 750, 750, 750];
pub const MAX_DISTANCE_MM: i32 = 1800;
