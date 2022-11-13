use braincell::controller::motor::{Direction, MotorDirections};
pub const TOF_FRONT_ADDRESS: u8 = 0x42;
pub const TOF_LEFT_ADDRESS: u8 = 0x69;

pub const SECONDS_PER_TICK: f32 = 0.001;

pub const MOTOR_DIRECTIONS: MotorDirections = MotorDirections {
    f_left: Direction::Backward,
    r_left: Direction::Backward,
    f_right: Direction::Forward,
    r_right: Direction::Forward,
};

pub const NUM_LEGS_IN_RACE: usize = 4;

// pub const FRONT_DISTANCE_TARGETS_MM: [i32; NUM_LEGS_IN_RACE] =
//     [120, 120, 120, 420, 420, 420, 420, 720, 720, 720, 720];
// pub const LEFT_DISTANCE_TARGETS_MM: [i32; NUM_LEGS_IN_RACE] =
//     [70, 70, 70, 70, 370, 370, 370, 370, 670, 670, 670]; 
pub const FRONT_DISTANCE_TARGETS_MM: [i32; NUM_LEGS_IN_RACE] =
    [120, 120, 120, 420];
pub const LEFT_DISTANCE_TARGETS_MM: [i32; NUM_LEGS_IN_RACE] =
    [90, 90, 90, 90];
pub const MAX_TOF_DISTANCE_MM: i32 = 1800;
pub const MIN_TOF_DISTANCE_MM: i32 = 40;
