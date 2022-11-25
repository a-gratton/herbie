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

pub const NUM_LEGS_IN_RACE: usize = 11;
pub const DROP_LEG: usize = 3;

pub const FRONT_DISTANCE_TARGETS_MM: [i32; NUM_LEGS_IN_RACE] =
    [100, 100, 80, 400, 410, 410, 440, 740, 760, 760, 780];

pub const FRONT_STARTING_DISTANCES_MM: [i32; NUM_LEGS_IN_RACE] = [
    1200, 1500, 1500, 1500, 1500, 1200, 1200, 1200, 1200, 900, 900,
];
pub const LEFT_DISTANCE_TARGETS_MM: [i32; NUM_LEGS_IN_RACE] =
    [102, 102, 102, 102, 412, 412, 442, 442, 762, 762, 762];

// pub const FRONT_DISTANCE_TARGETS_MM: [i32; NUM_LEGS_IN_RACE] =
// [410, 410, 440, 740, 760, 760, 780];

// pub const FRONT_STARTING_DISTANCES_MM: [i32; NUM_LEGS_IN_RACE] = [
//     1500, 1200, 1200, 1200, 1200, 900, 900,
// ];
// pub const LEFT_DISTANCE_TARGETS_MM: [i32; NUM_LEGS_IN_RACE] =
//     [412, 412, 442, 442, 762, 762, 762];

pub const MIN_TOF_DISTANCE_MM: i32 = 40;
pub const MAX_TOF_DISTANCE_MM: i32 = 1800;
