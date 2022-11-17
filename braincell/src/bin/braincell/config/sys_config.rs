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
//     [120, 120, 120, 420, 420, 420, 450, 750, 750, 750, 750];
    pub const FRONT_DISTANCE_TARGETS_MM: [i32; NUM_LEGS_IN_RACE] =
    [120, 120, 120, 420,];
// pub const FRONT_STARTING_DISTANCES_MM: [i32; NUM_LEGS_IN_RACE] = [
//     1200, 1500, 1500, 1500, 1500, 1200, 1200, 1200, 1200, 900, 900,
// ];
pub const FRONT_STARTING_DISTANCES_MM: [i32; NUM_LEGS_IN_RACE] = [
    1200, 1500, 1500, 1500,
];
// pub const LEFT_DISTANCE_TARGETS_MM: [i32; NUM_LEGS_IN_RACE] =
//     [95, 95, 95, 95, 395, 395, 425, 425, 725, 725, 725];
pub const LEFT_DISTANCE_TARGETS_MM: [i32; NUM_LEGS_IN_RACE] =
    [95, 95, 95, 95,];
    // pub const TURNING_YAW_TARGETS_DEG: [f32; NUM_LEGS_IN_RACE] = [
//     90.0, 90.0, 90.0, 90.0, 93.0, 94.0, 95.0, 95.0, 92.0, 92.0, 92.0,
// ];
pub const TURNING_YAW_TARGETS_DEG: [f32; NUM_LEGS_IN_RACE] = [
    90.0, 90.0, 90.0, 90.0,
];

// pub const FRONT_DISTANCE_TARGETS_MM: [i32; NUM_LEGS_IN_RACE] =
//     [120, 120, 120, 420];
// pub const FRONT_STARTING_DISTANCES_MM: [i32; NUM_LEGS_IN_RACE] =
//     [1200, 1500, 1500, 1500];
// pub const LEFT_DISTANCE_TARGETS_MM: [i32; NUM_LEGS_IN_RACE] =
//     [95, 95, 95, 95];

// pub const FRONT_DISTANCE_TARGETS_MM: [i32; NUM_LEGS_IN_RACE] =
//     [420, 420, 450, 750, 750, 750, 750];
// pub const FRONT_STARTING_DISTANCES_MM: [i32; NUM_LEGS_IN_RACE] =
//     [1500, 1200, 1200, 1200, 1200, 900, 900];
// pub const LEFT_DISTANCE_TARGETS_MM: [i32; NUM_LEGS_IN_RACE] =
//     [395, 395, 395, 395, 695, 695, 695];

// pub const FRONT_DISTANCE_TARGETS_MM: [i32; NUM_LEGS_IN_RACE] =
//     [120, 120];
// pub const FRONT_STARTING_DISTANCES_MM: [i32; NUM_LEGS_IN_RACE] =
//     [1200, 1500];
// pub const LEFT_DISTANCE_TARGETS_MM: [i32; NUM_LEGS_IN_RACE] =
//     [95, 95];

pub const MIN_TOF_DISTANCE_MM: i32 = 40;
pub const MAX_TOF_DISTANCE_MM: i32 = 1800;
