// speed control tuning params
pub const MOTOR_KP: f32 = 0.000015;
pub const MOTOR_KI: f32 = 0.003;
pub const MOTOR_KD: f32 = 0.00012;
pub const MOTOR_OUT_LIM: f32 = 100.0;
pub const MOTOR_P_LIM: f32 = MOTOR_OUT_LIM;
pub const MOTOR_I_LIM: f32 = MOTOR_OUT_LIM;
pub const MOTOR_D_LIM: f32 = MOTOR_OUT_LIM;

// filter tuning params
pub const IMU_SMA_FILTER_SIZE: usize = 5;
pub const TOF_FRONT_SMA_FILTER_SIZE: usize = 3;
pub const TOF_LEFT_SMA_FILTER_SIZE: usize = 2;
pub const IMU_FILTER_KP: f32 = 0.0;
pub const IMU_FILTER_KI: f32 = 0.0;
pub const IMU_FILTER_USE_MAG: bool = false;
pub const IMU_GYRO_BIAS_DPS: (f32, f32, f32) = (-0.015826736, 0.20015818, 0.40443522);

// ToF validity detection
pub const TOF_PITCH_LOWER_BOUND_DEG: f32 = -10.0;
pub const TOF_PITCH_UPPER_BOUND_DEG: f32 = 10.0;

// trap detection
pub const DETECTION_PITCH_LOWER_BOUND_DEG: f32 = -5.0;
pub const DETECTION_PITCH_UPPER_BOUND_DEG: f32 = 3.0;
pub const DETECTION_NUM_SAMPLES: usize = 10;
pub const MAX_LINEAR_SPEED_IN_DROP_DPS: f32 = 1000.0;

// linear base speed tuning
pub const MAX_LINEAR_SPEED_DPS: f32 = 1900.0;
pub const DISTANCE_PID_KP: f32 = 3.0;
pub const DISTANCE_PID_KI: f32 = 0.0;
pub const DISTANCE_PID_KD: f32 = 0.3;
pub const DISTANCE_PID_OUT_LIM: f32 = MAX_LINEAR_SPEED_DPS;
pub const DISTANCE_PID_P_LIM: f32 = DISTANCE_PID_OUT_LIM;
pub const DISTANCE_PID_I_LIM: f32 = DISTANCE_PID_OUT_LIM;
pub const DISTANCE_PID_D_LIM: f32 = DISTANCE_PID_OUT_LIM;
pub const DISTANCE_TOLERANCE_MM: i32 = 30;

// linear path correction tuning
pub const SIDE_DIST_COMPENSATION_PID_KP: f32 = 0.0015;
pub const SIDE_DIST_COMPENSATION_PID_KI: f32 = 0.0;
pub const SIDE_DIST_COMPENSATION_PID_KD: f32 = 0.0;
pub const SIDE_DIST_COMPENSATION_PID_OUT_LIM: f32 = 1.0;
pub const SIDE_DIST_COMPENSATION_PID_P_LIM: f32 = SIDE_DIST_COMPENSATION_PID_OUT_LIM;
pub const SIDE_DIST_COMPENSATION_PID_I_LIM: f32 = SIDE_DIST_COMPENSATION_PID_OUT_LIM;
pub const SIDE_DIST_COMPENSATION_PID_D_LIM: f32 = SIDE_DIST_COMPENSATION_PID_OUT_LIM;

// turning profile tuning
pub const MAX_TURNING_SPEED_DPS: f32 = 400.0;
pub const TURNING_SPEED_SLOPE: f32 = 3.9;
pub const YAW_TOLERANCE_DEG: f32 = 4.0;
pub const OL_INNER_YAW_TOL_DEG: f32 = 2.0;

// smooth acceleration (set to 1 to disable)
pub const SMOOTH_ACCEL_NUM_SAMPLES: usize = 1;

// steady-state samples required for transition
pub const STATE_TRANSITION_SAMPLES: usize = 3;

// angle correction frequency (every n samples at 100 Hz)
pub const ANGLE_CORRECT_SAMPLES: usize = 40;

// leg to start open loop inner turns
pub const INNER_TURNS_LEG: usize = 8;
