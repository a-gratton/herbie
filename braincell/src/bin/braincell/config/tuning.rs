pub const MOTOR_KP: f32 = 0.000015;
pub const MOTOR_KI: f32 = 0.003;
pub const MOTOR_KD: f32 = 0.00012;
pub const MOTOR_OUT_LIM: f32 = 100.0;
pub const MOTOR_P_LIM: f32 = MOTOR_OUT_LIM;
pub const MOTOR_I_LIM: f32 = MOTOR_OUT_LIM;
pub const MOTOR_D_LIM: f32 = MOTOR_OUT_LIM;

pub const TURNING_PID_KP: f32 = 0.0;
pub const TURNING_PID_KI: f32 = 0.0;
pub const TURNING_PID_KD: f32 = 0.0;
pub const TURNING_PID_OUT_LIM: f32 = MAX_MOTOR_SPEED_DPS;
pub const TURNING_PID_P_LIM: f32 = TURNING_PID_OUT_LIM;
pub const TURNING_PID_I_LIM: f32 = TURNING_PID_OUT_LIM;
pub const TURNING_PID_D_LIM: f32 = TURNING_PID_OUT_LIM;

pub const YAW_COMPENSATION_PID_KP: f32 = 0.0;
pub const YAW_COMPENSATION_PID_KI: f32 = 0.0;
pub const YAW_COMPENSATION_PID_KD: f32 = 0.0;
pub const YAW_COMPENSATION_PID_OUT_LIM: f32 = MAX_MOTOR_SPEED_DPS;
pub const YAW_COMPENSATION_PID_P_LIM: f32 = YAW_COMPENSATION_PID_OUT_LIM;
pub const YAW_COMPENSATION_PID_I_LIM: f32 = YAW_COMPENSATION_PID_OUT_LIM;
pub const YAW_COMPENSATION_PID_D_LIM: f32 = YAW_COMPENSATION_PID_OUT_LIM;

// Linear speed profile function: v(x) = v_max * (1 - exp(-(x-offset)/tau))
// where tau is the time constant, offset is the desired offset between the
// robot center and the wall, and v_max is the max motor speed
pub const LINEAR_SPEED_PROFILE_TAU_MM: f32 = 100.0;

pub const STEADY_STATE_NUM_SAMPLES: usize = 10;
