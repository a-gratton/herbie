use super::vl53l1x::DistanceMode;
use super::vl53l1x::TimingBudget;

// configuration constants
pub const TARGET_RATE: u16 = 0x0A00;
pub const DEFAULT_DM: DistanceMode = DistanceMode::Short; // short range mode
pub const DEFAULT_TB: TimingBudget = TimingBudget::Tb15ms; // default timing budget (ms)
pub const DEFAULT_IM_MS: u32 = 50; // default inter-measurement period (ms)
pub const DEFAULT_I2C_ADDR: u8 = 0x52 >> 1;
pub const SENSOR_ID: u16 = 0xeacc;
pub const OFFSET_VALUE: i16 = 27;

/*
0 = no error,
1 = sigma failure,
2 = signal failure,
4 = sensor out-of-bounds,
7 = wraparound
*/
pub const STATUS_RTN: [u8; 24] = [
    255, 255, 255, 5, 2, 4, 1, 7, 3, 0, 255, 255, 9, 13, 255, 255, 255, 255, 10, 6, 255, 255, 11,
    12,
];

pub const VL51L1X_DEFAULT_CONFIGURATION: [u8; 91] = [
    0x00, /* 0x2d : set bit 2 and 5 to 1 for fast plus mode (1MHz I2C), else don't touch */
    0x00, /* 0x2e : bit 0 if I2C pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
    0x00, /* 0x2f : bit 0 if GPIO pulled up at 1.8V, else set bit 0 to 1 (pull up at AVDD) */
    0x01, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low (bits 3:0 must be 0x1), use SetInterruptPolarity() */
    0x02, /* 0x31 : bit 1 = interrupt depending on the polarity, use CheckForDataReady() */
    0x00, /* 0x32 : not user-modifiable */
    0x02, /* 0x33 : not user-modifiable */
    0x08, /* 0x34 : not user-modifiable */
    0x00, /* 0x35 : not user-modifiable */
    0x08, /* 0x36 : not user-modifiable */
    0x10, /* 0x37 : not user-modifiable */
    0x01, /* 0x38 : not user-modifiable */
    0x01, /* 0x39 : not user-modifiable */
    0x00, /* 0x3a : not user-modifiable */
    0x00, /* 0x3b : not user-modifiable */
    0x00, /* 0x3c : not user-modifiable */
    0x00, /* 0x3d : not user-modifiable */
    0xff, /* 0x3e : not user-modifiable */
    0x00, /* 0x3f : not user-modifiable */
    0x0F, /* 0x40 : not user-modifiable */
    0x00, /* 0x41 : not user-modifiable */
    0x00, /* 0x42 : not user-modifiable */
    0x00, /* 0x43 : not user-modifiable */
    0x00, /* 0x44 : not user-modifiable */
    0x00, /* 0x45 : not user-modifiable */
    0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high, 2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
    0x0b, /* 0x47 : not user-modifiable */
    0x00, /* 0x48 : not user-modifiable */
    0x00, /* 0x49 : not user-modifiable */
    0x02, /* 0x4a : not user-modifiable */
    0x0a, /* 0x4b : not user-modifiable */
    0x21, /* 0x4c : not user-modifiable */
    0x00, /* 0x4d : not user-modifiable */
    0x00, /* 0x4e : not user-modifiable */
    0x05, /* 0x4f : not user-modifiable */
    0x00, /* 0x50 : not user-modifiable */
    0x00, /* 0x51 : not user-modifiable */
    0x00, /* 0x52 : not user-modifiable */
    0x00, /* 0x53 : not user-modifiable */
    0xc8, /* 0x54 : not user-modifiable */
    0x00, /* 0x55 : not user-modifiable */
    0x00, /* 0x56 : not user-modifiable */
    0x38, /* 0x57 : not user-modifiable */
    0xff, /* 0x58 : not user-modifiable */
    0x01, /* 0x59 : not user-modifiable */
    0x00, /* 0x5a : not user-modifiable */
    0x08, /* 0x5b : not user-modifiable */
    0x00, /* 0x5c : not user-modifiable */
    0x00, /* 0x5d : not user-modifiable */
    0x01, /* 0x5e : not user-modifiable */
    0xcc, /* 0x5f : not user-modifiable */
    0x0f, /* 0x60 : not user-modifiable */
    0x01, /* 0x61 : not user-modifiable */
    0xf1, /* 0x62 : not user-modifiable */
    0x0d, /* 0x63 : not user-modifiable */
    0x01, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB), use SetSigmaThreshold(), default value 90 mm  */
    0x68, /* 0x65 : Sigma threshold LSB */
    0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB), use SetSignalThreshold() */
    0x80, /* 0x67 : Min count Rate LSB */
    0x08, /* 0x68 : not user-modifiable */
    0xb8, /* 0x69 : not user-modifiable */
    0x00, /* 0x6a : not user-modifiable */
    0x00, /* 0x6b : not user-modifiable */
    0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register, use SetIntermeasurementInMs() */
    0x00, /* 0x6d : Intermeasurement period */
    0x0f, /* 0x6e : Intermeasurement period */
    0x89, /* 0x6f : Intermeasurement period LSB */
    0x00, /* 0x70 : not user-modifiable */
    0x00, /* 0x71 : not user-modifiable */
    0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB), use SetD:tanceThreshold() */
    0x00, /* 0x73 : distance threshold high LSB */
    0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB), use SetD:tanceThreshold() */
    0x00, /* 0x75 : distance threshold low LSB */
    0x00, /* 0x76 : not user-modifiable */
    0x01, /* 0x77 : not user-modifiable */
    0x0f, /* 0x78 : not user-modifiable */
    0x0d, /* 0x79 : not user-modifiable */
    0x0e, /* 0x7a : not user-modifiable */
    0x0e, /* 0x7b : not user-modifiable */
    0x00, /* 0x7c : not user-modifiable */
    0x00, /* 0x7d : not user-modifiable */
    0x02, /* 0x7e : not user-modifiable */
    0xc7, /* 0x7f : ROI center, use SetROI() */
    0xff, /* 0x80 : XY ROI (X=Width, Y=Height), use SetROI() */
    0x9B, /* 0x81 : not user-modifiable */
    0x00, /* 0x82 : not user-modifiable */
    0x00, /* 0x83 : not user-modifiable */
    0x00, /* 0x84 : not user-modifiable */
    0x01, /* 0x85 : not user-modifiable */
    0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
    0x00, /* 0x87 : start ranging, use StartRanging() or StopRanging(), If you want an automatic start after VL53L1X_init() call, put 0x40 in location 0x87 */
];

// register access constants
pub const SOFT_RESET: [u8; 2] = [0x00, 0x00];
pub const I2C_SLAVE__DEVICE_ADDRESS: [u8; 2] = [0x00, 0x01];
pub const VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND: [u8; 2] = [0x00, 0x08];
pub const ALGO__PART_TO_PART_RANGE_OFFSET_MM: [u8; 2] = [0x00, 0x1E];
pub const MM_CONFIG__INNER_OFFSET_MM: [u8; 2] = [0x00, 0x20];
pub const MM_CONFIG__OUTER_OFFSET_MM: [u8; 2] = [0x00, 0x22];
pub const DSS_CONFIG__TARGET_TOTAL_RATE_MCPS: [u8; 2] = [0x00, 0x24];
pub const PAD_I2C_HV__EXTSUP_CONFIG: [u8; 2] = [0x00, 0x2E];
pub const GPIO__TIO_HV_STATUS: [u8; 2] = [0x00, 0x31];
pub const SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS: [u8; 2] = [0x00, 0x36];
pub const SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS: [u8; 2] = [0x00, 0x37];
pub const ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM: [u8; 2] = [0x00, 0x39];
pub const ALGO__RANGE_IGNORE_VALID_HEIGHT_MM: [u8; 2] = [0x00, 0x3E];
pub const ALGO__RANGE_MIN_CLIP: [u8; 2] = [0x00, 0x3F];
pub const ALGO__CONSISTENCY_CHECK__TOLERANCE: [u8; 2] = [0x00, 0x40];
pub const PHASECAL_CONFIG__TIMEOUT_MACROP: [u8; 2] = [0x00, 0x4B];
pub const DSS_CONFIG__ROI_MODE_CONTROL: [u8; 2] = [0x00, 0x4F];
pub const SYSTEM__THRESH_RATE_HIGH: [u8; 2] = [0x00, 0x50];
pub const SYSTEM__THRESH_RATE_LOW: [u8; 2] = [0x00, 0x52];
pub const DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT: [u8; 2] = [0x00, 0x54];
pub const DSS_CONFIG__APERTURE_ATTENUATION: [u8; 2] = [0x00, 0x57];
pub const RANGE_CONFIG__TIMEOUT_MACROP_A_HI: [u8; 2] = [0x00, 0x5E];
pub const RANGE_CONFIG__VCSEL_PERIOD_A: [u8; 2] = [0x00, 0x60];
pub const RANGE_CONFIG__TIMEOUT_MACROP_B_HI: [u8; 2] = [0x00, 0x61];
pub const RANGE_CONFIG__VCSEL_PERIOD_B: [u8; 2] = [0x00, 0x63];
pub const RANGE_CONFIG__SIGMA_THRESH: [u8; 2] = [0x00, 0x64];
pub const RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS: [u8; 2] = [0x00, 0x66];
pub const RANGE_CONFIG__VALID_PHASE_HIGH: [u8; 2] = [0x00, 0x69];
pub const SYSTEM__INTERMEASUREMENT_PERIOD: [u8; 2] = [0x00, 0x6C];
pub const SYSTEM__GROUPED_PARAMETER_HOLD_0: [u8; 2] = [0x00, 0x71];
pub const SYSTEM__SEED_CONFIG: [u8; 2] = [0x00, 0x77];
pub const SD_CONFIG__WOI_SD0: [u8; 2] = [0x00, 0x78];
pub const SD_CONFIG__INITIAL_PHASE_SD0: [u8; 2] = [0x00, 0x7A];
pub const SYSTEM__GROUPED_PARAMETER_HOLD_1: [u8; 2] = [0x00, 0x7C];
pub const SD_CONFIG__QUANTIFIER: [u8; 2] = [0x00, 0x7E];
pub const SYSTEM__SEQUENCE_CONFIG: [u8; 2] = [0x00, 0x81];
pub const SYSTEM__GROUPED_PARAMETER_HOLD: [u8; 2] = [0x00, 0x82];
pub const SYSTEM__INTERRUPT_CLEAR: [u8; 2] = [0x00, 0x86];
pub const SYSTEM__MODE_START: [u8; 2] = [0x00, 0x87];
pub const RESULT__RANGE_STATUS: [u8; 2] = [0x00, 0x89];
pub const RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0: [u8; 2] = [0x00, 0x96];
pub const RESULT__OSC_CALIBRATE_VAL: [u8; 2] = [0x00, 0xDE];
pub const FIRMWARE__SYSTEM_STATUS: [u8; 2] = [0x00, 0xE5];
pub const IDENTIFICATION__MODEL_ID: [u8; 2] = [0x01, 0x0F];
