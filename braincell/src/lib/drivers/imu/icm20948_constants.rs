// ICM-20948 Registers and Configs

#[repr(u8)]
pub enum RegAddrGeneral {
    BankSel = 0x7F,
}

#[repr(u8)]
pub enum RegAddrBank0 {
    WhoAmI = 0x00,
    UserCtrl = 0x03,
    PwrMgmt1 = 0x06,
    IntPinConfig = 0x0F,
    IntEnable1 = 0x11,
    I2CMstStatus = 0x17,
    IntStatus = 0x19,
    IntStatus1 = 0x1A,
    AccelXoutH = 0x2D,
}

#[repr(u8)]
pub enum RegAddrBank2 {
    GyroConfig1 = 0x01,
    AccelConfig = 0x14,
}

#[repr(u8)]
pub enum RegAddrBank3 {
    I2CMstCtrl = 0x01,
    I2CPeriph0Addr = 0x03,
    I2CPeriph0Reg = 0x04,
    I2CPeriph0Ctrl = 0x05,
    I2CPeriph4Addr = 0x13,
    I2CPeriph4Reg = 0x14,
    I2CPeriph4Ctrl = 0x15,
    I2CPeriph4DO = 0x16,
    I2CPeriph4DI = 0x17,
}

#[repr(u8)]
pub enum RegAddrMag {
    WIA1 = 0x00,
    WIA2 = 0x01,
    ST1 = 0x10,
    Cntl12 = 0x31,
    Cntl3 = 0x32,
}

// PwrMgmt1 register:
// Bits:     |       7      |   6   |   5   |     4    |     3    |   2:0  |
// Function: | DEVICE_RESET | SLEEP | LP_EN | reserved | TEMP_DIS | CLKSEL |
#[repr(u8)]
#[allow(dead_code)]
pub enum PwrMgmt1Bits {
    ClkSel = 0x07 << 0,
    TempDis = 0x01 << 3,
    LPEnable = 0x01 << 5,
    Sleep = 0x01 << 6,
    DeviceReset = 0x01 << 7,
}

// AccelConfig register:
// Bits:     |    7:6   |       5:3     |      2:1     |       0       |
// Function: | reserved | ACCEL_DLPFCFG | ACCEL_FS_SEL | ACCEL_FCHOICE |
#[repr(u8)]
#[allow(dead_code)]
pub enum AccelConfigBits {
    AccelFChoice = 0x01 << 0,
    AccelFSSel = 0x03 << 1,
    AccelDLPFCFG = 0x07 << 3,
}

// GyroConfig1 register:
// Bits:     |    7:6   |      5:3     |     2:1     |       0      |
// Function: | reserved | GYRO_DLPFCFG | GYRO_FS_SEL | GYRO_FCHOICE |
#[repr(u8)]
#[allow(dead_code)]
pub enum GyroConfig1Bits {
    GyroFChoice = 0x01 << 0,
    GyroFSSel = 0x03 << 1,
    GyroDLPFCFG = 0x07 << 3,
}

// IntPinConfig register:
// Bits:     |     7     |     6     |        5      |         4        |      3     |          2        |     1     |     0    |
// Function: | INT1_ACTL | INT1_OPEN | INT1_LATCH_EN | INT_ANYRD_2CLEAR | ACTL_FSYNC | FSYNC_INT_MODE_EN | BYPASS_EN | reserved |
#[repr(u8)]
#[allow(dead_code)]
pub enum IntPinConfigBits {
    ByPassEnable = 0x01 << 1,
    FsyncIntModeEnable = 0x01 << 2,
    ActiveLowFsync = 0x01 << 3,
    IntAnyReadToClear = 0x01 << 4,
    Int1LatchEnable = 0x01 << 5,
    Int1Open = 0x01 << 6,
    Int1ActiveLow = 0x01 << 7,
}

// IntEnable1 register:
// Bits:      |    7:1   |         0         |
// Function:  | reserved | RAW_DATA_0_RDY_EN |
#[repr(u8)]
#[allow(dead_code)]
pub enum IntEnable1Bits {
    RawData0Ready = 0x01 << 0,
}

// Periph0Ctrl register:
// Bits:     |  7 |    6    |    5    |  4  |  3:0 |
// Function: | EN | BYTE_SW | REG_DIS | GRP | LENG |
#[repr(u8)]
#[allow(dead_code)]
pub enum Periph0CtrlBits {
    Length = 0x17 << 0,
    Grp = 0x01 << 4,
    RegDis = 0x01 << 5,
    ByteSwap = 0x01 << 6,
    Enable = 0x01 << 7,
}

// I2CMstCtrl register:
// Bits:     |      7      |    6:5   |       4       |     3:0     |
// Function: | MULT_MST_EN | reserved | I2C_MST_P_NSR | I2C_MST_CLK |
#[repr(u8)]
#[allow(dead_code)]
pub enum I2CMstCtrlBits {
    I2CMstClk = 0x17 << 0,
    I2CMstPNsr = 0x01 << 4,
    MultMstEnable = 0x01 << 7,
}

// UserCtrl register:
// Bits:     |    7   |    6    |      5     |      4     |    3    |    2     |      1      |     0    |
// Function: | DMP_EN | FIFO_EN | I2C_MST_EN | I2C_IF_DIS | DMP_RST | SRAM_RST | I2C_MST_RST | reserved |
#[repr(u8)]
#[allow(dead_code)]
pub enum UserCtrlBits {
    I2CMstRst = 0x01 << 1,
    SRAMRst = 0x01 << 2,
    DMPRst = 0x01 << 3,
    I2CIfDis = 0x01 << 4,
    I2CMstEnable = 0x01 << 5,
    FifoEnable = 0x01 << 6,
    DMPEnable = 0x01 << 7,
}

// Periph4Ctrl register:
// Bits:     |  7 |    6   |    5    | 4:0 |
// Function: | EN | INT_EN | REG_DIS | DLY |
#[repr(u8)]
#[allow(dead_code)]
pub enum Periph4CtrlBits {
    Delay = 0x37 << 0,
    RegDis = 0x01 << 5,
    IntEnable = 0x01 << 6,
    Enable = 0x01 << 7,
}

// I2CMstStatus register:
// Bits:     |       7      |         6        |       5      |         4        |         3        |         2        |         1        |         0        |
// Function: | PASS_THROUGH | I2C_PERIPH4_DONE | I2C_LOST_ARB | I2C_PERIPH4_NACK | I2C_PERIPH3_NACK | I2C_PERIPH2_NACK | I2C_PERIPH1_NACK | I2C_PERIPH0_NACK |
#[repr(u8)]
#[allow(dead_code)]
pub enum I2CMstStatusBits {
    I2CPeriph0Nack = 0x01 << 0,
    I2CPeriph1Nack = 0x01 << 1,
    I2CPeriph2Nack = 0x01 << 2,
    I2CPeriph3Nack = 0x01 << 3,
    I2CPeriph4Nack = 0x01 << 4,
    I2CLostArb = 0x01 << 5,
    I2CPeriph4Done = 0x01 << 6,
    PassThrough = 0x01 << 7,
}

#[repr(u8)]
#[derive(Copy, Clone)]
pub enum MagMode {
    PowerDown = 0x00,
    Single = 0x01 << 0,
    Continuous10Hz = 0x01 << 1,
    Continuous20Hz = 0x02 << 1,
    Continuous50Hz = 0x03 << 1,
    Continuous100Hz = 0x04 << 1,
    SelfTest = 0x01 << 4,
}

// Gyro full scale range in degrees per second
#[repr(u8)]
#[derive(Copy, Clone)]
pub enum GyroFullScaleSel {
    Dps250 = 0x00,
    Dps500 = 0x01,
    Dps1000 = 0x02,
    Dps2000 = 0x03,
}

// Gyro digital low pass filter config
// Format is dAbwB_nXbwY - A is integer part of 3db BW, B is fraction. X is integer part of nyquist bandwidth, Y is fraction
#[repr(u8)]
#[derive(Copy, Clone)]
pub enum GyroDLPFSel {
    D196bw6N229bw8 = 0x00,
    D151bw8N187bw6 = 0x01,
    D119bw5N154bw3 = 0x02,
    D51bw2N73bw3 = 0x03,
    D23bw9N35bw9 = 0x04,
    D11bw6N17bw8 = 0x05,
    D5bw7N8bw9 = 0x06,
    D361bw4N376bw5 = 0x07,
    Disable = 0xFF,
}

// Accel full scale range in G's (plus or minus)
#[repr(u8)]
#[derive(Copy, Clone)]
pub enum AccelFullScaleSel {
    Gpm2 = 0x00,
    Gpm4 = 0x01,
    Gpm8 = 0x02,
    Gpm16 = 0x03,
}

// Accel digital low pass filter config
// Format is dAbwB_nXbwZ - A is integer part of 3db BW, B is fraction. X is integer part of nyquist bandwidth, Y is fraction
#[repr(u8)]
#[derive(Copy, Clone)]
pub enum AccelDLPFSel {
    D246bwN265bw = 0x00,
    D246bwN265bw1 = 0x01,
    D111bw4N136bw = 0x02,
    D50bw4N68bw8 = 0x03,
    D23bw9N34bw4 = 0x04,
    D11bw5N17bw = 0x05,
    D5bw7N8bw3 = 0x06,
    D473bwN499bw = 0x07,
    Disable = 0xFF,
}

// Constants
pub const ICM_20948_WHO_AM_I: u8 = 0xEA;
pub const MAG_I2C_ADDR: u8 = 0x0C;
pub const MAG_WHO_AM_I: u16 = 0x4809;
pub const MAX_MAGNETOMETER_STARTS: u32 = 10;
pub const RAW_DATA_NUM_BYTES: usize = 23;

pub const ACCEL_SENSITIVITY_SCALE_GPM2: f32 = 16.384;
pub const ACCEL_SENSITIVITY_SCALE_GPM4: f32 = 8.192;
pub const ACCEL_SENSITIVITY_SCALE_GPM8: f32 = 4.096;
pub const ACCEL_SENSITIVITY_SCALE_GPM16: f32 = 2.048;

pub const GYRO_SENSITIVITY_SCALE_DPS250: f32 = 131.0;
pub const GYRO_SENSITIVITY_SCALE_DPS500: f32 = 65.5;
pub const GYRO_SENSITIVITY_SCALE_DPS1000: f32 = 32.8;
pub const GYRO_SENSITIVITY_SCALE_DPS2000: f32 = 16.4;

pub const MAG_SCALE: f32 = 0.15;
