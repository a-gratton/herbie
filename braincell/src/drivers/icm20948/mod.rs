// ICM-20948 IMU driver
// Datasheet: https://invensense.tdk.com/wp-content/uploads/2021/10/DS-000189-ICM-20948-v1.5.pdf
// Ported from this Arduino driver: https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary

use cortex_m::asm;
use stm32f4xx_hal::{
    prelude::{_embedded_hal_blocking_spi_Write, _embedded_hal_blocking_spi_Transfer},
    gpio::{Pin, Output, PushPull},
};

// Registers
#[repr(u8)]
enum RegAddrGeneral {
    BankSel = 0x7F,
}

#[repr(u8)]
enum RegAddrBank0 {
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
enum RegAddrBank2 {
    GyroConfig1 = 0x01,
    AccelConfig = 0x14,
}

#[repr(u8)]
enum RegAddrBank3 {
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
enum RegAddrMag {
    WIA1 = 0x00,
    WIA2 = 0x01,
    ST1 = 0x10,
    Cntl12 = 0x31,
    Cntl3 = 0x32,
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
const ICM_20948_WHO_AM_I: u8 = 0xEA;
const MAG_I2C_ADDR: u8 = 0x0C;
const MAG_WHO_AM_I: u16 = 0x4809;
const MAX_MAGNETOMETER_STARTS: u32 = 10;
const RAW_DATA_NUM_BYTES: usize = 23;
const MAG_SCALE: f32 = 0.15;

// Error codes
pub enum ErrorCode {
    ParamError,
    SpiError,
    WrongID,
    MagError,
    MagWrongID,
}

#[derive(Default)]
struct Axes {
    x: i16,
    y: i16,
    z: i16,
}

#[derive(Default)]
struct AGMData {
    accel: Axes,
    gyro: Axes,
    mag: Axes,
    mag_stat1: u8,
    mag_stat2: u8,
}

struct FssConfig {
    accel: AccelFullScaleSel,
    gyro: GyroFullScaleSel,
}

pub struct ICM20948<SPI, const P: char, const N: u8> {
    spi_handle: SPI,
    cs: Pin<P, N, Output<PushPull>>,
    curr_bank: u8,
    raw_agm: AGMData,
    fss_config: FssConfig,
}

impl<SPI, const P: char, const N: u8> ICM20948<SPI, P, N>
where
    SPI: _embedded_hal_blocking_spi_Write<u8> + _embedded_hal_blocking_spi_Transfer<u8>,
{
    pub fn new(spi_handle: SPI, cs: Pin<P, N, Output<PushPull>>) -> Self {
        Self {
            spi_handle,
            cs,
            curr_bank: 255,
            raw_agm: AGMData::default(),
            fss_config: FssConfig{accel: AccelFullScaleSel::Gpm2, gyro: GyroFullScaleSel::Dps250}
        }
    }

    pub fn init(&mut self, accel_fss_config: AccelFullScaleSel, accel_dlpf_config: AccelDLPFSel, gyro_fss_config: GyroFullScaleSel, gyro_dlpf_config: GyroDLPFSel, mag_mode: MagMode) -> Result<(), ErrorCode>
    {
        self.check_id()?;
        self.sw_reset()?;
        asm::delay(100_000);
        self.sleep(false)?;
        self.set_low_power(false)?;
        
        let mut attempts = 0;
        let mut mag_error: ErrorCode = ErrorCode::MagError;
        while attempts < MAX_MAGNETOMETER_STARTS {
            attempts += 1;
            match self.init_mag(mag_mode) {
                Ok(_) => break,
                Err(err) => {
                    mag_error = err;
                    asm::delay(100_000);
                },
            }
        }
        if attempts == MAX_MAGNETOMETER_STARTS {
            return Err(mag_error);
        }

        // TODO: set sample mode? set sample rate?
        self.config_accel(accel_fss_config, accel_dlpf_config)?;
        self.config_gyro(gyro_fss_config, gyro_dlpf_config)?;
        self.init_interrupt()?;
        Ok(())
    }

    pub fn data_ready(&mut self) -> Result<bool, ErrorCode> {
        self.set_bank(0)?;
        let data_ready = self.read_byte(RegAddrBank0::IntStatus1 as u8)?;
        let data_ready: bool = (data_ready & 0x01) == 1;
        Ok(data_ready)
    }

    pub fn clear_interrupts(&mut self) -> Result<(), ErrorCode> {
        self.set_bank(0)?;
        let _ = self.read_byte(RegAddrBank0::IntStatus as u8)?;
        let _ = self.read_byte(RegAddrBank0::IntStatus1 as u8)?;
        Ok(())
    }

    pub fn read_data(&mut self) -> Result<(), ErrorCode> {
        self.set_bank(0)?;
        let mut bytes: [u8; RAW_DATA_NUM_BYTES+1] = [0; RAW_DATA_NUM_BYTES+1];
        bytes[0] = RegAddrBank0::AccelXoutH as u8;
        let buf = self.read_bytes(&mut bytes[..])?;
        self.raw_agm.accel.x = (((buf[0] as u16) << 8) | buf[1] as u16) as i16;
        self.raw_agm.accel.y = (((buf[2] as u16) << 8) | buf[3] as u16) as i16;
        self.raw_agm.accel.z = (((buf[4] as u16) << 8) | buf[5] as u16) as i16;
        self.raw_agm.gyro.x = (((buf[6] as u16) << 8) | buf[7] as u16) as i16;
        self.raw_agm.gyro.y = (((buf[8] as u16) << 8) | buf[9] as u16) as i16;
        self.raw_agm.gyro.z = (((buf[10] as u16) << 8) | buf[11] as u16) as i16;
        self.raw_agm.mag_stat1 = buf[14];
        self.raw_agm.mag.x = (((buf[16] as u16) << 8) | buf[15] as u16) as i16;
        self.raw_agm.mag.y = (((buf[18] as u16) << 8) | buf[17] as u16) as i16;
        self.raw_agm.mag.z = (((buf[20] as u16) << 8) | buf[19] as u16) as i16;
        self.raw_agm.mag_stat2 = buf[22];
        Ok(())
    }

    // x-component of linear acceleration in Mega G's
    pub fn get_accel_x(&mut self) -> f32 {
        self.get_accel_mg(self.raw_agm.accel.x)
    }

    // y-component of linear acceleration in Mega G's
    pub fn get_accel_y(&mut self) -> f32 {
        self.get_accel_mg(self.raw_agm.accel.y)
    }

    // z-component of linear acceleration in Mega G's
    pub fn get_accel_z(&mut self) -> f32 {
        self.get_accel_mg(self.raw_agm.accel.z)
    }

    // x-component of angular velocity in degrees per second
    pub fn get_gyro_x(&mut self) -> f32 {
        self.get_gyro_dps(self.raw_agm.gyro.x)
    }

    // y-component of angular velocity in degrees per second
    pub fn get_gyro_y(&mut self) -> f32 {
        self.get_gyro_dps(self.raw_agm.gyro.y)
    }

    // z-component of angular velocity in degrees per second
    pub fn get_gyro_z(&mut self) -> f32 {
        self.get_gyro_dps(self.raw_agm.gyro.z)
    }

    // x-component of magnetic heading in micro Teslas
    pub fn get_mag_x(&mut self) -> f32 {
        self.get_mag_ut(self.raw_agm.mag.x)
    }

    // y-component of magnetic heading in micro Teslas
    pub fn get_mag_y(&mut self) -> f32 {
        self.get_mag_ut(self.raw_agm.mag.y)
    }

    // z-component of magnetic heading in micro Teslas
    pub fn get_mag_z(&mut self) -> f32 {
        self.get_mag_ut(self.raw_agm.mag.z)
    }

    fn get_accel_mg(&mut self, raw: i16) -> f32 {
        match self.fss_config.accel {
            AccelFullScaleSel::Gpm2 => (raw as f32) / 16.384,
            AccelFullScaleSel::Gpm4 => (raw as f32) / 8.192,
            AccelFullScaleSel::Gpm8 => (raw as f32) / 4.096,
            AccelFullScaleSel::Gpm16 => (raw as f32) / 2.048,
        }
    }

    fn get_gyro_dps(&mut self, raw: i16) -> f32 {
        match self.fss_config.gyro {
            GyroFullScaleSel::Dps250 => (raw as f32) / 131.0,
            GyroFullScaleSel::Dps500 => (raw as f32) / 65.5,
            GyroFullScaleSel::Dps1000 => (raw as f32) / 32.8,
            GyroFullScaleSel::Dps2000 => (raw as f32) / 16.4,
        }
    }

    fn get_mag_ut(&mut self, raw: i16) -> f32 {
        (raw as f32) * MAG_SCALE
    }

    fn check_id(&mut self) -> Result<(), ErrorCode> {
        self.set_bank(0)?;
        let whoami = self.read_byte(RegAddrBank0::WhoAmI as u8)?;
        if whoami != ICM_20948_WHO_AM_I {
            Err(ErrorCode::WrongID)
        } else {
            Ok(())
        }
    }

    fn sw_reset(&mut self) -> Result<(), ErrorCode> {
        self.set_bank(0)?;
        let mut reg = self.read_byte(RegAddrBank0::PwrMgmt1 as u8)?;

        // PwrMgmt1 register:
        // Bits:     |       7      |   6   |   5   |     4    |     3    |   2:0  |
        // Function: | DEVICE_RESET | SLEEP | LP_EN | reserved | TEMP_DIS | CLKSEL |

        // Set DEVICE_RESET bit
        reg |= 0x80;

        self.write_byte(RegAddrBank0::PwrMgmt1 as u8, reg)?;
        Ok(())
    }

    fn sleep(&mut self, sleep: bool) -> Result<(), ErrorCode> {
        self.set_bank(0)?;
        let mut reg = self.read_byte(RegAddrBank0::PwrMgmt1 as u8)?;

        // PwrMgmt1 register:
        // Bits:     |       7      |   6   |   5   |     4    |     3    |   2:0  |
        // Function: | DEVICE_RESET | SLEEP | LP_EN | reserved | TEMP_DIS | CLKSEL |

        // Set SLEEP bit
        if sleep {
            reg |= 0x40;
        } else {
            reg &= !0x40;
        }

        self.write_byte(RegAddrBank0::PwrMgmt1 as u8, reg)?;
        Ok(())
    }

    fn set_low_power(&mut self, enable: bool) ->Result<(), ErrorCode> {
        self.set_bank(0)?;
        let mut reg = self.read_byte(RegAddrBank0::PwrMgmt1 as u8)?;

        // PwrMgmt1 register:
        // Bits:     |       7      |   6   |   5   |     4    |     3    |   2:0  |
        // Function: | DEVICE_RESET | SLEEP | LP_EN | reserved | TEMP_DIS | CLKSEL |

        // Set LP_EN bit
        if enable {
            reg |= 0x20;
        } else {
            reg &= !0x20;
        }

        self.write_byte(RegAddrBank0::PwrMgmt1 as u8, reg)?;
        Ok(())
    }

    fn config_accel(&mut self, fss_config: AccelFullScaleSel, dlpf_config: AccelDLPFSel) -> Result<(), ErrorCode> {
        self.set_bank(2)?;
        let mut config = self.read_byte(RegAddrBank2::AccelConfig as u8)?;

        // AccelConfig register:
        // Bits:     |    7:6   |       5:3     |      2:1     |       0       |
        // Function: | reserved | ACCEL_DLPFCFG | ACCEL_FS_SEL | ACCEL_FCHOICE |

        // Configure DLPF
        if matches!(dlpf_config, AccelDLPFSel::Disable) {
            config &= 0b1111_1110;
        } else {
            config &= 0b1100_0111;
            config |= ((dlpf_config as u8) << 3) | 1;
        }

        // Configure FS
        config &= 0b1111_1001;
        config |= (fss_config as u8) << 1;

        self.write_byte(RegAddrBank2::AccelConfig as u8, config)?;
        self.fss_config.accel = fss_config;
        Ok(())
    }

    fn config_gyro(&mut self, fss_config: GyroFullScaleSel, dlpf_config: GyroDLPFSel) -> Result<(), ErrorCode> {
        self.set_bank(2)?;
        let mut config = self.read_byte(RegAddrBank2::GyroConfig1 as u8)?;

        // GyroConfig1 register:
        // Bits:     |    7:6   |      5:3     |     2:1     |       0      |
        // Function: | reserved | GYRO_DLPFCFG | GYRO_FS_SEL | GYRO_FCHOICE |

        // Configure DLPF
        if matches!(dlpf_config, GyroDLPFSel::Disable) {
            config &= 0b1111_1110;
        } else {
            config &= 0b1100_0111;
            config |= ((dlpf_config as u8) << 3) | 1;
        }

        // Configure FS
        config &= 0b1111_1001;
        config |= (fss_config as u8) << 1;

        self.write_byte(RegAddrBank2::GyroConfig1 as u8, config)?;
        self.fss_config.gyro = fss_config;
        Ok(())
    }

    fn init_interrupt(&mut self) -> Result<(), ErrorCode> {
        self.set_bank(0)?;
        let mut config = self.read_byte(RegAddrBank0::IntPinConfig as u8)?;

        // IntPinConfig register:
        // Bits:     |     7     |     6     |        5      |         4        |      3     |          2        |     1     |     0    |
        // Function: | INT1_ACTL | INT1_OPEN | INT1_LATCH_EN | INT_ANYRD_2CLEAR | ACTL_FSYNC | FSYNC_INT_MODE_EN | BYPASS_EN | reserved |

        config |= 0x80; // Configure active low by setting INT1_ACTL bit to 1
        config |= 0x40; // Configure open drain by setting INT1_OPEN bit to 1
        config &= !0x20; // Configure interrupt pulse width to 50us by setting INT1_LATCH_EN bit to 0
        config |= 0x10; // Configure clear interrupt by any read operation by setting INT_ANYRD_2CLEAR bit to 1

        self.write_byte(RegAddrBank0::IntPinConfig as u8, config)?;

        let mut int_enable = self.read_byte(RegAddrBank0::IntEnable1 as u8)?;

        // IntEnable1 register:
        // Bits:      |    7:1   |         0         |
        // Function:  | reserved | RAW_DATA_0_RDY_EN |

        // Set RAW_DATA_0_RDY_EN bit
        int_enable |= 0x01;

        self.write_byte(RegAddrBank0::IntEnable1 as u8, int_enable)?;
        Ok(())
    }

    fn init_mag(&mut self, mode: MagMode) -> Result<(), ErrorCode> {
        self.i2c_master_enable(true)?;
        // Reset mag
        self.write_mag(RegAddrMag::Cntl3 as u8, 1)?;

        // Reset master I2C until the mag responds
        let mut attempts = 0;
        while attempts < MAX_MAGNETOMETER_STARTS {
            attempts += 1;
            match self.mag_check_id() {
                Ok(_) => break,
                Err(err) => {
                    if matches!(err, ErrorCode::MagWrongID) {
                        return Err(ErrorCode::MagWrongID);
                    } else {
                        self.i2c_master_reset()?;
                        asm::delay(100_000);
                    }
                }
            }
        }
        if attempts == MAX_MAGNETOMETER_STARTS {
            return Err(ErrorCode::MagError);
        }

        // Configure mag mode
        self.write_mag(RegAddrMag::Cntl12 as u8, mode as u8)?;

        // Configure mag peripheral
        self.set_bank(3)?;
        let periph_addr: u8 = (1 << 7) | MAG_I2C_ADDR;
        self.write_byte(RegAddrBank3::I2CPeriph0Addr as u8, periph_addr)?; // Peripheral address
        self.write_byte(RegAddrBank3::I2CPeriph0Reg as u8, RegAddrMag::ST1 as u8)?; // Peripheral register

        // Periph0Ctrl register:
        // Bits:     |  7 |    6    |    5    |  4  |  3:0 |
        // Function: | EN | BYTE_SW | REG_DIS | GRP | LENG |

        let cntl: u8 = 0b1000_1001; // Enable + length of 9 bytes
        self.write_byte(RegAddrBank3::I2CPeriph0Ctrl as u8, cntl)?; // Peripheral control

        Ok(())
    }

    fn mag_check_id(&mut self) -> Result<(), ErrorCode> {
        let whoami1 = self.read_mag(RegAddrMag::WIA1 as u8)?;
        let whoami2 = self.read_mag(RegAddrMag::WIA2 as u8)?;
        if [whoami1, whoami2] != MAG_WHO_AM_I.to_be_bytes() {
            return Err(ErrorCode::MagWrongID);
        }
        Ok(())
    }

    fn i2c_master_passthrough(&mut self, passthrough: bool) -> Result<(), ErrorCode> {
        self.set_bank(0)?;
        let mut reg = self.read_byte(RegAddrBank0::IntPinConfig as u8)?;

        // IntPinConfig register:
        // Bits:     |     7     |     6     |        5      |         4        |      3     |          2        |     1     |     0    |
        // Function: | INT1_ACTL | INT1_OPEN | INT1_LATCH_EN | INT_ANYRD_2CLEAR | ACTL_FSYNC | FSYNC_INT_MODE_EN | BYPASS_EN | reserved |

        // Set BYPASS_EN bit
        if passthrough {
            reg |= 0x02;
        } else {
            reg &= !0x02;
        }

        self.write_byte(RegAddrBank0::IntPinConfig as u8, reg)?;
        Ok(())
    }

    fn i2c_master_enable(&mut self, enable: bool) -> Result<(), ErrorCode> {
        self.i2c_master_passthrough(false)?; // Do not connect the SDA/SCL pins to AUX_DA/AUX_CL
        self.set_bank(3)?;
        let mut reg = self.read_byte(RegAddrBank3::I2CMstCtrl as u8)?;

        // I2CMstCtrl register:
        // Bits:     |      7      |    6:5   |       4       |     3:0     |
        // Function: | MULT_MST_EN | reserved | I2C_MST_P_NSR | I2C_MST_CLK |
    
        // Set clk to  345.6 kHz
        reg |= 0x07;

        // Set I2C_MST_P_NSR bit
        reg |= 0x04;

        self.write_byte(RegAddrBank3::I2CMstCtrl as u8, reg)?;

        self.set_bank(0)?;
        let mut reg = self.read_byte(RegAddrBank0::UserCtrl as u8)?;

        // UserCtrl register:
        // Bits:     |    7   |    6    |      5     |      4     |    3    |    2     |      1      |     0    |
        // Function: | DMP_EN | FIFO_EN | I2C_MST_EN | I2C_IF_DIS | DMP_RST | SRAM_RST | I2C_MST_RST | reserved |

        // Set I2C_MST_EN bit
        if enable {
            reg |= 0x20;
        } else {
            reg &= !0x20;
        }

        self.write_byte(RegAddrBank0::UserCtrl as u8, reg)?;
        Ok(())
    }

    fn i2c_master_reset(&mut self) -> Result<(), ErrorCode> {
        self.set_bank(0)?;
        let mut reg = self.read_byte(RegAddrBank0::UserCtrl as u8)?;

        // UserCtrl register:
        // Bits:     |    7   |    6    |      5     |      4     |    3    |    2     |      1      |     0    |
        // Function: | DMP_EN | FIFO_EN | I2C_MST_EN | I2C_IF_DIS | DMP_RST | SRAM_RST | I2C_MST_RST | reserved |

        // Set I2C_MST_RST bit
        reg |= 0x02;

        self.write_byte(RegAddrBank0::UserCtrl as u8, reg)?;
        Ok(())
    }

    fn write_mag(&mut self, reg: u8, data: u8) -> Result<(), ErrorCode> {
        self.set_bank(3)?;
        self.write_byte(RegAddrBank3::I2CPeriph4Addr as u8, MAG_I2C_ADDR)?;
        self.write_byte(RegAddrBank3::I2CPeriph4Reg as u8, reg)?;

        // Periph4Ctrl register:
        // Bits:     |  7 |    6   |    5    | 4:0 |
        // Function: | EN | INT_EN | REG_DIS | DLY |

        let ctrl: u8 = 0x80;

        self.write_byte(RegAddrBank3::I2CPeriph4DO as u8, data)?;
        self.write_byte(RegAddrBank3::I2CPeriph4Ctrl as u8, ctrl)?;

        let mut peripheral_done: bool = false;
        let mut peripheral_nack: bool = false;
        // TODO: replace count with timeout
        let mut count = 0; // max count 1000
        while !peripheral_done && count < 1000 {
            self.set_bank(0)?;
            let i2c_mst_status = self.read_byte(RegAddrBank0::I2CMstStatus as u8)?;

            // I2CMstStatus register:
            // Bits:     |       7      |         6        |       5      |         4        |         3        |         2        |         1        |         0        |
            // Function: | PASS_THROUGH | I2C_PERIPH4_DONE | I2C_LOST_ARB | I2C_PERIPH4_NACK | I2C_PERIPH3_NACK | I2C_PERIPH2_NACK | I2C_PERIPH1_NACK | I2C_PERIPH0_NACK |

            peripheral_done = (i2c_mst_status & 0x40) != 0;
            peripheral_nack = (i2c_mst_status & 0x10) != 0;
            count += 1;
        }
        if !peripheral_done || peripheral_nack {
            return Err(ErrorCode::MagError);
        }

        Ok(())
    }

    fn read_mag(&mut self, reg: u8) -> Result<u8, ErrorCode> {
        self.set_bank(3)?;
        self.write_byte(RegAddrBank3::I2CPeriph4Addr as u8, MAG_I2C_ADDR | 0x80)?;
        self.write_byte(RegAddrBank3::I2CPeriph4Reg as u8, reg)?;

        // Periph4Ctrl register:
        // Bits:     |  7 |    6   |    5    | 4:0 |
        // Function: | EN | INT_EN | REG_DIS | DLY |

        let ctrl: u8 = 0x80;

        self.write_byte(RegAddrBank3::I2CPeriph4Ctrl as u8, ctrl)?;

        let mut peripheral_done: bool = false;
        let mut peripheral_nack: bool = false;
        // TODO: replace count with timeout
        let mut count = 0; // max count 1000
        while !peripheral_done && count < 1000 {
            self.set_bank(0)?;
            let i2c_mst_status = self.read_byte(RegAddrBank0::I2CMstStatus as u8)?;

            // I2CMstStatus register:
            // Bits:     |       7      |         6        |       5      |         4        |         3        |         2        |         1        |         0        |
            // Function: | PASS_THROUGH | I2C_PERIPH4_DONE | I2C_LOST_ARB | I2C_PERIPH4_NACK | I2C_PERIPH3_NACK | I2C_PERIPH2_NACK | I2C_PERIPH1_NACK | I2C_PERIPH0_NACK |

            peripheral_done = (i2c_mst_status & 0x40) != 0;
            peripheral_nack = (i2c_mst_status & 0x10) != 0;
            count += 1;
        }
        if !peripheral_done || peripheral_nack {
            return Err(ErrorCode::MagError);
        }

        self.set_bank(3)?;
        let data = self.read_byte(RegAddrBank3::I2CPeriph4DI as u8)?;

        Ok(data)
    }

    fn set_bank(&mut self, bank: u8) -> Result<(), ErrorCode> {
        if bank > 3 {
            Err(ErrorCode::ParamError)
        } else if self.curr_bank == bank {
            Ok(())
        } else {
            self.write_byte(RegAddrGeneral::BankSel as u8, (bank << 4) & 0x30)?;
            self.curr_bank = bank;
            Ok(())
        }
    }

    fn write_byte(&mut self, reg: u8, data: u8) -> Result<(), ErrorCode> {
        self.cs.set_low();
        let res = self.spi_handle.write(&[reg, data][..]);
        self.cs.set_high();
        match res {
            Ok(_) => Ok(()),
            Err(_) => Err(ErrorCode::SpiError),
        }
    }

    fn read_byte(&mut self, reg: u8) -> Result<u8, ErrorCode> {
        self.cs.set_low();
        let mut bytes = [reg | 0x80, 0];
        let res = self.spi_handle.transfer(&mut bytes[..]);
        self.cs.set_high();
        match res {
            Ok(data) => Ok(data[1]),
            Err(_) => Err(ErrorCode::SpiError),
        }
    }

    fn read_bytes<'w>(&mut self, data: &'w mut [u8]) -> Result<&'w [u8], ErrorCode> {
        data[0] |= 0x80;
        self.cs.set_low();
        let res = self.spi_handle.transfer(data);
        self.cs.set_high();
        match res {
            Ok(read_data) => Ok(&read_data[1..]),
            Err(_) => Err(ErrorCode::SpiError),
        }
    }
}
