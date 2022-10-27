// ICM-20948 IMU driver
// Datasheet: https://invensense.tdk.com/wp-content/uploads/2021/10/DS-000189-ICM-20948-v1.5.pdf
// Ported from this Arduino driver: https://github.com/sparkfun/SparkFun_ICM-20948_ArduinoLibrary

pub use crate::drivers::imu::icm20948_constants::*;

use cortex_m::asm;
use stm32f4xx_hal::prelude::{
    _embedded_hal_blocking_spi_Transfer, _embedded_hal_blocking_spi_Write,
};

// Error codes
pub enum ErrorCode {
    ParamError,
    SpiError,
    WrongID,
    MagError,
    MagWrongID,
    CSError,
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

struct SpiWrapper<SPI, CS> {
    spi_bus: SPI,
    cs: CS,
}

impl<SPI, CS> SpiWrapper<SPI, CS>
where
    SPI: _embedded_hal_blocking_spi_Write<u8> + _embedded_hal_blocking_spi_Transfer<u8>,
    CS: embedded_hal::digital::v2::OutputPin,
{
    fn transfer<'a>(&mut self, data: &'a mut [u8]) -> Result<&'a [u8], ErrorCode> {
        if let Err(_) = self.cs.set_low() {
            return Err(ErrorCode::CSError);
        }
        let res = self.spi_bus.transfer(data);
        if let Err(_) = self.cs.set_high() {
            return Err(ErrorCode::CSError);
        }
        match res {
            Ok(read_data) => Ok(&read_data[1..]),
            Err(_) => Err(ErrorCode::SpiError),
        }
    }
}

pub struct ICM20948<SPI, CS> {
    spi: SpiWrapper<SPI, CS>,
    curr_bank: u8,
    raw_agm: AGMData,
    fss_config: FssConfig,
}

impl<SPI, CS> ICM20948<SPI, CS>
where
    SPI: _embedded_hal_blocking_spi_Write<u8> + _embedded_hal_blocking_spi_Transfer<u8>,
    CS: embedded_hal::digital::v2::OutputPin,
{
    pub fn new(spi_bus: SPI, cs: CS) -> Self {
        Self {
            spi: SpiWrapper { spi_bus, cs },
            curr_bank: 255,
            raw_agm: AGMData::default(),
            fss_config: FssConfig {
                accel: AccelFullScaleSel::Gpm2,
                gyro: GyroFullScaleSel::Dps250,
            },
        }
    }

    pub fn init(
        &mut self,
        accel_fss_config: AccelFullScaleSel,
        accel_dlpf_config: AccelDLPFSel,
        gyro_fss_config: GyroFullScaleSel,
        gyro_dlpf_config: GyroDLPFSel,
        mag_mode: MagMode,
    ) -> Result<(), ErrorCode> {
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
                }
            }
        }
        if attempts == MAX_MAGNETOMETER_STARTS {
            return Err(mag_error);
        }

        self.config_accel(accel_fss_config, accel_dlpf_config)?;
        self.config_gyro(gyro_fss_config, gyro_dlpf_config)?;
        self.init_interrupt()?;
        Ok(())
    }

    pub fn data_ready(&mut self) -> Result<bool, ErrorCode> {
        self.set_bank(0)?;
        let data_ready = self.read_byte(RegAddrBank0::IntStatus1 as u8)?;
        Ok((data_ready & 0x01) == 1)
    }

    pub fn clear_interrupts(&mut self) -> Result<(), ErrorCode> {
        self.set_bank(0)?;
        let _ = self.read_byte(RegAddrBank0::IntStatus as u8)?;
        let _ = self.read_byte(RegAddrBank0::IntStatus1 as u8)?;
        Ok(())
    }

    pub fn read_data(&mut self) -> Result<(), ErrorCode> {
        self.set_bank(0)?;
        let mut bytes: [u8; RAW_DATA_NUM_BYTES + 1] = [0; RAW_DATA_NUM_BYTES + 1];
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

    // x-component of linear acceleration in milli G's
    pub fn get_accel_x(&mut self) -> f32 {
        self.get_accel_mg(self.raw_agm.accel.x)
    }

    // y-component of linear acceleration in milli G's
    pub fn get_accel_y(&mut self) -> f32 {
        self.get_accel_mg(self.raw_agm.accel.y)
    }

    // z-component of linear acceleration in milli G's
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
            AccelFullScaleSel::Gpm2 => (raw as f32) / ACCEL_SENSITIVITY_SCALE_GPM2,
            AccelFullScaleSel::Gpm4 => (raw as f32) / ACCEL_SENSITIVITY_SCALE_GPM4,
            AccelFullScaleSel::Gpm8 => (raw as f32) / ACCEL_SENSITIVITY_SCALE_GPM8,
            AccelFullScaleSel::Gpm16 => (raw as f32) / ACCEL_SENSITIVITY_SCALE_GPM16,
        }
    }

    fn get_gyro_dps(&mut self, raw: i16) -> f32 {
        match self.fss_config.gyro {
            GyroFullScaleSel::Dps250 => (raw as f32) / GYRO_SENSITIVITY_SCALE_DPS250,
            GyroFullScaleSel::Dps500 => (raw as f32) / GYRO_SENSITIVITY_SCALE_DPS500,
            GyroFullScaleSel::Dps1000 => (raw as f32) / GYRO_SENSITIVITY_SCALE_DPS1000,
            GyroFullScaleSel::Dps2000 => (raw as f32) / GYRO_SENSITIVITY_SCALE_DPS2000,
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

        // Set DEVICE_RESET bit
        reg |= PwrMgmt1Bits::DeviceReset as u8;

        self.write_byte(RegAddrBank0::PwrMgmt1 as u8, reg)?;
        Ok(())
    }

    fn sleep(&mut self, sleep: bool) -> Result<(), ErrorCode> {
        self.set_bank(0)?;
        let mut reg = self.read_byte(RegAddrBank0::PwrMgmt1 as u8)?;

        // Set SLEEP bit
        if sleep {
            reg |= PwrMgmt1Bits::Sleep as u8;
        } else {
            reg &= !(PwrMgmt1Bits::Sleep as u8);
        }

        self.write_byte(RegAddrBank0::PwrMgmt1 as u8, reg)?;
        Ok(())
    }

    fn set_low_power(&mut self, enable: bool) -> Result<(), ErrorCode> {
        self.set_bank(0)?;
        let mut reg = self.read_byte(RegAddrBank0::PwrMgmt1 as u8)?;

        // Set LP_EN bit
        if enable {
            reg |= PwrMgmt1Bits::LPEnable as u8;
        } else {
            reg &= !(PwrMgmt1Bits::LPEnable as u8);
        }

        self.write_byte(RegAddrBank0::PwrMgmt1 as u8, reg)?;
        Ok(())
    }

    fn config_accel(
        &mut self,
        fss_config: AccelFullScaleSel,
        dlpf_config: AccelDLPFSel,
    ) -> Result<(), ErrorCode> {
        self.set_bank(2)?;
        let mut config = self.read_byte(RegAddrBank2::AccelConfig as u8)?;

        // Configure DLPF
        if matches!(dlpf_config, AccelDLPFSel::Disable) {
            config &= !(AccelConfigBits::AccelFChoice as u8);
        } else {
            config &= !(AccelConfigBits::AccelDLPFCFG as u8);
            config |= ((dlpf_config as u8) << 3) | (AccelConfigBits::AccelFChoice as u8);
        }

        // Configure FS
        config &= !(AccelConfigBits::AccelFSSel as u8);
        config |= (fss_config as u8) << 1;

        self.write_byte(RegAddrBank2::AccelConfig as u8, config)?;
        self.fss_config.accel = fss_config;
        Ok(())
    }

    fn config_gyro(
        &mut self,
        fss_config: GyroFullScaleSel,
        dlpf_config: GyroDLPFSel,
    ) -> Result<(), ErrorCode> {
        self.set_bank(2)?;
        let mut config = self.read_byte(RegAddrBank2::GyroConfig1 as u8)?;

        // Configure DLPF
        if matches!(dlpf_config, GyroDLPFSel::Disable) {
            config &= !(GyroConfig1Bits::GyroFChoice as u8);
        } else {
            config &= !(GyroConfig1Bits::GyroDLPFCFG as u8);
            config |= ((dlpf_config as u8) << 3) | (GyroConfig1Bits::GyroFChoice as u8);
        }

        // Configure FS
        config &= !(GyroConfig1Bits::GyroFSSel as u8);
        config |= (fss_config as u8) << 1;

        self.write_byte(RegAddrBank2::GyroConfig1 as u8, config)?;
        self.fss_config.gyro = fss_config;
        Ok(())
    }

    fn init_interrupt(&mut self) -> Result<(), ErrorCode> {
        self.set_bank(0)?;
        let mut config = self.read_byte(RegAddrBank0::IntPinConfig as u8)?;

        config |= IntPinConfigBits::Int1ActiveLow as u8; // Configure active low by setting INT1_ACTL bit to 1
        config |= IntPinConfigBits::Int1Open as u8; // Configure open drain by setting INT1_OPEN bit to 1
        config &= !(IntPinConfigBits::Int1LatchEnable as u8); // Configure interrupt pulse width to 50us by setting INT1_LATCH_EN bit to 0
        config |= IntPinConfigBits::IntAnyReadToClear as u8; // Configure clear interrupt by any read operation by setting INT_ANYRD_2CLEAR bit to 1

        self.write_byte(RegAddrBank0::IntPinConfig as u8, config)?;

        let mut int_enable = self.read_byte(RegAddrBank0::IntEnable1 as u8)?;

        // Set RAW_DATA_0_RDY_EN bit
        int_enable |= IntEnable1Bits::RawData0Ready as u8;

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

        let cntl: u8 = (Periph0CtrlBits::Enable as u8) | 9; // Enable + length of 9 bytes
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

        // Set BYPASS_EN bit
        if passthrough {
            reg |= IntPinConfigBits::ByPassEnable as u8;
        } else {
            reg &= !(IntPinConfigBits::ByPassEnable as u8);
        }

        self.write_byte(RegAddrBank0::IntPinConfig as u8, reg)?;
        Ok(())
    }

    fn i2c_master_enable(&mut self, enable: bool) -> Result<(), ErrorCode> {
        self.i2c_master_passthrough(false)?; // Do not connect the SDA/SCL pins to AUX_DA/AUX_CL
        self.set_bank(3)?;
        let mut reg = self.read_byte(RegAddrBank3::I2CMstCtrl as u8)?;

        // Set clk to  345.6 kHz
        reg |= 0x07;

        // Set I2C_MST_P_NSR bit
        reg |= I2CMstCtrlBits::I2CMstPNsr as u8;

        self.write_byte(RegAddrBank3::I2CMstCtrl as u8, reg)?;

        self.set_bank(0)?;
        let mut reg = self.read_byte(RegAddrBank0::UserCtrl as u8)?;

        // Set I2C_MST_EN bit
        if enable {
            reg |= UserCtrlBits::I2CMstEnable as u8;
        } else {
            reg &= !(UserCtrlBits::I2CMstEnable as u8);
        }

        self.write_byte(RegAddrBank0::UserCtrl as u8, reg)?;
        Ok(())
    }

    fn i2c_master_reset(&mut self) -> Result<(), ErrorCode> {
        self.set_bank(0)?;
        let mut reg = self.read_byte(RegAddrBank0::UserCtrl as u8)?;

        // Set I2C_MST_RST bit
        reg |= UserCtrlBits::I2CMstRst as u8;

        self.write_byte(RegAddrBank0::UserCtrl as u8, reg)?;
        Ok(())
    }

    fn write_mag(&mut self, reg: u8, data: u8) -> Result<(), ErrorCode> {
        self.set_bank(3)?;
        self.write_byte(RegAddrBank3::I2CPeriph4Addr as u8, MAG_I2C_ADDR)?;
        self.write_byte(RegAddrBank3::I2CPeriph4Reg as u8, reg)?;

        let ctrl: u8 = Periph4CtrlBits::Enable as u8;

        self.write_byte(RegAddrBank3::I2CPeriph4DO as u8, data)?;
        self.write_byte(RegAddrBank3::I2CPeriph4Ctrl as u8, ctrl)?;

        let mut peripheral_done: bool = false;
        let mut peripheral_nack: bool = false;
        // TODO: replace count with timeout
        let mut count = 0; // max count 1000
        while !peripheral_done && count < 1000 {
            self.set_bank(0)?;
            let i2c_mst_status = self.read_byte(RegAddrBank0::I2CMstStatus as u8)?;

            peripheral_done = (i2c_mst_status & (I2CMstStatusBits::I2CPeriph4Done as u8)) != 0;
            peripheral_nack = (i2c_mst_status & (I2CMstStatusBits::I2CPeriph4Nack as u8)) != 0;
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

        let ctrl: u8 = Periph4CtrlBits::Enable as u8;
        self.write_byte(RegAddrBank3::I2CPeriph4Ctrl as u8, ctrl)?;

        let mut peripheral_done: bool = false;
        let mut peripheral_nack: bool = false;
        // TODO: replace count with timeout
        let mut count = 0; // max count 1000
        while !peripheral_done && count < 1000 {
            self.set_bank(0)?;
            let i2c_mst_status = self.read_byte(RegAddrBank0::I2CMstStatus as u8)?;

            peripheral_done = (i2c_mst_status & (I2CMstStatusBits::I2CPeriph4Done as u8)) != 0;
            peripheral_nack = (i2c_mst_status & (I2CMstStatusBits::I2CPeriph4Nack as u8)) != 0;
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
        let mut bytes = [reg, data];
        let res = self.spi.transfer(&mut bytes[..]);
        match res {
            Ok(_) => Ok(()),
            Err(e) => Err(e),
        }
    }

    fn read_byte(&mut self, reg: u8) -> Result<u8, ErrorCode> {
        let mut bytes = [reg | 0x80, 0];
        let res = self.spi.transfer(&mut bytes[..]);
        match res {
            Ok(data) => Ok(data[0]),
            Err(e) => Err(e),
        }
    }

    fn read_bytes<'a>(&mut self, data: &'a mut [u8]) -> Result<&'a [u8], ErrorCode> {
        data[0] |= 0x80;
        return self.spi.transfer(data);
    }
}
