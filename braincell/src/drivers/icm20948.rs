// ICM-20948 IMU driver

use stm32f4xx_hal::{
    prelude::{_embedded_hal_blocking_spi_Write, _embedded_hal_blocking_spi_Transfer, _embedded_hal_blocking_delay_DelayMs},
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
    I2CMstStatus = 0x17,
}

#[repr(u8)]
enum RegAddrBank3 {
    I2CMstCtrl = 0x01,
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
    Cntl3 = 0x32,
}

// Constants
const ICM_20948_WHO_AM_I: u8 = 0xEA;
const MAG_I2C_ADDR: u8 = 0x0C;
const MAG_WHO_AM_I: u16 = 0x4809;
const MAX_MAGNETOMETER_STARTS: u32 = 10;

// Error codes
pub enum ErrorCode {
    ParamError,
    SpiError,
    WrongID,
    MagError,
    MagWrongID,
}

pub struct ICM20948<'a, SPI, const P: char, const N: u8> {
    spi_handle: &'a mut SPI,
    cs: &'a mut Pin<P, N, Output<PushPull>>,
    curr_bank: u8,
}

impl<'a, 'b, SPI, const P: char, const N: u8> ICM20948<'a, SPI, P, N>
where
    SPI: _embedded_hal_blocking_spi_Write<u8> + _embedded_hal_blocking_spi_Transfer<u8>,
{
    pub fn new(spi_handle: &'a mut SPI, cs: &'a mut Pin<P, N, Output<PushPull>>) -> Self {
        Self {
            spi_handle,
            cs,
            curr_bank: 255,
        }
    }

    pub fn init<DELAY, TX>(&mut self, delay: &'a mut DELAY, tx: &'a mut TX) -> Result<(), ErrorCode>
    where
        DELAY: _embedded_hal_blocking_delay_DelayMs<u32>,
        TX: core::fmt::Write,
    {
        self.check_id()?;
        self.sw_reset()?;
        delay.delay_ms(50_u32);
        self.sleep(false)?;
        self.set_low_power(false)?;
        self.init_mag(delay)?;

        // TODO: done minimal setup, add more config here if necessary
        Ok(())
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

    fn init_mag<DELAY>(&mut self, delay: &'a mut DELAY) -> Result<(), ErrorCode>
    where
        DELAY: _embedded_hal_blocking_delay_DelayMs<u32>,
    {
        self.i2c_master_enable(true)?;
        // Reset mag
        self.write_mag(delay, RegAddrMag::Cntl3 as u8, 1)?;

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
                        delay.delay_ms(10_u32);
                    }
                }
            }
        }
        if attempts == MAX_MAGNETOMETER_STARTS {
            return Err(ErrorCode::MagError);
        }

        // TODO: done minimal setup of mag, add more config here if necessary
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

    fn write_mag<DELAY>(&mut self, delay: &'b mut DELAY, reg: u8, data: u8) -> Result<(), ErrorCode> 
    where
        DELAY: _embedded_hal_blocking_delay_DelayMs<u32>,
    {
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
            delay.delay_ms(1_u32);
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
}
