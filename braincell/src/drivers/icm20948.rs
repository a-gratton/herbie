// ICM-20948 IMU driver

use stm32f4xx_hal::{
    prelude::{_embedded_hal_blocking_spi_Write, _embedded_hal_blocking_spi_Transfer},
    gpio::{Pin, Output, PushPull},
};

// Registers
#[repr(u8)]
enum RegAddr {
    RegBankSel = 0x7F,
    AGB0RegWhoAmI = 0x00,
}

// Constants
const WHO_AM_I: u8 = 0xEA;

// Error codes
pub enum ErrorCode {
    ParamError,
    SpiError,
    WrongID,
}

pub struct ICM20948<'a, SPI, const P: char, const N: u8> {
    spi_handle: &'a mut SPI,
    cs: &'a mut Pin<P, N, Output<PushPull>>,
}

impl<'a, SPI, const P: char, const N: u8> ICM20948<'a, SPI, P, N>
where
    SPI: _embedded_hal_blocking_spi_Write<u8> + _embedded_hal_blocking_spi_Transfer<u8>,
{
    pub fn new(spi_handle: &'a mut SPI, cs: &'a mut Pin<P, N, Output<PushPull>>) -> Self {
        Self {
            spi_handle,
            cs,
        }
    }

    pub fn init(&mut self) -> Result<(), ErrorCode> {
        self.check_id()?;
        Ok(())
    }

    fn check_id(&mut self) -> Result<(), ErrorCode> {
        self.set_bank(0)?;
        let mut bytes: [u8; 2] = [RegAddr::AGB0RegWhoAmI as u8, 0];
        let bytes = &mut bytes[..];
        self.read(bytes)?;
        if bytes[1] != WHO_AM_I {
            Err(ErrorCode::WrongID)
        } else {
            Ok(())
        }
    }

    fn set_bank(&mut self, bank: u8) -> Result<(), ErrorCode> {
        if bank > 3 {
            Err(ErrorCode::ParamError)
        } else {
            let bytes: [u8; 2] = [RegAddr::RegBankSel as u8, bank];
            self.write(&bytes)
        }
    }

    fn write(&mut self, data: &[u8]) -> Result<(), ErrorCode> {
        self.cs.set_low();
        let res = self.spi_handle.write(data);
        self.cs.set_high();
        match res {
            Ok(_) => Ok(()),
            Err(_) => Err(ErrorCode::SpiError),
        }
    }

    fn read(&mut self, data: &mut [u8]) -> Result<(), ErrorCode> {
        self.cs.set_low();
        let res = self.spi_handle.transfer(data);
        self.cs.set_high();
        match res {
            Ok(_) => Ok(()),
            Err(_) => Err(ErrorCode::SpiError),
        }
    }
}
