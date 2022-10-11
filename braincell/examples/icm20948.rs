// ICM 20948 IMU example

#![no_main]
#![no_std]

use braincell::drivers::icm20948;

// Halt on panic
use panic_halt as _; // panic handler

use cortex_m_rt::entry;
use core::fmt::Write;
use stm32f4xx_hal as hal;

use crate::hal::{
    pac,
    prelude::*,
    serial::{Config, Serial},
    spi::{Mode, Phase, Polarity, Spi},
};

#[entry]
fn main() -> ! {
    if let (Some(dp), Some(cp)) = (
        pac::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();

        // Set up the system clock. We want to run at 48MHz for this one.
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

        // Create a delay abstraction based on SysTick
        let mut delay = cp.SYST.delay(&clocks);

        // Set up uart tx
        let tx_pin = gpioa.pa2.into_alternate();
        let mut tx = Serial::tx(
            dp.USART2,
            tx_pin,
            Config::default()
                .baudrate(115200.bps())
                .wordlength_8()
                .parity_none(),
            &clocks,
        )
        .unwrap();

        // Set up spi 1 to interface with imu
        let sclk = gpiob.pb3.into_alternate();
        let mosi = gpiob.pb5.into_alternate();
        let miso = gpiob.pb4.into_alternate();
        let mut cs = gpioa.pa4.into_push_pull_output();
        let mut spi = Spi::new(
            dp.SPI1,
            (sclk, miso, mosi),
            Mode {
                polarity: Polarity::IdleLow,
                phase: Phase::CaptureOnFirstTransition,
            },
            1.MHz(),
            &clocks,
        );

        writeln!(tx, "Initializing IMU...").unwrap();

        // icm-20948 driver
        let mut imu = icm20948::ICM20948::new(&mut spi, &mut cs);
        let res = imu.init(&mut delay, icm20948::AccelFullScaleSel::Gpm2, icm20948::AccelDLPFSel::Disable, icm20948::GyroFullScaleSel::Dps250, icm20948::GyroDLPFSel::Disable, icm20948::MagMode::Continuous100Hz);
        match res {
            Ok(_) => writeln!(tx, "IMU initialized").unwrap(),
            Err(e) => {
                match e {
                    icm20948::ErrorCode::ParamError => writeln!(tx, "Param Error").unwrap(),
                    icm20948::ErrorCode::SpiError => writeln!(tx, "SPI Error").unwrap(),
                    icm20948::ErrorCode::WrongID => writeln!(tx, "Wrong ID").unwrap(),
                    icm20948::ErrorCode::MagError => writeln!(tx, "Magnetometer Error").unwrap(),
                    icm20948::ErrorCode::MagWrongID => writeln!(tx, "Magnetometer wrong ID").unwrap(),
                }
                panic!();
            }
        }

        loop {
            match imu.data_ready() {
                Ok(data_ready) => {
                    if data_ready {
                        let _ = imu.read_data();
                        let accel_x = imu.get_accel_x();
                        let accel_y = imu.get_accel_y();
                        let accel_z = imu.get_accel_z();
                        let gyro_x = imu.get_gyro_x();
                        let gyro_y = imu.get_gyro_y();
                        let gyro_z = imu.get_gyro_z();
                        let mag_x = imu.get_mag_x();
                        let mag_y = imu.get_mag_y();
                        let mag_z = imu.get_mag_z();
                        writeln!(tx, "Accel (MG): [{:.2}, {:.2}, {:.2}], Gyro (dps): [{:.2}, {:.2}, {:.2}], Mag (uT): [{:.2}, {:.2}, {:.2}]", accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z).unwrap()
                    }
                },
                Err(_) => writeln!(tx, "error in data_ready()").unwrap(),
            }
            delay.delay_ms(100_u32);
        }
    }

    loop {}
}
