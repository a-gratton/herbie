#![no_main]
#![no_std]

// Halt on panic
use panic_halt as _; // panic handler

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI2])]
mod app {
    use braincell::drivers::icm20948;
    use core::fmt::Write;
    use cortex_m::asm;
    use stm32f4xx_hal::{
        gpio::{Alternate, PB3, PB4, PB5},
        pac::{SPI1, USART2},
        prelude::*,
        serial::{Config, Serial, Tx},
        spi::{Mode, Phase, Polarity, Spi},
    };
    use systick_monotonic::ExtU64;
    use systick_monotonic::Systick;

    #[shared]
    struct Shared {
        tx: Tx<USART2>,
    }

    #[local]
    struct Local {
        imu: icm20948::ICM20948<
            Spi<SPI1, (PB3<Alternate<5>>, PB4<Alternate<5>>, PB5<Alternate<5>>)>,
            'A',
            4,
        >,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let rcc = ctx.device.RCC.constrain();
        let mono = Systick::new(ctx.core.SYST, 48_000_000);

        let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

        let gpioa = ctx.device.GPIOA.split();
        let gpiob = ctx.device.GPIOB.split();

        // Set up uart tx
        let tx_pin = gpioa.pa2.into_alternate();
        let mut tx = Serial::tx(
            ctx.device.USART2,
            tx_pin,
            Config::default()
                .baudrate(115200.bps())
                .wordlength_8()
                .parity_none(),
            &clocks,
        )
        .unwrap();

        // Set up imu spi
        let imu_sclk = gpiob.pb3.into_alternate();
        let imu_mosi = gpiob.pb5.into_alternate();
        let imu_miso = gpiob.pb4.into_alternate();
        let imu_spi = Spi::new(
            ctx.device.SPI1,
            (imu_sclk, imu_miso, imu_mosi),
            Mode {
                polarity: Polarity::IdleLow,
                phase: Phase::CaptureOnFirstTransition,
            },
            1.MHz(),
            &clocks,
        );

        // Set up imu cs
        let imu_cs = gpioa.pa4.into_push_pull_output();

        // Set up ICM-20948 imu
        writeln!(tx, "Initializing IMU...").unwrap();

        let mut imu = icm20948::ICM20948::new(imu_spi, imu_cs);
        match imu.init(
            icm20948::AccelFullScaleSel::Gpm2,
            icm20948::AccelDLPFSel::Disable,
            icm20948::GyroFullScaleSel::Dps250,
            icm20948::GyroDLPFSel::Disable,
            icm20948::MagMode::Continuous100Hz,
        ) {
            Ok(_) => writeln!(tx, "IMU initialized").unwrap(),
            Err(e) => {
                match e {
                    icm20948::ErrorCode::ParamError => writeln!(tx, "Param Error").unwrap(),
                    icm20948::ErrorCode::SpiError => writeln!(tx, "SPI Error").unwrap(),
                    icm20948::ErrorCode::WrongID => writeln!(tx, "Wrong ID").unwrap(),
                    icm20948::ErrorCode::MagError => writeln!(tx, "Magnetometer Error").unwrap(),
                    icm20948::ErrorCode::MagWrongID => {
                        writeln!(tx, "Magnetometer wrong ID").unwrap()
                    }
                }
                panic!();
            }
        }

        // Schedule the imu polling task
        imu_poll::spawn().unwrap();

        (Shared { tx }, Local { imu }, init::Monotonics(mono))
    }

    #[task(local = [imu], shared = [tx])]
    fn imu_poll(mut cx: imu_poll::Context) {
        let imu = cx.local.imu;
        let task_start = monotonics::now();

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

                    cx.shared.tx.lock(|tx_locked| writeln!(tx_locked, "Accel (MG): [{:.2}, {:.2}, {:.2}], Gyro (dps): [{:.2}, {:.2}, {:.2}], Mag (uT): [{:.2}, {:.2}, {:.2}]", accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z).unwrap());
                }
            }
            Err(_) => cx
                .shared
                .tx
                .lock(|tx_locked| writeln!(tx_locked, "IMU error").unwrap()),
        }
        // Run at 20Hz
        imu_poll::spawn_at(task_start + ExtU64::millis(50)).unwrap();
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }
}
