#![no_main]
#![no_std]

// Halt on panic
use panic_halt as _; // panic handler

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI2])]
mod app {
    use braincell::drivers::imu::icm20948;
    use braincell::filtering::ahrs::{
        ahrs_filter::AHRSFilter, ahrs_filter::ImuData, madgwick, mahony,
    };
    use core::f32::consts::PI;
    use core::fmt::Write;
    use cortex_m::asm;
    use stm32f4xx_hal::{
        gpio::{Alternate, Output, Pin, PushPull, PB3, PB4, PB5},
        pac::{SPI1, USART2},
        prelude::*,
        serial::{Config, Serial, Tx},
        spi::{Mode, Phase, Polarity, Spi},
    };
    use systick_monotonic::ExtU64;
    use systick_monotonic::Systick;

    const DEG_TO_RAD: f32 = PI / 180.0;
    const IMU_GYRO_BIAS: (f32, f32, f32) = (0.319, 0.034, 0.21);

    #[shared]
    struct Shared {
        tx: Tx<USART2>,
    }

    #[local]
    struct Local {
        imu: icm20948::ICM20948<
            Spi<SPI1, (PB3<Alternate<5>>, PB4<Alternate<5>>, PB5<Alternate<5>>)>,
            Pin<'A', 4, Output<PushPull>>,
        >,
        madgwick_filter: madgwick::MadgwickFilter,
        mahony_filter: mahony::MahonyFilter,
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
                    icm20948::ErrorCode::CSError => writeln!(tx, "CS Error").unwrap(),
                }
                panic!();
            }
        }

        // Set up imu filters
        let madgwick_filter = madgwick::MadgwickFilter::new(madgwick::DEFAULT_BETA);
        let mahony_filter = mahony::MahonyFilter::new(mahony::DEFAULT_KP, mahony::DEFAULT_KI);

        // Schedule the imu polling task
        // Note: it is necessary to spawn the imu polling task after some delay so the initial madgwick filter
        // update can operate on non-zero imu measurement values
        imu_poll::spawn_after(ExtU64::millis(100)).unwrap();

        (
            Shared { tx },
            Local {
                imu,
                madgwick_filter,
                mahony_filter,
            },
            init::Monotonics(mono),
        )
    }

    #[task(local = [imu, madgwick_filter, mahony_filter], shared = [tx])]
    fn imu_poll(mut cx: imu_poll::Context) {
        let imu = cx.local.imu;
        let madgwick_filter = cx.local.madgwick_filter;
        let mahony_filter = cx.local.mahony_filter;
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

                    madgwick_filter.update(
                        ImuData {
                            accel: (accel_x, accel_y, accel_z),
                            gyro: (
                                (gyro_x - IMU_GYRO_BIAS.0) * DEG_TO_RAD,
                                (gyro_y - IMU_GYRO_BIAS.1) * DEG_TO_RAD,
                                (gyro_z - IMU_GYRO_BIAS.2) * DEG_TO_RAD,
                            ),
                            mag: (mag_x, mag_y, mag_z),
                        },
                        0.01,
                    );

                    mahony_filter.update(
                        ImuData {
                            accel: (accel_x, accel_y, accel_z),
                            gyro: (
                                (gyro_x - IMU_GYRO_BIAS.0) * DEG_TO_RAD,
                                (gyro_y - IMU_GYRO_BIAS.1) * DEG_TO_RAD,
                                (gyro_z - IMU_GYRO_BIAS.2) * DEG_TO_RAD,
                            ),
                            mag: (mag_x, mag_y, mag_z),
                        },
                        0.01,
                    );

                    // let angles = madgwick_filter.get_euler_angles();
                    let angles = mahony_filter.get_euler_angles();

                    cx.shared.tx.lock(|tx_locked| {
                        writeln!(
                            tx_locked,
                            "Roll: {} deg, Pitch: {} deg, Yaw: {} deg",
                            angles.0, angles.1, angles.2
                        )
                        .unwrap()
                    });
                } else {
                    cx.shared
                        .tx
                        .lock(|tx_locked| writeln!(tx_locked, "IMU data not ready").unwrap());
                }
            }
            Err(_) => {
                cx.shared
                    .tx
                    .lock(|tx_locked| writeln!(tx_locked, "IMU error").unwrap());
            }
        }
        // Run at 100Hz
        imu_poll::spawn_at(task_start + ExtU64::millis(10)).unwrap();
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }
}
