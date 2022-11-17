#![no_main]
#![no_std]

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use braincell::drivers::imu::icm20948;
    use braincell::filtering::ahrs::{
        ahrs_filter::AHRSFilter, ahrs_filter::ImuData, madgwick, mahony,
    };
    use braincell::filtering::sma;
    use core::f32::consts::PI;
    use core::fmt::Write;
    use cortex_m::asm;
    use panic_write::PanicHandler;
    use stm32f4xx_hal::{
        gpio::{Alternate, Output, Pin, PushPull, PA9, PC1, PC2},
        pac::{SPI2, USART2},
        prelude::*,
        serial::{Config, Serial, Tx},
        spi::{Mode, Phase, Polarity, Spi},
    };
    use systick_monotonic::{ExtU64, Systick};

    const DEG_TO_RAD: f32 = PI / 180.0;
    const SMA_FILTER_SIZE: usize = 5;
    const IMU_GYRO_BIAS: (f32, f32, f32) = (-0.259, 0.29839727, 0.378);

    #[shared]
    struct Shared {
        tx: core::pin::Pin<panic_write::PanicHandler<Tx<USART2>>>,
    }

    #[local]
    struct Local {
        imu: icm20948::ICM20948<
            Spi<SPI2, (PA9<Alternate<5>>, PC2<Alternate<5>>, PC1<Alternate<7>>)>,
            Pin<'B', 12, Output<PushPull>>,
        >,
        madgwick: madgwick::MadgwickFilter,
        mahony: mahony::MahonyFilter,
        accel_x: sma::SmaFilter<f32, SMA_FILTER_SIZE>,
        accel_y: sma::SmaFilter<f32, SMA_FILTER_SIZE>,
        accel_z: sma::SmaFilter<f32, SMA_FILTER_SIZE>,
        gyro_x: sma::SmaFilter<f32, SMA_FILTER_SIZE>,
        gyro_y: sma::SmaFilter<f32, SMA_FILTER_SIZE>,
        gyro_z: sma::SmaFilter<f32, SMA_FILTER_SIZE>,
        mag_x: sma::SmaFilter<f32, SMA_FILTER_SIZE>,
        mag_y: sma::SmaFilter<f32, SMA_FILTER_SIZE>,
        mag_z: sma::SmaFilter<f32, SMA_FILTER_SIZE>,
        prev_ticks: u64,
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
        let gpioc = ctx.device.GPIOC.split();

        // Set up uart tx
        let tx_pin = gpioa.pa2.into_alternate();
        let serial = Serial::tx(
            ctx.device.USART2,
            tx_pin,
            Config::default()
                .baudrate(115200.bps())
                .wordlength_8()
                .parity_none(),
            &clocks,
        )
        .unwrap();
        let mut tx = PanicHandler::new(serial);

        // Set up imu spi
        let imu_sclk = gpioa.pa9.into_alternate();
        let imu_mosi = gpioc.pc1.into_alternate();
        let imu_miso = gpioc.pc2.into_alternate();
        let imu_spi = Spi::new(
            ctx.device.SPI2,
            (imu_sclk, imu_miso, imu_mosi),
            Mode {
                polarity: Polarity::IdleLow,
                phase: Phase::CaptureOnFirstTransition,
            },
            1.MHz(),
            &clocks,
        );

        // Set up imu cs
        let imu_cs = gpiob.pb12.into_push_pull_output();

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
        let accel_x = sma::SmaFilter::<f32, SMA_FILTER_SIZE>::new();
        let accel_y = sma::SmaFilter::<f32, SMA_FILTER_SIZE>::new();
        let accel_z = sma::SmaFilter::<f32, SMA_FILTER_SIZE>::new();
        let gyro_x = sma::SmaFilter::<f32, SMA_FILTER_SIZE>::new();
        let gyro_y = sma::SmaFilter::<f32, SMA_FILTER_SIZE>::new();
        let gyro_z = sma::SmaFilter::<f32, SMA_FILTER_SIZE>::new();
        let mag_x = sma::SmaFilter::<f32, SMA_FILTER_SIZE>::new();
        let mag_y = sma::SmaFilter::<f32, SMA_FILTER_SIZE>::new();
        let mag_z = sma::SmaFilter::<f32, SMA_FILTER_SIZE>::new();
        let madgwick = madgwick::MadgwickFilter::new(IMU_GYRO_BIAS.2);
        let mahony = mahony::MahonyFilter::new(mahony::DEFAULT_KP, mahony::DEFAULT_KI, true);

        let prev_ticks = monotonics::now().ticks();

        // Schedule the imu polling task
        // Note: it is necessary to spawn the imu polling task after some delay so the initial madgwick filter
        // update can operate on non-zero imu measurement values
        imu_poll::spawn_after(ExtU64::millis(100)).unwrap();

        (
            Shared { tx },
            Local {
                imu,
                madgwick,
                mahony,
                accel_x,
                accel_y,
                accel_z,
                gyro_x,
                gyro_y,
                gyro_z,
                mag_x,
                mag_y,
                mag_z,
                prev_ticks,
            },
            init::Monotonics(mono),
        )
    }

    #[task(local = [imu, madgwick, mahony, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, prev_ticks], shared = [tx])]
    fn imu_poll(mut cx: imu_poll::Context) {
        let task_start_ticks = monotonics::now().ticks();

        let imu = cx.local.imu;
        let accel_x = cx.local.accel_x;
        let accel_y = cx.local.accel_y;
        let accel_z = cx.local.accel_z;
        let gyro_x = cx.local.gyro_x;
        let gyro_y = cx.local.gyro_y;
        let gyro_z = cx.local.gyro_z;
        let mag_x = cx.local.mag_x;
        let mag_y = cx.local.mag_y;
        let mag_z = cx.local.mag_z;
        let madgwick = cx.local.madgwick;
        let mahony = cx.local.mahony;

        match imu.data_ready() {
            Ok(data_ready) => {
                if data_ready {
                    let _ = imu.read_data();
                    accel_x.insert(imu.get_accel_x());
                    accel_y.insert(imu.get_accel_y());
                    accel_z.insert(imu.get_accel_z());
                    gyro_x.insert(imu.get_gyro_x());
                    gyro_y.insert(imu.get_gyro_y());
                    gyro_z.insert(imu.get_gyro_z());
                    mag_x.insert(imu.get_mag_x());
                    mag_y.insert(imu.get_mag_y());
                    mag_z.insert(imu.get_mag_z());

                    if let Some(_) = accel_x.filtered() {
                        let ax = accel_x.filtered().unwrap_or(1.0);
                        let ay = accel_y.filtered().unwrap_or(1.0);
                        let az = accel_z.filtered().unwrap_or(1.0);
                        let gx = gyro_x.filtered().unwrap_or(1.0);
                        let gy = gyro_y.filtered().unwrap_or(1.0);
                        let gz = gyro_z.filtered().unwrap_or(1.0);
                        let mx = mag_x.filtered().unwrap_or(1.0);
                        let my = mag_y.filtered().unwrap_or(1.0);
                        let mz = mag_z.filtered().unwrap_or(1.0);

                        let deltat: f32 = (task_start_ticks - *cx.local.prev_ticks) as f32 / 1000.0;

                        madgwick.update(
                            ImuData {
                                accel: (ax, ay, az),
                                gyro: (
                                    (gx - IMU_GYRO_BIAS.0) * DEG_TO_RAD,
                                    (gy - IMU_GYRO_BIAS.1) * DEG_TO_RAD,
                                    (gz - IMU_GYRO_BIAS.2) * DEG_TO_RAD,
                                ),
                                mag: (mx, my, mz),
                            },
                            deltat,
                        );

                        mahony.update(
                            ImuData {
                                accel: (ax, ay, az),
                                gyro: (
                                    (gx - IMU_GYRO_BIAS.0) * DEG_TO_RAD,
                                    (gy - IMU_GYRO_BIAS.1) * DEG_TO_RAD,
                                    (gz - IMU_GYRO_BIAS.2) * DEG_TO_RAD,
                                ),
                                mag: (mx, my, mz),
                            },
                            deltat,
                        );

                        // let angles = madgwick.get_euler_angles();
                        let angles = mahony.get_euler_angles();

                        cx.shared.tx.lock(|tx_locked| {
                            writeln!(
                                tx_locked,
                                "Roll: {} deg, Pitch: {} deg, Yaw: {} deg",
                                angles.0, angles.1, angles.2
                            )
                            .unwrap()
                        });
                    }
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
        *cx.local.prev_ticks = task_start_ticks;

        // Run at 100Hz
        imu_poll::spawn_after(ExtU64::millis(10)).unwrap();
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }
}
