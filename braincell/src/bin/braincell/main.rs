#![no_main]
#![no_std]

mod config;
mod filter;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI2])]
mod app {
    use crate::config::sys_config;
    use crate::filter;
    use braincell::drivers::imu::icm20948;
    use braincell::drivers::tof::vl53l1x;
    use braincell::filtering::{ahrs::mahony, sma};
    use core::fmt::Write;
    use cortex_m::asm;
    use panic_write::PanicHandler;
    use stm32f4xx_hal::{
        gpio::{Alternate, Output, Pin, PushPull, PB3, PB4, PB5, PB8, PB9},
        i2c::{I2c, Mode as i2cMode},
        pac::{I2C1, SPI1, USART2},
        prelude::*,
        serial::{Config, Serial, Tx},
        spi::{Mode, Phase, Polarity, Spi},
    };
    use systick_monotonic::{fugit::Duration, Systick};

    #[shared]
    struct Shared {
        tof_front_filter: sma::SmaFilter<u16, 10>,
        tof_left_filter: sma::SmaFilter<u16, 10>,
        imu_filter: filter::ImuFilter<{ sys_config::IMU_SMA_FILTER_SIZE }, mahony::MahonyFilter>,
    }

    #[local]
    struct Local {
        i2c: I2c<I2C1, (PB8, PB9)>,
        tx: core::pin::Pin<panic_write::PanicHandler<Tx<USART2>>>,
        tof_front: vl53l1x::VL53L1<I2C1, PB8, PB9>,
        tof_left: vl53l1x::VL53L1<I2C1, PB8, PB9>,
        imu: icm20948::ICM20948<
            Spi<SPI1, (PB3<Alternate<5>>, PB4<Alternate<5>>, PB5<Alternate<5>>)>,
            Pin<'A', 4, Output<PushPull>>,
        >,
        filter_data_prev_ticks: u64,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // configure clocks
        let rcc = ctx.device.RCC.constrain();
        let mono = Systick::new(ctx.core.SYST, 48_000_000);
        let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

        // configure I2C
        let gpiob = ctx.device.GPIOB.split();
        let scl = gpiob.pb8;
        let sda = gpiob.pb9;
        let mut i2c = I2c::new(
            ctx.device.I2C1,
            (scl, sda),
            i2cMode::Standard {
                frequency: 100.kHz(),
            },
            &clocks,
        );

        // configure IMU spi and cs
        let gpioa = ctx.device.GPIOA.split();
        let imu_cs = gpioa.pa4.into_push_pull_output();
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

        // set up uart tx
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

        // set up ToF sensors
        let tof_front: vl53l1x::VL53L1<I2C1, PB8, PB9> =
            match vl53l1x::VL53L1::new(&mut i2c, sys_config::TOF_FRONT_ADDRESS) {
                Ok(val) => val,
                Err(_) => {
                    writeln!(tx, "tof_front initialization failed\r").unwrap();
                    panic!("tof_front initialization failed");
                }
            };
        if let Err(_) = tof_front.start_ranging(
            &mut i2c,
            Some(vl53l1x::DistanceMode::Short),
            Some(vl53l1x::TimingBudget::Tb15ms),
            Some(20),
        ) {
            writeln!(tx, "error starting tof_front ranging").unwrap();
            panic!("tof_front start_ranging failed");
        }

        let tof_left: vl53l1x::VL53L1<I2C1, PB8, PB9> =
            match vl53l1x::VL53L1::new(&mut i2c, sys_config::TOF_LEFT_ADDRESS) {
                Ok(val) => val,
                Err(_) => {
                    writeln!(tx, "tof_left initialization failed\r").unwrap();
                    panic!("tof_front initialization failed");
                }
            };
        if let Err(_) = tof_left.start_ranging(
            &mut i2c,
            Some(vl53l1x::DistanceMode::Short),
            Some(vl53l1x::TimingBudget::Tb15ms),
            Some(20),
        ) {
            writeln!(tx, "error starting tof_left ranging").unwrap();
            panic!("tof_left start_ranging failed");
        }

        // set up IMU sensor
        let mut imu = icm20948::ICM20948::new(imu_spi, imu_cs);
        match imu.init(
            icm20948::AccelFullScaleSel::Gpm2,
            icm20948::AccelDLPFSel::Disable,
            icm20948::GyroFullScaleSel::Dps250,
            icm20948::GyroDLPFSel::Disable,
            icm20948::MagMode::Continuous100Hz,
        ) {
            Ok(_) => writeln!(tx, "imu initialized").unwrap(),
            Err(e) => {
                match e {
                    icm20948::ErrorCode::ParamError => writeln!(tx, "param error").unwrap(),
                    icm20948::ErrorCode::SpiError => writeln!(tx, "SPI error").unwrap(),
                    icm20948::ErrorCode::WrongID => writeln!(tx, "wrong ID").unwrap(),
                    icm20948::ErrorCode::MagError => writeln!(tx, "magnetometer error").unwrap(),
                    icm20948::ErrorCode::MagWrongID => {
                        writeln!(tx, "magnetometer wrong ID").unwrap()
                    }
                    icm20948::ErrorCode::CSError => writeln!(tx, "CS error").unwrap(),
                }
                panic!("imu initialization failed");
            }
        }

        // IMU calibration
        let imu_gyro_bias = {
            if sys_config::CALIBRATE_IMU {
                writeln!(tx, "calibrating imu...").unwrap();
                let num_samples = sys_config::IMU_CALIBRATION_NUM_SAMPLES;
                let mut gyro_sum: (f32, f32, f32) = (0.0, 0.0, 0.0);
                for _ in 1..=num_samples {
                    while !imu.data_ready().unwrap_or(false) {}
                    if let Ok(_) = imu.read_data() {
                        gyro_sum.0 += imu.get_gyro_x();
                        gyro_sum.1 += imu.get_gyro_y();
                        gyro_sum.2 += imu.get_gyro_z();
                    }
                }
                (
                    gyro_sum.0 / (num_samples as f32),
                    gyro_sum.1 / (num_samples as f32),
                    gyro_sum.2 / (num_samples as f32),
                )
            } else {
                sys_config::DEFAULT_IMU_GYRO_BIAS_DPS
            }
        };
        writeln!(
            tx,
            "imu gyro bias (deg/s): [{}, {}, {}]",
            imu_gyro_bias.0, imu_gyro_bias.1, imu_gyro_bias.2
        )
        .unwrap();

        let tof_front_filter = sma::SmaFilter::<u16, 10>::new();
        let tof_left_filter = sma::SmaFilter::<u16, 10>::new();
        let imu_filter =
            filter::ImuFilter::<{ sys_config::IMU_SMA_FILTER_SIZE }, mahony::MahonyFilter>::new(
                mahony::MahonyFilter::new(mahony::DEFAULT_KP, mahony::DEFAULT_KI),
                imu_gyro_bias,
            );

        writeln!(tx, "system initialized\r").unwrap();

        let filter_data_prev_ticks: u64 = monotonics::now().ticks() + 1000;
        filter_data::spawn_after(Duration::<u64, 1, 1000>::secs(1)).unwrap();

        (
            Shared {
                tof_front_filter,
                tof_left_filter,
                imu_filter,
            },
            Local {
                i2c,
                tx,
                tof_front,
                tof_left,
                imu,
                filter_data_prev_ticks,
            },
            init::Monotonics(mono),
        )
    }

    use crate::filter::filter_data;
    extern "Rust" {
        #[task(local=[imu, filter_data_prev_ticks], shared=[tof_front_filter, tof_left_filter, imu_filter])]
        fn filter_data(mut cx: filter_data::Context);
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }
}
