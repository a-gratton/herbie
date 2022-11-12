#![no_main]
#![no_std]

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI2])]
mod app {
    use braincell::drivers::imu::icm20948;
    use core::fmt::Write;
    use cortex_m::asm;
    use panic_write::PanicHandler;
    use stm32f4xx_hal::{
        gpio::{Alternate, Output, Pin, PushPull, PB3, PB4, PB5},
        pac::{SPI1, USART2},
        prelude::*,
        serial::{Config, Serial, Tx},
        spi::{Mode, Phase, Polarity, Spi},
    };
    use systick_monotonic::{ExtU64, Systick};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        tx: core::pin::Pin<panic_write::PanicHandler<Tx<USART2>>>,
        imu: icm20948::ICM20948<
            Spi<SPI1, (PB3<Alternate<5>>, PB4<Alternate<5>>, PB5<Alternate<5>>)>,
            Pin<'A', 4, Output<PushPull>>,
        >,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    const CALIBRATION_TIME_MS: u64 = 120_000;

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let rcc = ctx.device.RCC.constrain();
        let mono = Systick::new(ctx.core.SYST, 48_000_000);

        let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

        let gpioa = ctx.device.GPIOA.split();
        let gpiob = ctx.device.GPIOB.split();

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

        imu_calibration::spawn_after(ExtU64::millis(100)).unwrap();

        (Shared {}, Local { tx, imu }, init::Monotonics(mono))
    }

    #[task(local = [tx, imu], shared = [])]
    fn imu_calibration(cx: imu_calibration::Context) {
        writeln!(cx.local.tx, "Calibrating IMU...").unwrap();
        let start_ticks = monotonics::now().ticks();
        let mut gyro_sum: (f32, f32, f32) = (0.0, 0.0, 0.0);
        let mut num_samples = 0;
        while monotonics::now().ticks() - start_ticks < CALIBRATION_TIME_MS {
            while !cx.local.imu.data_ready().unwrap_or(false) {}
            if let Ok(_) = cx.local.imu.read_data() {
                gyro_sum.0 += cx.local.imu.get_gyro_x();
                gyro_sum.1 += cx.local.imu.get_gyro_y();
                gyro_sum.2 += cx.local.imu.get_gyro_z();
                num_samples += 1;
            }
        }
        let imu_gyro_bias = (
            gyro_sum.0 / (num_samples as f32),
            gyro_sum.1 / (num_samples as f32),
            gyro_sum.2 / (num_samples as f32),
        );

        writeln!(
            cx.local.tx,
            "Calibrated gyro bias (deg/s): [{}, {}, {}]",
            imu_gyro_bias.0, imu_gyro_bias.1, imu_gyro_bias.2
        )
        .unwrap();
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }
}
