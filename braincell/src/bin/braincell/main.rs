#![no_main]
#![no_std]

mod config;
mod filter;
mod motors;
mod supervisor;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI2])]
mod app {
    use crate::config::sys_config;
    use crate::config::tuning;
    use crate::filter;
    use crate::supervisor;
    use braincell::controller;
    use braincell::drivers::encoder::n20;
    use braincell::drivers::imu::icm20948;
    use braincell::drivers::motor::mdd3a;
    use braincell::drivers::tof::vl53l1x;
    use braincell::filtering::{ahrs::mahony, sma};
    use core::fmt::Write;
    use cortex_m::asm;
    use panic_write::PanicHandler;
    use stm32f4xx_hal::{
        gpio::{Alternate, Input, Output, Pin, PushPull, PB10, PB3, PB4, PB5, PC12},
        i2c::{I2c, Mode as i2cMode},
        pac::{I2C2, SPI1, TIM1, TIM2, TIM3, TIM4, TIM5, TIM8, USART2},
        prelude::*,
        qei::Qei,
        serial::{Config, Serial, Tx},
        spi::{Mode, Phase, Polarity, Spi},
        timer::pwm::PwmChannel,
    };
    use systick_monotonic::{fugit::Duration, Systick};

    #[shared]
    struct Shared {
        tx: core::pin::Pin<panic_write::PanicHandler<Tx<USART2>>>,
        imu_filter: filter::ImuFilter<{ sys_config::IMU_SMA_FILTER_SIZE }, mahony::MahonyFilter>,
        tof_front_filter: sma::SmaFilter<i32, 10>,
        motor_setpoints: controller::motor::MotorSetPoints,
    }

    #[local]
    struct Local {
        i2c: I2c<I2C2, (PB10, PC12)>,
        tof_front: vl53l1x::VL53L1<I2C2, PB10, PC12>,
        imu: icm20948::ICM20948<
            Spi<SPI1, (PB3<Alternate<5>>, PB4<Alternate<5>>, PB5<Alternate<5>>)>,
            Pin<'A', 4, Output<PushPull>>,
        >,
        motors: controller::motor::Motors<
            mdd3a::MDD3A<PwmChannel<TIM1, 0>, PwmChannel<TIM1, 1>>,
            mdd3a::MDD3A<PwmChannel<TIM1, 2>, PwmChannel<TIM1, 3>>,
            mdd3a::MDD3A<PwmChannel<TIM8, 0>, PwmChannel<TIM8, 1>>,
            mdd3a::MDD3A<PwmChannel<TIM8, 2>, PwmChannel<TIM8, 3>>,
        >,
        encoder_f_left:
            n20::N20<Qei<TIM2, (Pin<'A', 15, Alternate<1>>, Pin<'B', 9, Alternate<1>>)>>,
        encoder_r_left: n20::N20<Qei<TIM3, (Pin<'A', 6, Alternate<2>>, Pin<'A', 7, Alternate<2>>)>>,
        encoder_f_right:
            n20::N20<Qei<TIM4, (Pin<'B', 6, Alternate<2>>, Pin<'B', 7, Alternate<2>>)>>,
        encoder_r_right:
            n20::N20<Qei<TIM5, (Pin<'A', 0, Alternate<2>>, Pin<'A', 1, Alternate<2>>)>>,
        filter_data_prev_ticks: u64,
        button: Pin<'C', 13, Input>,
        state: supervisor::State,
        curr_leg: usize,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // configure clocks
        let rcc = ctx.device.RCC.constrain();
        let mono = Systick::new(ctx.core.SYST, 48_000_000);
        let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

        let gpioa = ctx.device.GPIOA.split();
        let gpiob = ctx.device.GPIOB.split();
        let gpioc = ctx.device.GPIOC.split();

        // configure I2C
        let scl = gpiob.pb10;
        let sda = gpioc.pc12;
        let mut i2c = I2c::new(
            ctx.device.I2C2,
            (scl, sda),
            i2cMode::Standard {
                frequency: 100.kHz(),
            },
            &clocks,
        );

        // configure IMU spi and cs
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

        // set up button
        let gpioc = ctx.device.GPIOC.split();
        let button = gpioc.pc13.into_input();

        // set up ToF sensors
        let tof_front: vl53l1x::VL53L1<I2C2, PB10, PC12> =
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

        let tof_front_filter = sma::SmaFilter::<i32, 10>::new();
        let imu_filter =
            filter::ImuFilter::<{ sys_config::IMU_SMA_FILTER_SIZE }, mahony::MahonyFilter>::new(
                mahony::MahonyFilter::new(
                    mahony::DEFAULT_KP,
                    mahony::DEFAULT_KI,
                    sys_config::IMU_USE_MAG,
                ),
                sys_config::IMU_GYRO_BIAS_DPS,
            );

        //set up PWM
        let channels1 = (
            gpioa.pa8.into_alternate(),
            gpioa.pa9.into_alternate(),
            gpioa.pa10.into_alternate(),
            gpioa.pa11.into_alternate(),
        );
        let pwms1 = ctx.device.TIM1.pwm_hz(channels1, 20.kHz(), &clocks).split();
        let pwm1 = (pwms1.0, pwms1.1);
        let pwm2 = (pwms1.2, pwms1.3);

        let channels2 = (
            gpioc.pc6.into_alternate(),
            gpioc.pc7.into_alternate(),
            gpioc.pc8.into_alternate(),
            gpioc.pc9.into_alternate(),
        );
        let pwms2 = ctx.device.TIM8.pwm_hz(channels2, 20.kHz(), &clocks).split();
        let pwm3 = (pwms2.0, pwms2.1);
        let pwm4 = (pwms2.2, pwms2.3);

        let motor_f_left = mdd3a::MDD3A::new(pwm1);
        let motor_r_left = mdd3a::MDD3A::new(pwm2);
        let motor_f_right = mdd3a::MDD3A::new(pwm3);
        let motor_r_right = mdd3a::MDD3A::new(pwm4);

        let tune: controller::pid_params::TuningParams = controller::pid_params::TuningParams {
            kp: tuning::MOTOR_KP,
            ki: tuning::MOTOR_KI,
            kd: tuning::MOTOR_KD,
            p_lim: tuning::MOTOR_P_LIM,
            i_lim: tuning::MOTOR_I_LIM,
            d_lim: tuning::MOTOR_D_LIM,
            out_lim: tuning::MOTOR_OUT_LIM,
        };
        let motors = controller::motor::Motors::new(
            motor_f_left,
            motor_r_left,
            motor_f_right,
            motor_r_right,
            tune,
            sys_config::MOTOR_DIRECTIONS,
        );
        let motor_setpoints = controller::motor::MotorSetPoints::default();

        //setup encoders
        let encoder1_qei = Qei::new(
            ctx.device.TIM2,
            (gpioa.pa15.into_alternate(), gpiob.pb9.into_alternate()),
        );
        let encoder2_qei = Qei::new(
            ctx.device.TIM3,
            (gpioa.pa6.into_alternate(), gpioa.pa7.into_alternate()),
        );
        let encoder3_qei = Qei::new(
            ctx.device.TIM4,
            (gpiob.pb6.into_alternate(), gpiob.pb7.into_alternate()),
        );
        let encoder4_qei = Qei::new(
            ctx.device.TIM5,
            (gpioa.pa0.into_alternate(), gpioa.pa1.into_alternate()),
        );
        let encoder_f_left = n20::N20::new(encoder1_qei);
        let encoder_r_left = n20::N20::new(encoder2_qei);
        let encoder_f_right = n20::N20::new(encoder3_qei);
        let encoder_r_right = n20::N20::new(encoder4_qei);

        // initial state
        let state: supervisor::State = supervisor::State::Idle;
        let curr_leg: usize = 0;

        writeln!(tx, "system initialized\r").unwrap();

        let filter_data_prev_ticks: u64 = monotonics::now().ticks() + 1000;
        filter_data::spawn_after(Duration::<u64, 1, 1000>::secs(1)).unwrap();
        speed_control::spawn_after(Duration::<u64, 1, 1000>::secs(1)).unwrap();

        (
            Shared {
                tx,
                tof_front_filter,
                imu_filter,
                motor_setpoints,
            },
            Local {
                i2c,
                tof_front,
                imu,
                motors,
                encoder_f_left,
                encoder_r_left,
                encoder_f_right,
                encoder_r_right,
                filter_data_prev_ticks,
                button,
                state,
                curr_leg,
            },
            init::Monotonics(mono),
        )
    }

    use crate::filter::filter_data;
    extern "Rust" {
        #[task(local=[imu, tof_front, i2c, filter_data_prev_ticks], shared=[tx, imu_filter, tof_front_filter])]
        fn filter_data(mut cx: filter_data::Context);
    }

    use crate::motors::speed_control;
    extern "Rust" {
        #[task(local=[motors, encoder_f_left, encoder_r_left, encoder_f_right, encoder_r_right], shared=[tx, motor_setpoints])]
        fn speed_control(context: speed_control::Context);
    }

    // use crate::supervisor::supervisor;
    // extern "Rust" {
    //     #[task(local=[button, state, curr_leg], shared=[motor_setpoints, tof_front_filter, imu_filter])]
    //     fn supervisor(context: supervisor::Context);
    // }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }
}
