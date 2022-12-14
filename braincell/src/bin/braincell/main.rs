#![no_main]
#![no_std]

mod config;
mod filter;
mod motors;
mod supervisor;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI1, SPI3, SPI4, EXTI1])]
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
    use braincell::drivers::tof::vl53l1x_constants;
    use braincell::filtering::{ahrs::mahony, sma};
    use core::fmt::Write;
    use cortex_m::asm;
    use panic_write::PanicHandler;
    use pid;
    use stm32f4xx_hal::{
        gpio::{
            Alternate, Input, OpenDrain, Output, Pin, PushPull, PA8, PA9, PB10, PC1, PC12, PC2, PC9,
        },
        i2c::{I2c, Mode as i2cMode},
        pac::{I2C2, I2C3, SPI2, TIM1, TIM10, TIM12, TIM14, TIM2, TIM3, TIM4, TIM5, TIM8, USART2},
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
        imu_filter: filter::ImuFilter<{ tuning::IMU_SMA_FILTER_SIZE }, mahony::MahonyFilter>,
        tof_front_filter: sma::SmaFilter<i32, { tuning::TOF_FRONT_SMA_FILTER_SIZE }>,
        tof_left_filter: sma::SmaFilter<i32, { tuning::TOF_LEFT_SMA_FILTER_SIZE }>,
        motor_setpoints: controller::motor::MotorSetPoints,
    }

    #[local]
    struct Local {
        i2c2: I2c<I2C2, (PB10, PC12)>,
        i2c3: I2c<I2C3, (PA8, PC9)>,
        tof_front: vl53l1x::VL53L1<I2C2, PB10, PC12, Pin<'C', 10, Output<OpenDrain>>>,
        tof_left: vl53l1x::VL53L1<I2C3, PA8, PC9, Pin<'C', 8, Output<OpenDrain>>>,
        imu: icm20948::ICM20948<
            Spi<SPI2, (PA9<Alternate<5>>, PC2<Alternate<5>>, PC1<Alternate<7>>)>,
            Pin<'B', 12, Output<PushPull>>,
        >,
        motors: controller::motor::Motors<
            mdd3a::MDD3A<PwmChannel<TIM1, 2>, PwmChannel<TIM1, 3>>,
            mdd3a::MDD3A<PwmChannel<TIM8, 0>, PwmChannel<TIM8, 1>>,
            mdd3a::MDD3A<PwmChannel<TIM12, 0>, PwmChannel<TIM12, 1>>,
            mdd3a::MDD3A<PwmChannel<TIM10, 0>, PwmChannel<TIM14, 0>>,
        >,
        encoder_f_left:
            n20::N20<Qei<TIM2, (Pin<'A', 15, Alternate<1>>, Pin<'B', 9, Alternate<1>>)>>,
        encoder_r_left: n20::N20<Qei<TIM3, (Pin<'A', 6, Alternate<2>>, Pin<'B', 5, Alternate<2>>)>>,
        encoder_f_right:
            n20::N20<Qei<TIM4, (Pin<'B', 6, Alternate<2>>, Pin<'B', 7, Alternate<2>>)>>,
        encoder_r_right:
            n20::N20<Qei<TIM5, (Pin<'A', 0, Alternate<2>>, Pin<'A', 1, Alternate<2>>)>>,
        filter_data_prev_ticks: u64,
        led: Pin<'A', 5, Output>,
        supervisor_state: supervisor::Data<Pin<'C', 13, Input>, f32>,
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
        let scl2 = gpiob.pb10;
        let sda2 = gpioc.pc12;
        let scl3 = gpioa.pa8;
        let sda3 = gpioc.pc9;
        let mut i2c2 = I2c::new(
            ctx.device.I2C2,
            (scl2, sda2),
            i2cMode::Standard {
                frequency: 100.kHz(),
            },
            &clocks,
        );
        let mut i2c3 = I2c::new(
            ctx.device.I2C3,
            (scl3, sda3),
            i2cMode::Standard {
                frequency: 100.kHz(),
            },
            &clocks,
        );

        // configure IMU spi and cs
        let imu_cs = gpiob.pb12.into_push_pull_output();
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
        let button = gpioc.pc13.into_input();

        // set up led
        let led = gpioa.pa5.into_push_pull_output();

        // set up ToF sensors
        let tof_front_xshut = gpioc.pc10.into_open_drain_output();
        let tof_front = match vl53l1x::VL53L1::new(
            &mut i2c2,
            sys_config::TOF_FRONT_ADDRESS,
            Some(tof_front_xshut),
        ) {
            Ok(val) => val,
            Err(_) => {
                writeln!(tx, "tof_front initialization failed\r").unwrap();
                panic!("tof_front initialization failed");
            }
        };
        if let Err(_) = tof_front.start_ranging(
            &mut i2c2,
            Some(vl53l1x_constants::DEFAULT_DM),
            Some(vl53l1x_constants::DEFAULT_TB),
            Some(vl53l1x_constants::DEFAULT_IM_MS),
        ) {
            writeln!(tx, "error starting tof_front ranging").unwrap();
            panic!("tof_front start_ranging failed");
        }

        let tof_left_xshut = gpioc.pc8.into_open_drain_output();
        let tof_left = match vl53l1x::VL53L1::new(
            &mut i2c3,
            sys_config::TOF_LEFT_ADDRESS,
            Some(tof_left_xshut),
        ) {
            Ok(val) => val,
            Err(_) => {
                writeln!(tx, "tof_left initialization failed\r").unwrap();
                panic!("tof_left initialization failed");
            }
        };
        if let Err(_) = tof_left.start_ranging(
            &mut i2c3,
            Some(vl53l1x_constants::DEFAULT_DM),
            Some(vl53l1x_constants::DEFAULT_TB),
            Some(vl53l1x_constants::DEFAULT_IM_MS),
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

        // set up filters
        let tof_front_filter = sma::SmaFilter::<i32, { tuning::TOF_FRONT_SMA_FILTER_SIZE }>::new();
        let tof_left_filter = sma::SmaFilter::<i32, { tuning::TOF_LEFT_SMA_FILTER_SIZE }>::new();
        let imu_filter =
            filter::ImuFilter::<{ tuning::IMU_SMA_FILTER_SIZE }, mahony::MahonyFilter>::new(
                mahony::MahonyFilter::new(
                    tuning::IMU_FILTER_KP,
                    tuning::IMU_FILTER_KI,
                    tuning::IMU_FILTER_USE_MAG,
                ),
                tuning::IMU_GYRO_BIAS_DPS,
            );

        //set up PWM
        let channels1 = (gpioa.pa10.into_alternate(), gpioa.pa11.into_alternate());
        let pwm1 = ctx.device.TIM1.pwm_hz(channels1, 20.kHz(), &clocks).split();

        let channels2 = (gpioc.pc6.into_alternate(), gpioc.pc7.into_alternate());
        let pwm2 = ctx.device.TIM8.pwm_hz(channels2, 20.kHz(), &clocks).split();

        let channelsmth = (gpiob.pb14.into_alternate(), gpiob.pb15.into_alternate());
        let pwm3 = ctx
            .device
            .TIM12
            .pwm_hz(channelsmth, 20.kHz(), &clocks)
            .split();

        let channel3 = gpiob.pb8.into_alternate();
        let channel4 = gpioa.pa7.into_alternate();
        let pwmtest1 = ctx.device.TIM10.pwm_hz(channel3, 20.kHz(), &clocks).split();
        let pwmtest2 = ctx.device.TIM14.pwm_hz(channel4, 20.kHz(), &clocks).split();
        let pwm4 = (pwmtest1, pwmtest2);

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
            (gpioa.pa6.into_alternate(), gpiob.pb5.into_alternate()),
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

        // supervisor state variables
        let distance_pid = pid::Pid::new(
            tuning::DISTANCE_PID_KP,
            tuning::DISTANCE_PID_KI,
            tuning::DISTANCE_PID_KD,
            tuning::DISTANCE_PID_P_LIM,
            tuning::DISTANCE_PID_I_LIM,
            tuning::DISTANCE_PID_D_LIM,
            tuning::DISTANCE_PID_OUT_LIM,
            0.0,
        );
        let side_dist_compensation_pid = pid::Pid::new(
            tuning::SIDE_DIST_COMPENSATION_PID_KP,
            tuning::SIDE_DIST_COMPENSATION_PID_KI,
            tuning::SIDE_DIST_COMPENSATION_PID_KD,
            tuning::SIDE_DIST_COMPENSATION_PID_P_LIM,
            tuning::SIDE_DIST_COMPENSATION_PID_I_LIM,
            tuning::SIDE_DIST_COMPENSATION_PID_D_LIM,
            tuning::SIDE_DIST_COMPENSATION_PID_OUT_LIM,
            0.0,
        );
        let supervisor_state =
            supervisor::Data::new(button, distance_pid, side_dist_compensation_pid);

        writeln!(tx, "system initialized\r").unwrap();

        let filter_data_prev_ticks: u64 = monotonics::now().ticks();
        filter_data::spawn_after(Duration::<u64, 1, 1000>::millis(1)).unwrap();
        speed_control::spawn_after(Duration::<u64, 1, 1000>::millis(1)).unwrap();
        blinky::spawn_after(Duration::<u64, 1, 1000>::millis(1)).unwrap();
        supervisor_task::spawn_after(Duration::<u64, 1, 1000>::millis(1000)).unwrap();

        (
            Shared {
                tx,
                tof_front_filter,
                tof_left_filter,
                imu_filter,
                motor_setpoints,
            },
            Local {
                i2c2,
                i2c3,
                tof_front,
                tof_left,
                imu,
                motors,
                encoder_f_left,
                encoder_r_left,
                encoder_f_right,
                encoder_r_right,
                filter_data_prev_ticks,
                led,
                supervisor_state,
            },
            init::Monotonics(mono),
        )
    }

    use crate::filter::filter_data;
    extern "Rust" {
        #[task(local=[imu, tof_front, tof_left, i2c2, i2c3, filter_data_prev_ticks], shared=[tx, imu_filter, tof_front_filter, tof_left_filter], priority=4)]
        fn filter_data(mut cx: filter_data::Context);
    }

    use crate::motors::speed_control;
    extern "Rust" {
        #[task(local=[motors, encoder_f_left, encoder_r_left, encoder_f_right, encoder_r_right], shared=[tx, motor_setpoints], priority=3)]
        fn speed_control(context: speed_control::Context);
    }

    use crate::supervisor::supervisor_task;
    extern "Rust" {
        #[task(local=[supervisor_state], shared=[motor_setpoints, tof_front_filter, tof_left_filter, imu_filter, tx], priority=2)]
        fn supervisor_task(context: supervisor_task::Context);
    }

    #[task(local=[led], shared=[tx], priority=1)]
    fn blinky(cx: blinky::Context) {
        cx.local.led.toggle();
        blinky::spawn_after(Duration::<u64, 1, 1000>::millis(1000)).unwrap();
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }
}
