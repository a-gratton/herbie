#![no_main]
#![no_std]

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use braincell::controller;
    use braincell::controller::motor::{Direction, MotorDirections};
    use braincell::drivers::encoder::n20;
    use braincell::drivers::motor::mdd3a;

    use core::fmt::Write;
    use panic_write::PanicHandler;
    use stm32f4xx_hal::gpio::Alternate;
    use stm32f4xx_hal::gpio::Pin;
    use stm32f4xx_hal::{
        pac::USART2,
        pac::{TIM1, TIM10, TIM12, TIM14, TIM2, TIM3, TIM4, TIM5, TIM8},
        prelude::*,
        qei::Qei,
        serial::{Config, Serial, Tx},
        timer::pwm::PwmChannel,
    };
    use systick_monotonic::{fugit::Duration, Systick};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        tx: core::pin::Pin<panic_write::PanicHandler<Tx<USART2>>>,
        encoder1: n20::N20<Qei<TIM2, (Pin<'A', 15, Alternate<1>>, Pin<'B', 9, Alternate<1>>)>>,
        encoder2: n20::N20<Qei<TIM3, (Pin<'A', 6, Alternate<2>>, Pin<'B', 5, Alternate<2>>)>>,
        encoder3: n20::N20<Qei<TIM4, (Pin<'B', 6, Alternate<2>>, Pin<'B', 7, Alternate<2>>)>>,
        encoder4: n20::N20<Qei<TIM5, (Pin<'A', 0, Alternate<2>>, Pin<'A', 1, Alternate<2>>)>>,
        motors: controller::motor::Motors<
            mdd3a::MDD3A<PwmChannel<TIM1, 2>, PwmChannel<TIM1, 3>>,
            mdd3a::MDD3A<PwmChannel<TIM8, 0>, PwmChannel<TIM8, 1>>,
            mdd3a::MDD3A<PwmChannel<TIM12, 0>, PwmChannel<TIM12, 1>>,
            mdd3a::MDD3A<PwmChannel<TIM10, 0>, PwmChannel<TIM14, 0>>,
        >,
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
        let encoder1 = n20::N20::new(encoder1_qei);
        let encoder2 = n20::N20::new(encoder2_qei);
        let encoder3 = n20::N20::new(encoder3_qei);
        let encoder4 = n20::N20::new(encoder4_qei);

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

        let motor1 = mdd3a::MDD3A::new(pwm1);
        let motor2 = mdd3a::MDD3A::new(pwm2);
        let motor3 = mdd3a::MDD3A::new(pwm3);
        let motor4 = mdd3a::MDD3A::new(pwm4);

        let tune: controller::pid_params::TuningParams = controller::pid_params::TuningParams {
            kp: 0.000015,
            ki: 0.003,
            kd: 0.00012,
            p_lim: 100.0,
            i_lim: 100.0,
            d_lim: 100.0,
            out_lim: 100.0,
        };
        let motor_directions: MotorDirections = MotorDirections {
            f_left: Direction::Forward,
            r_left: Direction::Forward,
            f_right: Direction::Backward,
            r_right: Direction::Backward,
        };
        let motors =
            controller::motor::Motors::new(motor1, motor2, motor3, motor4, tune, motor_directions);

        writeln!(tx, "system initialized\r").unwrap();

        read_speed::spawn_after(Duration::<u64, 1, 1000>::secs(1)).unwrap();
        (
            Shared {},
            Local {
                tx,
                encoder1,
                encoder2,
                encoder3,
                encoder4,
                motors,
            },
            init::Monotonics(mono),
        )
    }

    #[task(local=[tx, encoder1, encoder2, encoder3, encoder4, motors], shared=[])]
    fn read_speed(cx: read_speed::Context) {
        let start = monotonics::now().ticks() as f32 * 0.001;
        writeln!(cx.local.tx, "time: {start}, pwr 0.0, vel 0.0\r").unwrap();

        for setspeed in [2000.0, -2000.0, 1320.0, -1320.0, 0.0] {
            let start = monotonics::now().ticks() as f32 * 0.001;
            let setpoints = controller::motor::MotorSetPoints {
                f_left: setspeed,
                r_left: setspeed,
                f_right: setspeed,
                r_right: setspeed,
            };
            cx.local.motors.set_speed_targets(&setpoints);
            while monotonics::now().ticks() as f32 * 0.001 - start < 3.0 {
                let timeis: f32 = monotonics::now().ticks() as f32 * 0.001;
                let vels = controller::motor::VelocityMeasurement {
                    f_left: cx.local.encoder1.get_speed(timeis),
                    r_left: cx.local.encoder2.get_speed(timeis),
                    f_right: cx.local.encoder3.get_speed(timeis),
                    r_right: cx.local.encoder4.get_speed(timeis),
                };
                cx.local.motors.step(&vels);
                let fl = vels.f_left;
                let rl = vels.r_left;
                let fr = vels.f_right;
                let rr = vels.r_right;
                writeln!(
                    cx.local.tx,
                    "setspeed {setspeed} f_left {fl} r_left {rl} f_right {fr} r_right {rr}\r"
                )
                .unwrap();
            }
        }
        cx.local.motors.stop();
    }
}
