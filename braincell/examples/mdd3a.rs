#![no_main]
#![no_std]

// Halt on panic
//use panic_halt as _; // panic handler

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use braincell::drivers::motor::mdd3a;
    use core::fmt::Write;
    use panic_write::PanicHandler;
    use stm32f4xx_hal::{
        pac::USART2,
        pac::{TIM1, TIM2, TIM8, TIM12},
        prelude::*,
        serial::{Config, Serial, Tx},
        timer::pwm::PwmChannel,
    };
    use systick_monotonic::{fugit::Duration, Systick};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        tx: core::pin::Pin<panic_write::PanicHandler<Tx<USART2>>>,
        motor1: mdd3a::MDD3A<PwmChannel<TIM1, 2>, PwmChannel<TIM1, 3>>,
        motor2: mdd3a::MDD3A<PwmChannel<TIM2, 0>, PwmChannel<TIM2, 1>>,
        motor3: mdd3a::MDD3A<PwmChannel<TIM8, 2>, PwmChannel<TIM8, 3>>,
        motor4: mdd3a::MDD3A<PwmChannel<TIM12, 0>, PwmChannel<TIM12, 1>>,
        motorsmth: mdd3a::MDD3A<PwmChannel<TIM2, 2>, PwmChannel<TIM2,3>>,
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

        // set up PWM

        //let channels2 = (gpioa.pa0.into_alternate(), gpioa.pa1.into_alternate(), gpiob.pb10.into_alternate(), gpioa.pa3.into_alternate()); //D6 2/3 D0 2/4
        //let channelsx = (gpioa.pa0.into_alternate(), gpioa.pa1.into_alternate());
        //let pwms = ctx.device.TIM2.pwm_hz(channels2, 20.kHz(), &clocks).split();
        //let pmw2 = (pwms.0, pwms.1);
        //let pwm4 = (pwms.2, pwms.3);

        //let channels1 = (gpioa.pa10.into_alternate(), gpioa.pa11.into_alternate()); // 1/3  1/4
        let channels2 = (gpioa.pa0.into_alternate(), gpioa.pa1.into_alternate(), gpiob.pb10.into_alternate(), gpioa.pa3.into_alternate()); //D6 2/3 D0 2/4
        //let channels3 = (gpioc.pc8.into_alternate(), gpioc.pc9.into_alternate()); // 3/3  3/4
        //let channels4 = (gpiob.pb14.into_alternate(), gpiob.pb15.into_alternate()); // 4/3 4/4

        //let pwm1 = ctx.device.TIM1.pwm_hz(channels1, 20.kHz(), &clocks).split();
        //let pwm2 = ctx.device.TIM2.pwm_hz(channels2, 20.kHz(), &clocks).split();
        let pwms = ctx.device.TIM2.pwm_hz(channels2, 20.kHz(), &clocks).split();
        let pwm2 = (pwms.0, pwms.1);
        let pwm5 = (pwms.2, pwms.3);

        //let pwm3 = ctx.device.TIM8.pwm_hz(channels3, 20.kHz(), &clocks).split();
        //let pwm4 = ctx.device.TIM12.pwm_hz(channels4, 20.kHz(), &clocks).split();

        //let mut motor1 = mdd3a::MDD3A::new(pwm1);
        let mut motor2 = mdd3a::MDD3A::new(pwm2);
        let mut motorsmth = mdd3a::MDD3A::new(pwm5);
        //let mut motor3 = mdd3a::MDD3A::new(pwm3);
        //let mut motor4 = mdd3a::MDD3A::new(pwm4);

        //let max_duty = motor2.get_duty();
        //let md1 = max_duty.0;
        //let md2 = max_duty.1;

        //writeln!(tx, "({md1},{md2})\r").unwrap();
/*
        motor1.set_power(0.0);
        motor2.set_power(0.0);
        motor3.set_power(0.0);
        motor4.set_power(0.0);
        motor1.start();
        motor2.start();
        motor3.start();
        motor4.start();
        */

        //writeln!(tx, "system initialized\r").unwrap();
        set_pwm_pwr::spawn_after(Duration::<u64, 1, 1000>::secs(1)).unwrap();
        (
            Shared {},
            Local {
                tx,
                motor1,
                motor2,
                motor3,
                motor4,
                motorsmth,
            },
            init::Monotonics(mono),
        )
    }

    #[task(local=[tx, motor1, motor2, motor3, motor4], shared=[])]
    fn set_pwm_pwr(cx: set_pwm_pwr::Context) {
        let motor1 = cx.local.motor1;
        let motor2 = cx.local.motor2;
        //let task_start = monotonics::now();
        motor1.set_power(100.0);
        motor2.set_power(100.0);

        // run at 100 Hz
        set_pwm_pwr::spawn_after(Duration::<u64, 1, 1000>::millis(10)).unwrap();
    }
}
