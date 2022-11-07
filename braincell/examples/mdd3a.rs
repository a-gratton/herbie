#![no_main]
#![no_std]

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use braincell::drivers::motor::mdd3a;
    use core::fmt::Write;
    use panic_write::PanicHandler;
    use stm32f4xx_hal::{
        pac::USART2,
        pac::{TIM1, TIM8},
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
        motor1: mdd3a::MDD3A<PwmChannel<TIM1, 0>, PwmChannel<TIM1, 1>>,
        motor2: mdd3a::MDD3A<PwmChannel<TIM1, 2>, PwmChannel<TIM1, 3>>,
        motor3: mdd3a::MDD3A<PwmChannel<TIM8, 0>, PwmChannel<TIM8, 1>>,
        motor4: mdd3a::MDD3A<PwmChannel<TIM8, 2>, PwmChannel<TIM8, 3>>,
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

        let mut motor1 = mdd3a::MDD3A::new(pwm1);
        let mut motor2 = mdd3a::MDD3A::new(pwm2);
        let mut motor3 = mdd3a::MDD3A::new(pwm3);
        let mut motor4 = mdd3a::MDD3A::new(pwm4);

        motor1.start();
        motor2.start();
        motor3.start();
        motor4.start();

        writeln!(tx, "system initialized\r").unwrap();

        set_pwm_pwr::spawn_after(Duration::<u64, 1, 1000>::secs(1)).unwrap();
        (
            Shared {},
            Local {
                tx,
                motor1,
                motor2,
                motor3,
                motor4,
            },
            init::Monotonics(mono),
        )
    }

    #[task(local=[tx, motor1, motor2, motor3, motor4], shared=[])]
    fn set_pwm_pwr(cx: set_pwm_pwr::Context) {
        cx.local.motor1.set_power(100.0);
        cx.local.motor2.set_power(100.0);
        cx.local.motor3.set_power(100.0);
        cx.local.motor4.set_power(100.0);

        // run at 100 Hz
        set_pwm_pwr::spawn_after(Duration::<u64, 1, 1000>::millis(10)).unwrap();
    }
}
