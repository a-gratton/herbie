#![no_main]
#![no_std]

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use braincell::drivers::encoder::n20;
    use braincell::drivers::motor::mdd3a;
    use braincell::drivers::motor::mdd3a::SetPower;
    use braincell::drivers::motor::mdd3a::Start;

    use cortex_m::asm;

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
        motor1: mdd3a::MDD3A<PwmChannel<TIM1, 2>, PwmChannel<TIM1, 3>>,
        motor2: mdd3a::MDD3A<PwmChannel<TIM8, 0>, PwmChannel<TIM8, 1>>,
        motor3: mdd3a::MDD3A<PwmChannel<TIM12, 0>, PwmChannel<TIM12, 1>>,
        motor4: mdd3a::MDD3A<PwmChannel<TIM10, 0>, PwmChannel<TIM14, 0>>,
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

        let mut motor1 = mdd3a::MDD3A::new(pwm1);
        let mut motor2 = mdd3a::MDD3A::new(pwm2);
        let mut motor3 = mdd3a::MDD3A::new(pwm3);
        let mut motor4 = mdd3a::MDD3A::new(pwm4);

        //default pwm channels are not enabled
        motor1.start();
        motor2.start();
        motor3.start();
        motor4.start();

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
                motor1,
                motor2,
                motor3,
                motor4,
            },
            init::Monotonics(mono),
        )
    }

    #[task(local=[tx, encoder1, encoder2, encoder3, encoder4, motor1, motor2, motor3, motor4], shared=[])]
    fn read_speed(cx: read_speed::Context) {
        let start: u64 = monotonics::now().ticks();

        //spin 2 sec
        cx.local.motor1.set_power(100.0);
        cx.local.motor2.set_power(100.0);
        cx.local.motor3.set_power(100.0);
        cx.local.motor4.set_power(100.0);

        while monotonics::now().ticks() - start < 2000 {
            let timeis: f32 = monotonics::now().ticks() as f32 * 0.001;
            let new_count1 = cx.local.encoder1.get_speed(timeis);
            writeln!(cx.local.tx, "enc1: {new_count1}\r").unwrap();
            let new_count2 = cx.local.encoder2.get_speed(timeis);
            writeln!(cx.local.tx, "enc2: {new_count2}\r").unwrap();
            let new_count3 = cx.local.encoder3.get_speed(timeis);
            writeln!(cx.local.tx, "enc3: {new_count3}\r").unwrap();
            let new_count4 = cx.local.encoder4.get_speed(timeis);
            writeln!(cx.local.tx, "enc4: {new_count4}\r").unwrap();
            asm::delay(1000000);
        }

        cx.local.motor2.set_power(0.0);
        cx.local.motor3.set_power(0.0);
        cx.local.motor4.set_power(0.0);
        cx.local.motor1.set_power(0.0);

        // run at 100 Hz
        //read_speed::spawn_after(Duration::<u64, 1, 1000>::millis(10)).unwrap();
    }
}
