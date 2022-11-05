#![no_main]
#![no_std]

// Halt on panic
use panic_halt as _; // panic handler

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use braincell::drivers::motor::mdd3a;
    use core::fmt::Write;
    use stm32f4xx_hal::{
        pac::{USART2},
        prelude::*,
        serial::{Config, Serial, Tx},
        timer::pwm::PwmChannel,
        pac::{TIM1, TIM2, TIM3, TIM4},
    };
    use systick_monotonic::{fugit::Duration, Systick};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        tx: Tx<USART2>,
        motor1: mdd3a::MDD3A<PwmChannel<TIM1,0>, PwmChannel<TIM1,1>>,
        motor2: mdd3a::MDD3A<PwmChannel<TIM2,2>,PwmChannel<TIM2,3>>,
        motor3: mdd3a::MDD3A<PwmChannel<TIM3,0>, PwmChannel<TIM3,1>>,
        motor4: mdd3a::MDD3A<PwmChannel<TIM4,0>, PwmChannel<TIM4,1>>,
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

        // set up PWM
        let channels1 = (gpioa.pa8.into_alternate(), gpioa.pa9.into_alternate()); //D7 1/1 D8 1/2 
        let channels2 = (gpiob.pb10.into_alternate(), gpioa.pa3.into_alternate()); //D6 2/3 D0 2/4
        let channels3 = (gpioa.pa6.into_alternate(), gpioc.pc7.into_alternate()); //d12 3/1 D9 3/2
        let channels4 = (gpiob.pb6.into_alternate(), gpiob.pb7.into_alternate()); //D10 4/1 farleft on same line as lowgnd 4/2

        let pwm1 = ctx.device.TIM1.pwm_hz(channels1, 20.kHz(), &clocks).split();
        let pwm2 = ctx.device.TIM2.pwm_hz(channels2, 20.kHz(), &clocks).split();
        let pwm3 = ctx.device.TIM3.pwm_hz(channels3, 20.kHz(), &clocks).split();
        let pwm4 = ctx.device.TIM4.pwm_hz(channels4, 20.kHz(), &clocks).split();

        let mut motor1 = mdd3a::MDD3A::new(pwm1);
        let mut motor2 = mdd3a::MDD3A::new(pwm2);
        let mut motor3 = mdd3a::MDD3A::new(pwm3);
        let mut motor4 = mdd3a::MDD3A::new(pwm4);

        let max_duty = motor2.get_duty();
        let md1 = max_duty.0;
        let md2 = max_duty.1;

        writeln!(tx,"({md1},{md2})\r").unwrap();

        let power = mdd3a::convert_pidout_to_power(0.0);
        motor1.set_power(power);
        motor2.set_power(power);
        motor3.set_power(power);
        motor4.set_power(power);
        motor1.start();
        motor2.start();
        motor3.start();
        motor4.start();


        writeln!(tx, "system initialized\r").unwrap();
        set_pwm_pwr::spawn_after(Duration::<u64, 1, 1000>::secs(1)).unwrap();
        (Shared {}, Local { tx, motor1, motor2, motor3, motor4 }, init::Monotonics(mono))
    }

    #[task(local=[tx, motor1, motor2, motor3, motor4], shared=[])]
    fn set_pwm_pwr(cx: set_pwm_pwr::Context) {
        let motor1 = cx.local.motor1;
        let motor2 = cx.local.motor2;
        //let task_start = monotonics::now();
        let power = mdd3a::convert_pidout_to_power(100.0);
        motor1.set_power(power);
        motor2.set_power(power);



        // run at 100 Hz
        set_pwm_pwr::spawn_after(Duration::<u64, 1, 1000>::millis(10)).unwrap();
    }
}