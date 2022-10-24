#![no_main]
#![no_std]

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    #[warn(unused_imports)]
    use cortex_m::asm;
    #[allow(unused_imports)]
    use panic_write::PanicHandler;
    use stm32f4xx_hal::{
        gpio::{Output, PushPull, PA5},
        prelude::*,
    };
    use systick_monotonic::{fugit::Duration, Systick};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        led: PA5<Output<PushPull>>,
        state: bool,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let rcc = ctx.device.RCC.constrain();
        let mono = Systick::new(ctx.core.SYST, 48_000_000);

        let _clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

        let gpioa = ctx.device.GPIOA.split();
        let mut led = gpioa.pa5.into_push_pull_output();
        led.set_high();

        // Schedule the blinking task
        blink::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();

        (
            Shared {},
            Local { led, state: false },
            init::Monotonics(mono),
        )
    }

    #[task(local = [led, state])]
    fn blink(cx: blink::Context) {
        if *cx.local.state {
            cx.local.led.set_high();
            *cx.local.state = false;
        } else {
            cx.local.led.set_low();
            *cx.local.state = true;
        }
        blink::spawn_after(Duration::<u64, 1, 1000>::from_ticks(1000)).unwrap();
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }
}
