#![no_main]
#![no_std]

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use braincell::drivers::encoder::n20;
    
    use core::fmt::Write;
    use panic_write::PanicHandler;
    use stm32f4xx_hal::{
        pac::USART2,
        pac::{TIM2, TIM3, TIM4, TIM5},
        prelude::*,
        serial::{Config, Serial, Tx},
        qei::Qei,
        
    };
    use systick_monotonic::{fugit::Duration, Systick};
    use stm32f4xx_hal::gpio::Pin;
    use stm32f4xx_hal::gpio::Alternate;


    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        tx: core::pin::Pin<panic_write::PanicHandler<Tx<USART2>>>,
        encoder1: n20::N20<Qei<TIM2, (Pin<'A', 15, Alternate<1>>, Pin<'B', 9, Alternate<1>>)>>,
        encoder2: n20::N20<Qei<TIM3, (Pin<'A', 6, Alternate<2>>, Pin<'A', 7, Alternate<2>>)>>,
        encoder3: n20::N20<Qei<TIM4, (Pin<'B', 6, Alternate<2>>, Pin<'B', 7, Alternate<2>>)>>,
        encoder4: n20::N20<Qei<TIM5, (Pin<'A', 0, Alternate<2>>, Pin<'A', 1, Alternate<2>>)>>,

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

        let encoder1_pins = (gpioa.pa15.into_alternate(), gpiob.pb9.into_alternate());
        let encoder2_pins = (gpioa.pa6.into_alternate(), gpioa.pa7.into_alternate());
        let encoder3_pins = (gpiob.pb6.into_alternate(), gpiob.pb7.into_alternate());
        let encoder4_pins = (gpioa.pa0.into_alternate(), gpioa.pa1.into_alternate());
    
        let encoder1_timer = ctx.device.TIM2;
        let encoder2_timer = ctx.device.TIM3;
        let encoder3_timer = ctx.device.TIM4;
        let encoder4_timer = ctx.device.TIM5;
    
        let encoder1_qei = Qei::new(encoder1_timer, encoder1_pins);
        let encoder2_qei = Qei::new(encoder2_timer, encoder2_pins);
        let encoder3_qei = Qei::new(encoder3_timer, encoder3_pins);
        let encoder4_qei = Qei::new(encoder4_timer, encoder4_pins);
        let mut encoder1 = n20::N20::new(encoder1_qei);
        let mut encoder2 = n20::N20::new(encoder2_qei);
        let mut encoder3 = n20::N20::new(encoder3_qei);
        let mut encoder4 = n20::N20::new(encoder4_qei);

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
            },
            init::Monotonics(mono),
        )
    }

    #[task(local=[tx, encoder1, encoder2, encoder3, encoder4], shared=[])]
    fn read_speed(cx: read_speed::Context) {

        let timeis: u64 = monotonics::now().ticks();
        let new_count1 = cx.local.encoder1.get_speed(timeis);
        let new_count2 = cx.local.encoder2.get_speed(timeis);
        let new_count3 = cx.local.encoder3.get_speed(timeis);
        let new_count4 = cx.local.encoder4.get_speed(timeis);
        writeln!(cx.local.tx, "deg/s: {new_count4}\r").unwrap();

        // run at 100 Hz
        read_speed::spawn_after(Duration::<u64, 1, 1000>::millis(10)).unwrap();
    }
}
