#![no_main]
#![no_std]

// Halt on panic
use panic_halt as _; // panic handler

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use braincell::drivers::tof::vl53l1x;
    use core::fmt::Write;
    use stm32f4xx_hal::{
        gpio::{PB8, PB9},
        i2c::{I2c, Mode as i2cMode},
        pac::{I2C1, USART2},
        prelude::*,
        serial::{Config, Serial, Tx},
    };
    use systick_monotonic::{fugit::Duration, Systick};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        i2c: I2c<I2C1, (PB8, PB9)>,
        tx: Tx<USART2>,
        tof: vl53l1x::VL53L1<I2C1, PB8, PB9>,
    }

    #[monotonic(binds = SysTick, default = true)]
    type MonoTimer = Systick<1000>;

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        // configure clocks
        let rcc = ctx.device.RCC.constrain();
        let mono = Systick::new(ctx.core.SYST, 48_000_000);
        let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

        // configure I2C
        let gpiob = ctx.device.GPIOB.split();
        let scl = gpiob.pb8;
        let sda = gpiob.pb9;
        let mut i2c = I2c::new(
            ctx.device.I2C1,
            (scl, sda),
            i2cMode::Standard {
                frequency: 100.kHz(),
            },
            &clocks,
        );

        // set up uart tx
        let gpioa = ctx.device.GPIOA.split();
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

        // set up ToF sensor
        let tof: vl53l1x::VL53L1<I2C1, PB8, PB9> = match vl53l1x::VL53L1::new(&mut i2c, 0x42) {
            Ok(val) => val,
            Err(_) => {
                writeln!(tx, "tof initialization failed\r").unwrap();
                panic!("tof initialization failed");
            }
        };
        if let Err(_) = tof.start_ranging(
            &mut i2c,
            Some(vl53l1x::DistanceMode::Short),
            Some(vl53l1x::TimingBudget::Tb15ms),
            Some(20),
        ) {
            writeln!(tx, "error starting tof ranging").unwrap();
        }

        writeln!(tx, "system initialized\r").unwrap();
        print_tof_data::spawn_after(Duration::<u64, 1, 1000>::secs(1)).unwrap();
        (Shared {}, Local { i2c, tx, tof }, init::Monotonics(mono))
    }

    #[task(local=[i2c, tx, tof], shared=[])]
    fn print_tof_data(cx: print_tof_data::Context) {
        if cx
            .local
            .tof
            .check_for_data_ready(cx.local.i2c)
            .unwrap_or(false)
        {
            let x: u16 = cx.local.tof.get_distance(cx.local.i2c).unwrap_or(0);
            writeln!(cx.local.tx, "distance: {x}\r").unwrap();
            if let Err(_) = cx.local.tof.clear_interrupt(cx.local.i2c) {
                writeln!(cx.local.tx, "error clearing interrupt").unwrap();
            };
        }
        // run at 100 Hz
        print_tof_data::spawn_after(Duration::<u64, 1, 1000>::millis(10)).unwrap();
    }
}
