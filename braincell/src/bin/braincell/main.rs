#![no_main]
#![no_std]

// Halt on panic
use panic_halt as _; // panic handler
mod config;

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use crate::config::sys_config;
    use braincell::drivers::tof::vl53l1x;
    use core::fmt::Write;
    use cortex_m::asm;
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
        tof_front: vl53l1x::VL53L1<I2C1, PB8, PB9>,
        tof_left: vl53l1x::VL53L1<I2C1, PB8, PB9>,
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

        // set up ToF sensors
        let tof_front: vl53l1x::VL53L1<I2C1, PB8, PB9> =
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

        let tof_left: vl53l1x::VL53L1<I2C1, PB8, PB9> =
            match vl53l1x::VL53L1::new(&mut i2c, sys_config::TOF_LEFT_ADDRESS) {
                Ok(val) => val,
                Err(_) => {
                    writeln!(tx, "tof_left initialization failed\r").unwrap();
                    panic!("tof_front initialization failed");
                }
            };
        if let Err(_) = tof_left.start_ranging(
            &mut i2c,
            Some(vl53l1x::DistanceMode::Short),
            Some(vl53l1x::TimingBudget::Tb15ms),
            Some(20),
        ) {
            writeln!(tx, "error starting tof_left ranging").unwrap();
            panic!("tof_left start_ranging failed");
        }

        writeln!(tx, "system initialized\r").unwrap();

        (
            Shared {},
            Local {
                i2c,
                tx,
                tof_front,
                tof_left,
            },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            asm::nop();
        }
    }
}
