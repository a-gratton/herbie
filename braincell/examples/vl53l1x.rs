#![no_main]
#![no_std]

#[rtic::app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [SPI1])]
mod app {
    use braincell::drivers::tof::{
        vl53l1x,
        vl53l1x_constants::{DEFAULT_DM, DEFAULT_IM_MS, DEFAULT_TB},
    };
    use core::fmt::Write;
    use panic_write::PanicHandler;
    use stm32f4xx_hal::{
        gpio::{PA8, PB10, PC12, PC9},
        i2c::{I2c, Mode as i2cMode},
        pac::{I2C2, I2C3, USART2},
        prelude::*,
        serial::{Config, Serial, Tx},
    };
    use systick_monotonic::{fugit::Duration, Systick};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        i2c2: I2c<I2C2, (PB10, PC12)>,
        i2c3: I2c<I2C3, (PA8, PC9)>,
        tx: core::pin::Pin<panic_write::PanicHandler<Tx<USART2>>>,
        tof_front: vl53l1x::VL53L1<I2C2, PB10, PC12>,
        tof_left: vl53l1x::VL53L1<I2C3, PA8, PC9>,
        front_dist: Option<u16>,
        left_dist: Option<u16>,
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
        let gpioa = ctx.device.GPIOA.split();
        let gpiob = ctx.device.GPIOB.split();
        let gpioc = ctx.device.GPIOC.split();
        let scl2 = gpiob.pb10;
        let sda2 = gpioc.pc12;
        let scl3 = gpioa.pa8;
        let sda3 = gpioc.pc9;
        let mut i2c2 = I2c::new(
            ctx.device.I2C2,
            (scl2, sda2),
            i2cMode::Standard {
                frequency: 200.kHz(),
            },
            &clocks,
        );
        let mut i2c3 = I2c::new(
            ctx.device.I2C3,
            (scl3, sda3),
            i2cMode::Standard {
                frequency: 200.kHz(),
            },
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

        // set up ToF sensors
        let tof_front: vl53l1x::VL53L1<I2C2, PB10, PC12> =
            match vl53l1x::VL53L1::new(&mut i2c2, 0x42) {
                Ok(val) => val,
                Err(_) => {
                    writeln!(tx, "tof_front initialization failed\r").unwrap();
                    panic!("tof_front initialization failed");
                }
            };
        if let Err(_) = tof_front.start_ranging(
            &mut i2c2,
            Some(DEFAULT_DM),
            Some(DEFAULT_TB),
            Some(DEFAULT_IM_MS),
        ) {
            writeln!(tx, "error starting tof_front ranging").unwrap();
        }

        let tof_left: vl53l1x::VL53L1<I2C3, PA8, PC9> = match vl53l1x::VL53L1::new(&mut i2c3, 0x69)
        {
            Ok(val) => val,
            Err(_) => {
                writeln!(tx, "tof_left initialization failed\r").unwrap();
                panic!("tof_left initialization failed");
            }
        };
        if let Err(_) = tof_left.start_ranging(
            &mut i2c3,
            Some(DEFAULT_DM),
            Some(DEFAULT_TB),
            Some(DEFAULT_IM_MS),
        ) {
            writeln!(tx, "error starting tof_left ranging").unwrap();
        }
        let front_dist: Option<u16> = None;
        let left_dist: Option<u16> = None;
        writeln!(tx, "system initialized\r").unwrap();
        print_tof_data::spawn_after(Duration::<u64, 1, 1000>::secs(1)).unwrap();
        (
            Shared {},
            Local {
                i2c2,
                i2c3,
                tx,
                tof_front,
                tof_left,
                front_dist,
                left_dist,
            },
            init::Monotonics(mono),
        )
    }

    #[task(local=[i2c2, i2c3, tx, tof_front, tof_left, front_dist, left_dist], shared=[])]
    fn print_tof_data(cx: print_tof_data::Context) {
        if let Err(_) = cx.local.tof_front.clear_interrupt(cx.local.i2c2) {
            writeln!(cx.local.tx, "error clearing tof_front interrupt").unwrap();
        };
        if let Err(_) = cx.local.tof_left.clear_interrupt(cx.local.i2c3) {
            writeln!(cx.local.tx, "error clearing tof_left interrupt").unwrap();
        };
        if cx
            .local
            .tof_front
            .check_for_data_ready(cx.local.i2c2)
            .unwrap_or(false)
        {
            // read ToF
            match cx.local.tof_front.get_distance(cx.local.i2c2) {
                Ok(distance) => {
                    if matches!(cx.local.tof_front.distance_valid(cx.local.i2c2), Ok(true)) {
                        *cx.local.front_dist = Some(distance);
                    }
                }
                Err(_e) => {}
            }
            if let Err(_) = cx.local.tof_front.clear_interrupt(cx.local.i2c2) {
                writeln!(cx.local.tx, "error clearing tof_front interrupt").unwrap();
            };
        }
        if cx
            .local
            .tof_left
            .check_for_data_ready(cx.local.i2c3)
            .unwrap_or(false)
        {
            // read ToF
            match cx.local.tof_left.get_distance(cx.local.i2c3) {
                Ok(distance) => {
                    if matches!(cx.local.tof_left.distance_valid(cx.local.i2c3), Ok(true)) {
                        *cx.local.left_dist = Some(distance);
                    }
                }
                Err(_e) => {}
            }
            if let Err(_) = cx.local.tof_left.clear_interrupt(cx.local.i2c3) {
                writeln!(cx.local.tx, "error clearing tof_left interrupt").unwrap();
            };
        }
        if cx.local.front_dist.is_some() && cx.local.left_dist.is_some() {
            writeln!(
                cx.local.tx,
                "front_dist {} left_dist {}\r",
                cx.local.front_dist.unwrap(),
                cx.local.left_dist.unwrap()
            )
            .unwrap();
            *cx.local.front_dist = None;
            *cx.local.left_dist = None;
        }
        // run at 100 Hz
        print_tof_data::spawn_after(Duration::<u64, 1, 1000>::millis(10)).unwrap();
    }
}
