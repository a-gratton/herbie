// Uart serial printf example
// See: https://dev.to/apollolabsbin/stm32f4-embedded-rust-at-the-hal-uart-serial-communication-1oc8

#![no_main]
#![no_std]

// Halt on panic
use panic_halt as _; // panic handler

use cortex_m_rt::entry;
use core::fmt::Write;
use stm32f4xx_hal as hal;

use crate::hal::{
    pac,
    prelude::*,
    serial::{Config, Serial},
};

#[entry]
fn main() -> ! {
    if let (Some(dp), Some(cp)) = (
        pac::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        let gpioa = dp.GPIOA.split();

        // Set up the system clock. We want to run at 48MHz for this one.
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

        // Create a delay abstraction based on SysTick
        let mut delay = cp.SYST.delay(&clocks);

        // Set up uart tx
        let tx_pin = gpioa.pa2.into_alternate();
        let mut tx = Serial::tx(
            dp.USART2,
            tx_pin,
            Config::default()
                .baudrate(115200.bps())
                .wordlength_8()
                .parity_none(),
            &clocks,
        )
        .unwrap();

        writeln!(tx, "Hello, world\r").unwrap();

        for n in 1.. {
            writeln!(tx, "Count: {}\r", n).unwrap();
            delay.delay_ms(1000_u32);
        }
    }

    loop {}
}
