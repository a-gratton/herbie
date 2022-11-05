#![deny(unsafe_code)]
#![no_main]
#![no_std]

// Demonstrates the use of a rotary encoder. This example was tested
// on a "black pill" USB C board:
// https://stm32-base.org/boards/STM32F411CEU6-WeAct-Black-Pill-V2.0
//
// The rotary encoder A and B pins are connected to pins A0 and A1,
// and they each have a 10K ohm pull-up resistor.

use braincell::drivers::encoder;
// Halt on panic
use panic_halt as _;

use cortex_m_rt::entry;
use embedded_hal::Direction as RotaryDirection;
use stm32f4xx_hal::{pac, prelude::*, qei::Qei};

use stm32f4xx_hal::{
    gpio::{Alternate, Output, Pin, PushPull, PB3, PB4, PB5},
    pac::{SPI1, USART2},
    prelude::*,
    serial::{Config, Serial, Tx},
    spi::{Mode, Phase, Polarity, Spi},
};
use core::fmt::Write;

#[entry]
fn main() -> ! {
    let dp = pac::Peripherals::take().expect("Failed to get stm32 peripherals");
    let cp = cortex_m::peripheral::Peripherals::take().expect("Failed to get cortex_m peripherals");

    // Set up the system clock.
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.freeze();

    // Create a delay abstraction based on SysTick.
    let mut delay = cp.SYST.delay(&clocks);


    let gpioa = dp.GPIOA.split();

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

    // Connect a rotary encoder to pins A0 and A1.
    let rotary_encoder_pins = (gpioa.pa0.into_alternate(), gpioa.pa1.into_alternate());
    let rotary_encoder_timer = dp.TIM2;
    let rotary_encoder = Qei::new(rotary_encoder_timer, rotary_encoder_pins);
    let mut encoder = encoder::n20::QeiWrapper::new(rotary_encoder);

    loop {
        //let new_count = rotary_encoder.count();
        let new_count = encoder.get_speed();
        //writeln!(tx, "Count: {new_count}\r").unwrap();
        writeln!(tx, "Count: {new_count}\r").unwrap();

        delay.delay_ms(10_u32);
    }
}