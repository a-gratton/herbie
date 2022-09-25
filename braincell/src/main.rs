#![no_main]
#![no_std]

// Halt on panic
use panic_halt as _; // panic handler

use cortex_m::asm;
use cortex_m_rt::entry;
use stm32f4xx_hal as hal;


#[entry]
fn main() -> ! {
    loop {
        asm::nop();
    }
}