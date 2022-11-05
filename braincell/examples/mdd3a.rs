#![deny(unsafe_code)]
//#![allow(dead_code)]
#![no_main]
#![no_std]

use braincell::drivers::motor::*;
// Halt on panic
use panic_halt as _;

use cortex_m_rt::entry;
use stm32f4xx_hal::pac::{self};
use stm32f4xx_hal::{
    prelude::*,
};


#[entry]
fn main() -> ! {
    if let Some(dp) = pac::Peripherals::take() {
        // Set up the system clock.
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.freeze();

        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();
        let gpioc = dp.GPIOC.split();
        
        let channels1 = (gpioa.pa8.into_alternate(), gpioa.pa9.into_alternate()); //D7 1/1 D8 1/2 
        let channels2 = (gpiob.pb10.into_alternate(), gpioa.pa3.into_alternate()); //D6 2/3 D0 2/4
        let channels3 = (gpioa.pa6.into_alternate(), gpioc.pc7.into_alternate()); //d12 3/1 D9 3/2
        let channels4 = (gpiob.pb6.into_alternate(), gpiob.pb7.into_alternate()); //D10 4/1 farleft on same line as lowgnd 4/2

        let pwm1 = dp.TIM1.pwm_hz(channels1, 20.kHz(), &clocks).split();
        let pwm2 = dp.TIM2.pwm_hz(channels2, 20.kHz(), &clocks).split();
        let pwm3 = dp.TIM3.pwm_hz(channels3, 20.kHz(), &clocks).split();
        let pwm4 = dp.TIM4.pwm_hz(channels4, 20.kHz(), &clocks).split();

        let mut motor1 = mdd3a::MDD3A::new(pwm1);
        let mut motor2 = mdd3a::MDD3A::new(pwm2);
        let mut motor3 = mdd3a::MDD3A::new(pwm3);
        let mut motor4 = mdd3a::MDD3A::new(pwm4);
        let power = mdd3a::convert_pidout_to_power(100.0);
        motor1.set_power(power);
        motor2.set_power(power);
        motor3.set_power(power);
        motor4.set_power(power);
        motor1.start();
        motor2.start();
        motor3.start();
        motor4.start();

    }

    loop {
        cortex_m::asm::nop();
    }
}