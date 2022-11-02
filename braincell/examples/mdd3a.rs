/*
#![no_main]
#![no_std]

// Halt on panic
use panic_halt as _;

use cortex_m_rt::entry;
use stm32f4xx_hal::{pac, prelude::*};
use braincell::drivers::motor::mdd3a;

#[entry]
fn main() -> ! {
    if let Some(dp) = pac::Peripherals::take() {
        // Set up the system clock.
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

        let gpioa = dp.GPIOA.split();
        let channels = (gpioa.pa8.into_alternate(), gpioa.pa9.into_alternate());

        let pwm = dp.TIM1.pwm_hz(channels, 20.kHz(), &clocks).split();
        let (mut ch1, _ch2) = pwm;
        
        let mut motor = mdd3a::MDD3A::new(ch1, ch2);
        motor.init(mdd3a::Direction::Stop, 90);
        
    }

    loop {
        cortex_m::asm::nop();
    }
}
*/

#![deny(unsafe_code)]
//#![allow(dead_code)]
#![no_main]
#![no_std]

use braincell::drivers::motor::*;
// Halt on panic
use panic_halt as _;

use cortex_m_rt::entry;
use stm32f4xx_hal::pac::{self, TIM1};
use stm32f4xx_hal::timer::Channel;
use stm32f4xx_hal::{
    gpio::{Alternate, Output, Pin, PushPull, PB3, PB4, PB5, PB8},
    prelude::*,
    timer::{pwm, Instance},
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
        
        //let smth = dp.TIM1;
        
        //let gpiosmth = gpioa; //if I pass ownership of this, then ill own all of gpioa -> not good
        //let gpiosmth = dp.GPIOA.split();
        let channels1 = (gpioa.pa8.into_alternate(), gpioa.pa9.into_alternate()); //D7 1/1 D8 1/2 
        //let channels2 = (gpiob.pb10.into_alternate(), gpioa.pa3.into_alternate()); //D6 2/3 D0 2/4
        //let channels3 = (gpioa.pa6.into_alternate(), gpioc.pc7.into_alternate()); //d12 3/1 D9 3/2
        //let channels41 = gpioa.pa0.into_alternate(); //A0 5/1
        //let channels42 = gpiob.pb6.into_alternate(); //D10 4/1
        //let channels11 = gpioa.pa8.into_alternate();
        //let channels12 = gpioa.pa9.into_alternate();
        //let channelx = channels41; //pass ownership of this into mdd3a driver


        let pwm1 = dp.TIM1.pwm_hz(channels1, 20.kHz(), &clocks).split();
        //let pwm2 = dp.TIM2.pwm_hz(channels2, 20.kHz(), &clocks).split();
        //let pwm3 = dp.TIM3.pwm_hz(channels3, 20.kHz(), &clocks).split();
        //let mut pwm41 = dp.TIM5.pwm_hz(channels41, 20.kHz(), &clocks);
        //let mut pwm42 = dp.TIM4.pwm_hz(channels42, 20.kHz(), &clocks);
        //let mut pwm11 = dp.TIM1.pwm_hz((channels11,channels12), 20.kHz(), &clocks).split();
        let mut test = mdd3a::MDD3A::new(pwm1);
        let power= test.set_power(25.0);
        test.change(power);
        test.start();


        //let (mut pwm1ch1, mut pwm1ch2) = pwm1;
        //let (mut pwm2ch1, mut pwm2ch2) = pwm2;
        //let (mut pwm3ch1, mut pwm3ch2) = pwm3;

        //let nu = 16;
        //let mut test = mdd3a::MDD3A::new(pwm1ch1);
        //test.change(16);
        //let max_dutypwm1ch1 = pwm1ch1.get_max_duty();
        //let max_dutypwm1ch2 = pwm1ch2.get_max_duty();
        //let max_dutypwm2ch1 = pwm2ch1.get_max_duty();
        //let max_dutypwm2ch2 = pwm2ch2.get_max_duty();
        //let max_dutypwm3ch1 = pwm3ch1.get_max_duty();
        //let max_dutypwm3ch2 = pwm3ch2.get_max_duty();
        //let max_dutypwm4ch1 = pwm41.get_max_duty();
        //let max_dutypwm4ch2 = pwm42.get_max_duty();

        //pwm1ch1.set_duty(max_dutypwm1ch1); //D7 pa8
        //pwm1ch2.set_duty(0);
        //pwm2ch1.set_duty(max_dutypwm2ch1); //D6 pb10
        //pwm2ch2.set_duty(0);
        //pwm3ch1.set_duty(max_dutypwm3ch1); //D12 pa6
        //pwm3ch2.set_duty(0);
        //pwm41.set_duty(Channel::C1, 0);
        //pwm42.set_duty(Channel::C1, max_dutypwm4ch2); //D10 Pb6

        //pwm1ch1.enable();
        //pwm1ch2.enable();
        //pwm2ch1.enable();
        //pwm2ch2.enable();
        //pwm3ch1.enable();
        //pwm3ch2.enable();
        //pwm41.enable(Channel::C1);
        //pwm42.enable(Channel::C1);
        
        
        //let motor1 = mdd3a::MDD3A::new(dp.TIM1);


    }

    loop {
        cortex_m::asm::nop();
    }
}