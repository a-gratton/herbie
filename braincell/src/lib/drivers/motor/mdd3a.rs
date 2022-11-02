/*
use stm32f4xx_hal::timer::PwmChannel;
use stm32f4xx_hal::pac::TIM1;
use stm32f4xx_hal::timer::Channel;
// use crate::drivers::motor::mdd3a_constants::*;
//use stm32f4xx_hal::{prelude::*};

//ill probably need a struct for timer and pin
pub enum Direction {
    Stop = 0,
    Backward = 1,
    Forward = 2
}

struct PWMWrapper{
    fw: PwmChannel<TIM1, {Channel::C1}>,
    bw: PwmChannel<TIM1, {Channel::C2}>,
    fw_maxduty: u16,
    bw_maxduty: u16
}


impl PWMWrapper
{
    fn init (&mut self){
        self.set_maxduty();
        self.fw.set_duty(0);
        self.bw.set_duty(0);
        self.fw.enable();
        self.bw.enable();
    }

    fn set_maxduty (&mut self){
        self.fw_maxduty = self.fw.get_max_duty();
        self.bw_maxduty = self.bw.get_max_duty();

    }
}

pub struct MDD3A{
    control: PWMWrapper,
    dir: Direction,
    power: u8,

}

impl MDD3A
{
    pub fn new(ahead: PwmChannel, behind: PwmChannel) -> Self {
        Self {
            control: PWMWrapper{bw: behind, fw: ahead, fw_maxduty: 0, bw_maxduty:0},
            dir: Direction::Stop,
            power: 0
        }
    }
    
    pub fn init(
        &mut self, d: Direction, pwr: u8
    ){
        self.control.init();
        self.set_dir(d);
        self.set_power(pwr);
    }
    
    pub fn set_dir(&mut self, d: Direction) {
        self.dir = d;
    }

    pub fn set_power(&mut self, pwr: u8) {
        self.power = pwr;
        match self.dir{
            Direction::Forward => {
                self.control.bw.set_duty(0);
                self.control.fw.set_duty(self.control.fw_maxduty/10*9); //don't think this works
            }
            Direction::Backward => {
                self.control.fw.set_duty(0);
                self.control.bw.set_duty(self.control.bw_maxduty/10*9);
            }
            Direction::Stop => {
                self.control.fw.set_duty(0);
                self.control.bw.set_duty(0);
            }
        }
    }

    //unused fn get_power

    //unused fn get_dir
}
*/
/*
use stm32f4xx_hal::{timer::{pwm::{PwmExt}, Instance, Channel, pwm::Ch}, rcc::Clocks, time::Hertz};
use stm32f4xx_hal::timer::Pins;
//use stm32f4xx_hal::timer::sealed::WithPwm;

use core::marker::PhantomData;

pub struct MDD3A<PWM, PINFW, PINBW, TIM>  
{
    somepwm: PWM,
    pinfw: PINFW,
    pinbw:  PINBW,
    _tim: TIM,
}

impl<PWM, PINFW, PINBW, TIM>MDD3A<PWM, PINFW, PINBW, TIM>
where
TIM: Instance,
PWM: PwmExt,
PINFW: Pins<TIM, Ch<0>>,
PINBW: Pins<TIM, Ch<1>>,
{
    pub fn new(input: PWM, p1: PINFW, p2: PINBW, t:TIM) -> Self {
        Self {
            somepwm: input,
            pinfw: p1,
            pinbw: p2,
            _tim: t,
        }
    }
    pub fn init(&mut self, time: Hertz, clocks: &Clocks) {
        let smth = self.somepwm.pwm_hz(self.pinfw,time,clocks).split();
    }

    pub fn set_power(&mut self, channel: Channel, duty: u16) {

    }
}

*/
/* 
use embedded_hal::PwmPin;
use f32;

pub struct MDD3A<X, Y>  
{
    x: X,
    y: Y,
    dir: bool,
}

impl<X,Y> MDD3A<X,Y> 
where
X: embedded_hal::PwmPin,
Y: embedded_hal::PwmPin,
{
    pub fn new(in1: X, in2: Y) -> Self {
        Self {
            x: in1,
            y: in2,
            dir: true,
        }
    }

    pub fn set_power(&mut self, mut power: f32) -> u16{

        if power < 0.0 {
            self.dir = false;
            power = -1.0 * power;
        }

        return (power*655.36) as u16;
        
    }

    pub fn change(&mut self, speedx: <X as PwmPin>::Duty, speedy: <Y as PwmPin>::Duty) {
        if self.dir == true {
            self.x.set_duty(speedx);
        } else {
            self.y.set_duty(speedy);
        }
    }

    pub fn start(&mut self) {
        self.x.enable();
        self.y.enable();
    }
}
*/

use embedded_hal::PwmPin;

pub struct MDD3A<X, Y>  
{
    pwm: (X,Y),
}

impl<X, Y> MDD3A<X, Y>
where
X: embedded_hal::PwmPin,
Y: embedded_hal::PwmPin
{
    pub fn new(in1: (X,Y)) -> Self {
        Self {
            pwm: in1,
        }
    }
    pub fn set_power(&mut self, mut power: f32) -> (u16,u16){
        let mut dir = true;
        if power < 0.0 {
            dir = false;
            power = -1.0 * power;
        }
        let duty = (power*655.36) as u16;

        if dir == true {
            return (duty, 0 as u16);
        } else {
            return (0 as u16,duty);
        }
        
    }
    pub fn change(&mut self, (speedx,speedy): (<X as PwmPin>::Duty, <Y as PwmPin>::Duty)) {
            self.pwm.0.set_duty(speedx);
            self.pwm.1.set_duty(speedy);
    }
    pub fn start(&mut self) {
        self.pwm.0.enable();
        self.pwm.1.enable();
    }

}


