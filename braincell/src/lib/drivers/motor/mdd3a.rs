use embedded_hal::PwmPin;

/*
Helper Function
input: -100 < float < 100
out: 0 < tuple: u16 < max_duty //TODO: FIGURE OUT WHY MAX_DUTY CHANGES SOMETIMES
Decription: converts float to tuple (_,0) or (0,_) depending sign of float
out is to be used as the parameter in set_power
*/
pub fn convert_pidout_to_power(mut f: f32) -> (u16, u16) {
    let mut dir = true;

    if f < 0.0 {
        dir = false;
        f = -1.0 * f;
    }

    let mut duty = (f * 24.0) as u16; //truncate

    //even though it's okay to set duty greater than max_duty, just in case something bad happens
    if duty > 2400 {
        duty = 2400;
    }

    if dir == true {
        return (duty, 0);
    } else {
        return (0, duty);
    }
}

pub struct MDD3A<X, Y> {
    pwm: (X, Y),
}

impl<X, Y> MDD3A<X, Y>
where
    X: embedded_hal::PwmPin,
    Y: embedded_hal::PwmPin,
{
    pub fn new(in1: (X, Y)) -> Self {
        Self { pwm: in1 }
    }

    pub fn set_power(&mut self, (speedx, speedy): (<X as PwmPin>::Duty, <Y as PwmPin>::Duty)) {
        self.pwm.0.set_duty(speedx);
        self.pwm.1.set_duty(speedy);
    }

    pub fn get_duty(&mut self) -> (<X as PwmPin>::Duty, <Y as PwmPin>::Duty) {
        let x = self.pwm.0.get_max_duty();
        let y = self.pwm.1.get_max_duty();
        return (x, y);
    }

    pub fn start(&mut self) {
        self.pwm.0.enable();
        self.pwm.1.enable();
    }
}
