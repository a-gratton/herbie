use embedded_hal::PwmPin;

pub struct MDD3A<PWM1, PWM2> {
    pwm: (PWM1, PWM2),
}

impl<PWM1, PWM2> MDD3A<PWM1, PWM2>
where
    PWM1: embedded_hal::PwmPin + PwmPin<Duty = u16>,
    PWM2: embedded_hal::PwmPin + PwmPin<Duty = u16>,
{
    pub fn new(tuple_pwm: (PWM1, PWM2)) -> Self {
        Self {
            pwm: tuple_pwm
        }
    }

    pub fn set_power(&mut self, speed: f32) {
        let (speedx, speedy) = self.convert_pidout_to_power(speed);
        self.pwm.0.set_duty(speedx);
        self.pwm.1.set_duty(speedy);
    }

    pub fn start(&mut self) {
        self.set_power(0.0);
        self.pwm.0.enable();
        self.pwm.1.enable();
    }

    fn get_max_duty(&mut self) -> (u16, u16) {
        let x = self.pwm.0.get_max_duty();
        let y = self.pwm.1.get_max_duty();
        return (x, y);
    }

    /*
    Helper Function
    input: -100 < float < 100
    out: 0 < tuple: u16 < max_duty //TODO: FIGURE OUT WHY MAX_DUTY CHANGES SOMETIMES
    Decription: converts float to tuple (_,0) or (0,_) depending sign of float
    out is to be used as the parameter in set_power
    */
    fn convert_pidout_to_power(&mut self, mut f: f32) -> (u16, u16) {
        let mut positive_power = true;

        if f < 0.0 {
            positive_power = false;
            f = -1.0 * f;
        }

        let (md1, md2) = self.get_max_duty();
        if positive_power {
            let duty = (f * (md1 as f32) / 100.0) as u16; //truncate
            return (duty, 0);
        } else {
            let duty = (f * (md2 as f32) / 100.0) as u16; //truncate
            return (0, duty);
        }
    }
}
