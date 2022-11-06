use embedded_hal::PwmPin;

pub struct MDD3A<X, Y> {
    pwm: (X, Y),
}

impl<X, Y> MDD3A<X, Y>
where
    X: embedded_hal::PwmPin + PwmPin<Duty = u16>,
    Y: embedded_hal::PwmPin + PwmPin<Duty = u16>,
{
    pub fn new(in1: (X, Y)) -> Self {
        Self { pwm: in1 }
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

    fn get_max_duty(&mut self) -> (<X as PwmPin>::Duty, <Y as PwmPin>::Duty) {
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
