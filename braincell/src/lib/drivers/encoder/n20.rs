use crate::drivers::encoder::n20_constants::*;
use embedded_hal::Direction as RotaryDirection;
use stm32f4xx_hal::prelude::_embedded_hal_Qei;

pub struct N20<QEI> {
    encoder: QEI,
    pastsec: f32,
    pastcount: u64,
}

impl<QEI> N20<QEI>
where
    QEI: _embedded_hal_Qei,
    u32: From<<QEI as _embedded_hal_Qei>::Count>,
{
    pub fn new(qei: QEI) -> Self {
        Self {
            encoder: qei,
            pastsec: 0.0,
            pastcount: 0,
        }
    }
    pub fn get_speed(&mut self, currsec: f32) -> f32 {
        //get current count on encoder
        let count = self.encoder.count();

        //convert Qei::Count to u64
        let currcount = <<QEI as _embedded_hal_Qei>::Count as Into<u32>>::into(count) as u64;

        //obtain the difference in counts while taking into account overflow of 16bit and 32bit timers
        let mut countdiff: u64;
        if currcount > self.pastcount {
            countdiff = currcount - self.pastcount;
            if countdiff > OVERFLOW_TH_32BIT {
                let temp = self.pastcount + (u32::MAX) as u64;
                countdiff = temp - currcount;
            } else if countdiff > OVERFLOW_TH_16BIT {
                let temp = self.pastcount + (u16::MAX) as u64;
                countdiff = temp - currcount;
            }
        } else {
            countdiff = self.pastcount - currcount;
            if countdiff > OVERFLOW_TH_32BIT {
                let temp = currcount + (u32::MAX) as u64;
                countdiff = temp - self.pastcount;
            } else if countdiff > OVERFLOW_TH_16BIT {
                let temp = currcount + (u16::MAX) as u64;
                countdiff = temp - self.pastcount;
            }
        }

        let degreeselapsed = countdiff as f32 / COUNTS_PER_REVOLUTION * 360.0;
        let timediff = currsec - self.pastsec;

        let degreespersecond = degreeselapsed / timediff;

        self.pastsec = currsec;
        self.pastcount = currcount;

        //use direction specified in encoder to determine whether pos or neg deg/s
        if self.encoder.direction() == RotaryDirection::Downcounting {
            return degreespersecond * -1.0;
        }

        return degreespersecond;
    }
}
