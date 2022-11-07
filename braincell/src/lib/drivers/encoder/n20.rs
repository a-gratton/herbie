use stm32f4xx_hal::{
    prelude::*,
};
use embedded_hal::Direction as RotaryDirection;
use stm32f4xx_hal::prelude::_embedded_hal_Qei;
//use stm32f4xx_hal::{pac, prelude::*, qei::Qei};

pub struct N20 <X> {
    encoder: X,
    pasttick: u64,
    pastcount: u32,
}

impl<X> N20<X> 
where
X: _embedded_hal_Qei, u32: From<<X as _embedded_hal_Qei>::Count>
{
    pub fn new(qei: X) -> Self {
        Self {
            encoder: qei,
            pasttick: 0,
            pastcount: 0,
        }
    }
     pub fn get_speed(&mut self, currtick: u64) -> f32 {
        let timediff = ((currtick - self.pasttick) as f32) * 0.001;
        self.pasttick = currtick;


        let count = self.encoder.count();
        let mut currcount = <<X as _embedded_hal_Qei>::Count as Into<u32>>::into(count); //need to take into account u16 overflow

        let mut countdiff:u32 = 0;
        /*
        if currcount - self.pastcount > 32000 {
            if currcount > self.pastcount {
                let smth = self.pastcount + 65536;
                countdiff = smth - currcount;
            } else {
                let smth = currcount + 65536;
                countdiff = smth - self.pastcount;
            }
        }
        self.pastcount = currcount;
        */
        countdiff = currcount - self.pastcount;
        self.pastcount = currcount;

        let countselapsed = (countdiff as f32);
        let degreeselapsed = countselapsed / 1400.0 * 360.0;
        let degreespersecond = degreeselapsed/timediff;

        return degreespersecond;

     }
}