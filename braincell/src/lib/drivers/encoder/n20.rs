use stm32f4xx_hal::{
    prelude::*,
};
use embedded_hal::Direction as RotaryDirection;
use stm32f4xx_hal::prelude::_embedded_hal_Qei;
//use stm32f4xx_hal::{pac, prelude::*, qei::Qei};

pub struct QeiWrapper <X> {
    encoder: X
}

impl<X> QeiWrapper<X> 
where
X: _embedded_hal_Qei
{
    pub fn new(qei: X) -> Self {
        Self {
            encoder: qei,
        }
    }
     pub fn get_speed(&mut self) -> <X as _embedded_hal_Qei>::Count {
        let mut fw = true;
        let count = self.encoder.count();
        match self.encoder.direction() {
            RotaryDirection::Upcounting => fw = true,
            RotaryDirection::Downcounting => fw = false,
        }
        return count;

     }
}