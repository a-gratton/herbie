#![no_main]
#![no_std]

use core::fmt::Write;
use cortex_m_rt::entry;
use panic_write::PanicHandler;
use stm32f4xx_hal as hal;

use crate::hal::{
    pac,
    prelude::*,
    serial::{Config, Serial},
};

use braincell::filtering::sma;

#[entry]
fn main() -> ! {
    if let (Some(dp), Some(_cp)) = (
        pac::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        let gpioa = dp.GPIOA.split();

        // Set up the system clock. We want to run at 48MHz for this one.
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

        // Set up uart tx
        let tx_pin = gpioa.pa2.into_alternate();
        let serial = Serial::tx(
            dp.USART2,
            tx_pin,
            Config::default()
                .baudrate(115200.bps())
                .wordlength_8()
                .parity_none(),
            &clocks,
        )
        .unwrap();

        let mut tx = PanicHandler::new(serial);
        writeln!(tx, "Hello, world\r").unwrap();
        let mut int_filter = sma::SmaFilter::<i32, 5>::new();
        let mut float_filter = sma::SmaFilter::<f32, 5>::new();
        for _ in 0..5 {
            assert_eq!(
                int_filter.filtered(),
                None,
                "\rint filter is full but shouldn't be"
            );
            int_filter.insert(1);
            assert_eq!(
                float_filter.filtered(),
                None,
                "\rfloat filter is full but shouldn't be"
            );
            float_filter.insert(1.0);
        }

        assert!(int_filter.filtered() != None, "\rint filter output is None");
        let i = int_filter.filtered().unwrap();
        assert_eq!(i, 1, "\rint filter value {i} != expected 1");

        assert!(
            float_filter.filtered() != None,
            "\rfloat filter output is None"
        );
        let f = float_filter.filtered().unwrap();
        assert_eq!(f, 1.0, "\rfloat filter value {f} != expected 1.0");

        int_filter.insert(10);
        let i = int_filter.filtered().unwrap();
        assert_eq!(i, 2, "\rint filter value {i} != expected 2");

        float_filter.insert(10.0);
        let f = float_filter.filtered().unwrap();
        assert_eq!(f, 2.8, "\rfloat filter value {f} != expected 2.8");

        int_filter.reset();
        float_filter.reset();

        for _ in 0..5 {
            assert_eq!(
                int_filter.filtered(),
                None,
                "\rint filter is full but shouldn't be"
            );
            int_filter.insert(2);
            assert_eq!(
                float_filter.filtered(),
                None,
                "\rfloat filter is full but shouldn't be"
            );
            float_filter.insert(2.0);
        }

        writeln!(tx, "sma filter tests passed\r").unwrap();
        loop {}
    }

    panic!("\rperipheral acquisition failed")
}
