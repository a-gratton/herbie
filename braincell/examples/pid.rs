// Uart serial printf example
// See: https://dev.to/apollolabsbin/stm32f4-embedded-rust-at-the-hal-uart-serial-communication-1oc8

#![no_main]
#![no_std]

// Halt on panic
use panic_write::PanicHandler; // panic handler

use core::fmt::Write;
use cortex_m_rt::entry;
use stm32f4xx_hal as hal;

use crate::hal::{
    pac,
    prelude::*,
    serial::{Config, Serial},
};

use pid::Pid;

#[entry]
fn main() -> ! {
    if let (Some(dp), Some(cp)) = (
        pac::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        let gpioa = dp.GPIOA.split();

        // Set up the system clock. We want to run at 48MHz for this one.
        let rcc = dp.RCC.constrain();
        let clocks = rcc.cfgr.sysclk(48.MHz()).freeze();

        // Create a delay abstraction based on SysTick
        let _delay = cp.SYST.delay(&clocks);

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

        // example from the README at https://crates.io/crates/pid
        // Set only kp (proportional) to 10. The setpoint is 15.
        // Set limits for P, I, and D to 100 each.
        let mut pid: Pid<f32> = Pid::new(10.0, 0.0, 0.0, 100.0, 100.0, 100.0, 100.0, 15.0);

        // Fake a measurement of 10.0, which is an error of 5.0.
        let output = pid.next_control_output(10.0);
        // Verify that kp * error = 10.0 * 5.0 = 50.0
        assert_eq!(output.output, 50.0);
        // Verify that all output was from the proportional term
        assert_eq!(output.p, 50.0);
        assert_eq!(output.i, 0.0);
        assert_eq!(output.d, 0.0);

        // Verify that the same measurement produces the same output since we
        // aren't using the stateful derivative & integral terms.
        let output = pid.next_control_output(10.0);
        assert_eq!(output.p, 50.0);

        // Add an integral term
        pid.ki = 1.0;
        let output = pid.next_control_output(10.0);
        assert_eq!(output.p, 50.0);
        // Verify that the integral term is adding to the output signal.
        assert_eq!(output.i, 5.0);
        assert_eq!(output.output, 55.0);

        // Add a derivative term
        pid.kd = 2.0;
        let output = pid.next_control_output(15.0); // Match the desired target
                                                    // No proportional term since no error
        assert_eq!(output.p, 0.0);
        // Integral term stays the same
        assert_eq!(output.i, 5.0);
        // Derivative on measurement produces opposing signal
        assert_eq!(output.d, -10.0);
        assert_eq!(output.output, -5.0);

        writeln!(tx, "PID tests pass").unwrap();

        loop {}
    }

    panic!("peripheral acquisiton failed");
}
