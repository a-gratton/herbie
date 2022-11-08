use crate::app::{monotonics, speed_control};
use crate::config::sys_config;
use braincell::controller::motor::VelocityMeasurement;
use rtic::Mutex;
use systick_monotonic::fugit::Duration;

pub fn speed_control(mut cx: speed_control::Context) {
    cx.shared
        .motor_setpoints
        .lock(|motor_setpoints| cx.local.motors.set_speed_targets(motor_setpoints));
    let curtime = monotonics::now().ticks() as f32 * sys_config::SECONDS_PER_TICK;
    let vels = VelocityMeasurement {
        f_left: cx.local.encoder_f_left.get_speed(curtime),
        r_left: cx.local.encoder_r_left.get_speed(curtime),
        f_right: cx.local.encoder_f_right.get_speed(curtime),
        r_right: cx.local.encoder_r_right.get_speed(curtime),
    };
    cx.local.motors.step(&vels);
    // run at 1 kHz
    speed_control::spawn_after(Duration::<u64, 1, 1000>::millis(1)).unwrap();
}
