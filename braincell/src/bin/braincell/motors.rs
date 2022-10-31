use crate::app::speed_control;
use rtic::Mutex;
use systick_monotonic::fugit::Duration;

pub fn speed_control(mut cx: speed_control::Context) {
    cx.shared
        .motor_setpoints
        .lock(|motor_setpoints| cx.local.motors.set_speed_targets(motor_setpoints));
    cx.shared.motor_velocities.lock(|motor_velocities| {
        cx.local.motors.step(motor_velocities);
    });
    // run at 1 kHz
    speed_control::spawn_after(Duration::<u64, 1, 1000>::millis(1)).unwrap();
}
