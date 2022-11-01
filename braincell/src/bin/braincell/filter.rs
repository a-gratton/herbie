use crate::app::filter_data;
use core::fmt::Write;
use rtic::Mutex;
use systick_monotonic::fugit::Duration;

pub fn filter_data(mut cx: filter_data::Context) {
    writeln!(cx.local.tx, "filter thread running\r").unwrap();
    cx.shared.tof_front_filter.lock(|tof_front_filter| {
        tof_front_filter.insert(10);
    });
    cx.shared.tof_left_filter.lock(|tof_left_filter| {
        tof_left_filter.insert(10);
    });
    // run at 100 Hz
    filter_data::spawn_after(Duration::<u64, 1, 1000>::millis(10)).unwrap();
}
