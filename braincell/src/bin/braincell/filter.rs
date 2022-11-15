use crate::app::{filter_data, monotonics};
use crate::config::sys_config;
use braincell::filtering::{ahrs::ahrs_filter::AHRSFilter, ahrs::ahrs_filter::ImuData, sma};
use core::{f32::consts::PI, fmt::Write};
use rtic::Mutex;
use systick_monotonic::fugit::Duration;

const DEG_TO_RAD: f32 = PI / 180.0;

pub struct ImuFilter<const SIZE: usize, FilterType: AHRSFilter> {
    accel_x: sma::SmaFilter<f32, SIZE>,
    accel_y: sma::SmaFilter<f32, SIZE>,
    pub accel_z: sma::SmaFilter<f32, SIZE>,
    gyro_x: sma::SmaFilter<f32, SIZE>,
    gyro_y: sma::SmaFilter<f32, SIZE>,
    gyro_z: sma::SmaFilter<f32, SIZE>,
    mag_x: sma::SmaFilter<f32, SIZE>,
    mag_y: sma::SmaFilter<f32, SIZE>,
    mag_z: sma::SmaFilter<f32, SIZE>,
    ahrs_filter: FilterType,
    gyro_bias: (f32, f32, f32),
}

impl<const SIZE: usize, FilterType: AHRSFilter> ImuFilter<SIZE, FilterType> {
    // gyro bias of (x,y,z) axes in deg/s
    pub fn new(ahrs_filter: FilterType, gyro_bias: (f32, f32, f32)) -> Self {
        Self {
            accel_x: sma::SmaFilter::<f32, SIZE>::new(),
            accel_y: sma::SmaFilter::<f32, SIZE>::new(),
            accel_z: sma::SmaFilter::<f32, SIZE>::new(),
            gyro_x: sma::SmaFilter::<f32, SIZE>::new(),
            gyro_y: sma::SmaFilter::<f32, SIZE>::new(),
            gyro_z: sma::SmaFilter::<f32, SIZE>::new(),
            mag_x: sma::SmaFilter::<f32, SIZE>::new(),
            mag_y: sma::SmaFilter::<f32, SIZE>::new(),
            mag_z: sma::SmaFilter::<f32, SIZE>::new(),
            ahrs_filter: ahrs_filter,
            gyro_bias: gyro_bias,
        }
    }

    pub fn update(&mut self, imu_data: ImuData, deltat: f32) {
        self.accel_x.insert(imu_data.accel.0);
        self.accel_y.insert(imu_data.accel.1);
        self.accel_z.insert(imu_data.accel.2);
        self.gyro_x.insert(imu_data.gyro.0);
        self.gyro_y.insert(imu_data.gyro.1);
        self.gyro_z.insert(imu_data.gyro.2);
        self.mag_x.insert(imu_data.mag.0);
        self.mag_y.insert(imu_data.mag.1);
        self.mag_z.insert(imu_data.mag.2);

        if let Some(_) = self.accel_x.filtered() {
            self.ahrs_filter.update(
                ImuData {
                    accel: (
                        self.accel_x.filtered().unwrap_or(1.0),
                        self.accel_y.filtered().unwrap_or(1.0),
                        self.accel_z.filtered().unwrap_or(1.0),
                    ),
                    gyro: (
                        (self.gyro_x.filtered().unwrap_or(1.0) - self.gyro_bias.0) * DEG_TO_RAD,
                        (self.gyro_y.filtered().unwrap_or(1.0) - self.gyro_bias.1) * DEG_TO_RAD,
                        (self.gyro_z.filtered().unwrap_or(1.0) - self.gyro_bias.2) * DEG_TO_RAD,
                    ),
                    mag: (
                        self.mag_x.filtered().unwrap_or(1.0),
                        self.mag_y.filtered().unwrap_or(1.0),
                        self.mag_z.filtered().unwrap_or(1.0),
                    ),
                },
                deltat,
            );
        }
    }

    // returns (roll, pitch, yaw) in degrees
    pub fn filtered(&mut self) -> Option<(f32, f32, f32)> {
        match self.accel_x.filtered() {
            Some(_) => Some(self.ahrs_filter.get_euler_angles()),
            None => None,
        }
    }

    pub fn reset(&mut self) {
        self.ahrs_filter.reset();
    }
}

pub fn filter_data(mut cx: filter_data::Context) {
    let task_start_ticks: u64 = monotonics::now().ticks();
    let deltat: f32 =
        (task_start_ticks - *cx.local.filter_data_prev_ticks) as f32 * sys_config::SECONDS_PER_TICK;
    *cx.local.filter_data_prev_ticks = task_start_ticks;

    // read ToF sensors
    if cx
        .local
        .tof_front
        .check_for_data_ready(cx.local.i2c2)
        .unwrap_or(false)
    {
        match cx.local.tof_front.get_distance(cx.local.i2c2) {
            Ok(distance) => {
                if matches!(cx.local.tof_front.distance_valid(cx.local.i2c2), Ok(true)) {
                    cx.shared.tof_front_filter.lock(|tof_front_filter| {
                        tof_front_filter.insert(distance as i32);
                    });
                }
            }
            Err(_e) => {}
        }
    }
    if matches!(cx.local.tof_front.clear_interrupt(cx.local.i2c2), Err(_)) {
        if matches!(cx.local.tof_front.software_reset(cx.local.i2c2), Err(_)) {
            cx.shared.tx.lock(|tx| {
                writeln!(tx, "tof front software reset failed\r").unwrap();
            });
        }
        cx.shared.tx.lock(|tx| {
            writeln!(tx, "tof front clear interrupt failed\r").unwrap();
        });
    }

    if cx
        .local
        .tof_left
        .check_for_data_ready(cx.local.i2c3)
        .unwrap_or(false)
    {
        match cx.local.tof_left.get_distance(cx.local.i2c3) {
            Ok(distance) => {
                if matches!(cx.local.tof_left.distance_valid(cx.local.i2c3), Ok(true)) {
                    cx.shared.tof_left_filter.lock(|tof_left_filter| {
                        tof_left_filter.insert(distance as i32);
                    });
                }
            }
            Err(_e) => {}
        }
    }
    if matches!(cx.local.tof_left.clear_interrupt(cx.local.i2c3), Err(_)) {
        if matches!(cx.local.tof_left.software_reset(cx.local.i2c3), Err(_)) {
            cx.shared.tx.lock(|tx| {
                writeln!(tx, "tof left software reset failed\r").unwrap();
            });
        }
        cx.shared.tx.lock(|tx| {
            writeln!(tx, "tof left clear interrupt failed\r").unwrap();
        });
    }

    // read IMU
    if cx.local.imu.data_ready().unwrap_or(false) {
        if let Ok(_) = cx.local.imu.read_data() {
            // update IMU filter
            cx.shared.imu_filter.lock(|imu_filter| {
                imu_filter.update(cx.local.imu.get_data(), deltat);
            });
        }
    }

    // run at 100 Hz
    filter_data::spawn_after(Duration::<u64, 1, 1000>::millis(10)).unwrap();
}
