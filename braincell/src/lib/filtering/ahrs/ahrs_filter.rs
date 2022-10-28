pub struct ImuData {
    pub accel: (f32, f32, f32), // (ax, ay, az)
    pub gyro: (f32, f32, f32),  // (gx, gy, gz)
    pub mag: (f32, f32, f32),   // (mx, my, mz)
}

pub trait AHRSFilter {
    // accel in any unit
    // gyro in rad/s
    // mag in any unit
    // deltat (time delta between update calls) in seconds
    fn update(&mut self, imu_data: ImuData, deltat: f32);

    // (roll, pitch, yaw) in degrees
    fn get_euler_angles(&self) -> (f32, f32, f32);

    // reset the filter to initial conditions
    fn reset(&mut self);
}
