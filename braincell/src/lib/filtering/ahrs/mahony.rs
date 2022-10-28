// Mahony filter implementation
// Ported from https://github.com/drcpattison/ICM-20948/blob/master/src/AHRSAlgorithms.cpp

pub use crate::filtering::ahrs::ahrs_filter::*;
use core::f32::consts::PI;
use libm::{asinf, atan2f, sqrtf};

const RAD_TO_DEG: f32 = 180.0 / PI;
pub const DEFAULT_KP: f32 = 0.0;
pub const DEFAULT_KI: f32 = 0.0;

pub struct MahonyFilter {
    q: (f32, f32, f32, f32), // quaternion
    e: (f32, f32, f32),      // integral error
    kp: f32,                 // proportional gain
    ki: f32,                 // integral gain
}

impl MahonyFilter {
    pub fn new(kp: f32, ki: f32) -> Self {
        Self {
            q: (1.0, 0.0, 0.0, 0.0),
            e: (0.0, 0.0, 0.0),
            kp,
            ki,
        }
    }
}

impl AHRSFilter for MahonyFilter {
    fn update(&mut self, imu_data: ImuData, deltat: f32) {
        let mut q1 = self.q.0;
        let mut q2 = self.q.1;
        let mut q3 = self.q.2;
        let mut q4 = self.q.3;

        let mut ax = imu_data.accel.0;
        let mut ay = imu_data.accel.1;
        let mut az = imu_data.accel.2;
        let mut gx = imu_data.gyro.0;
        let mut gy = imu_data.gyro.1;
        let mut gz = imu_data.gyro.2;
        let mut mx = imu_data.mag.0;
        let mut my = imu_data.mag.1;
        let mut mz = imu_data.mag.2;

        // Auxiliary variables to avoid repeated arithmetic
        let q1q1: f32 = q1 * q1;
        let q1q2: f32 = q1 * q2;
        let q1q3: f32 = q1 * q3;
        let q1q4: f32 = q1 * q4;
        let q2q2: f32 = q2 * q2;
        let q2q3: f32 = q2 * q3;
        let q2q4: f32 = q2 * q4;
        let q3q3: f32 = q3 * q3;
        let q3q4: f32 = q3 * q4;
        let q4q4: f32 = q4 * q4;

        // Normalize accelerometer measurement
        let mut norm: f32 = 1.0 / sqrtf(ax * ax + ay * ay + az * az);
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Normalize magnetometer measurement
        norm = 1.0 / sqrtf(mx * mx + my * my + mz * mz);
        mx *= norm;
        my *= norm;
        mz *= norm;

        // Reference direction of Earth's magnetic field
        let hx =
            2.0 * mx * (0.5 - q3q3 - q4q4) + 2.0 * my * (q2q3 - q1q4) + 2.0 * mz * (q2q4 + q1q3);
        let hy =
            2.0 * mx * (q2q3 + q1q4) + 2.0 * my * (0.5 - q2q2 - q4q4) + 2.0 * mz * (q3q4 - q1q2);
        let bx = sqrtf(hx * hx + hy * hy);
        let bz =
            2.0 * mx * (q2q4 - q1q3) + 2.0 * my * (q3q4 + q1q2) + 2.0 * mz * (0.5 - q2q2 - q3q3);

        // Estimated direction of gravity and magnetic field
        let vx = 2.0 * (q2q4 - q1q3);
        let vy = 2.0 * (q1q2 + q3q4);
        let vz = q1q1 - q2q2 - q3q3 + q4q4;
        let wx = 2.0 * bx * (0.5 - q3q3 - q4q4) + 2.0 * bz * (q2q4 - q1q3);
        let wy = 2.0 * bx * (q2q3 - q1q4) + 2.0 * bz * (q1q2 + q3q4);
        let wz = 2.0 * bx * (q1q3 + q2q4) + 2.0 * bz * (0.5 - q2q2 - q3q3);

        // Error is cross product between estimated direction and measured direction of gravity
        let ex = (ay * vz - az * vy) + (my * wz - mz * wy);
        let ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
        let ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
        if self.ki > 0.0 {
            self.e.0 += ex; // accumulate integral error
            self.e.1 += ey;
            self.e.2 += ez;
        } else {
            self.e.0 = 0.0; // prevent integral wind up
            self.e.1 = 0.0;
            self.e.2 = 0.0;
        }

        // Apply feedback terms
        gx = gx + self.kp * ex + self.ki * self.e.0;
        gy = gy + self.kp * ey + self.ki * self.e.1;
        gz = gz + self.kp * ez + self.ki * self.e.2;

        // Integrate rate of change of quaternion
        let pa = q2;
        let pb = q3;
        let pc = q4;
        q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5 * deltat);
        q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5 * deltat);
        q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5 * deltat);
        q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5 * deltat);

        // Normalize quaternion
        norm = 1.0 / sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
        self.q.0 = q1 * norm;
        self.q.1 = q2 * norm;
        self.q.2 = q3 * norm;
        self.q.3 = q4 * norm;
    }

    fn get_euler_angles(&self) -> (f32, f32, f32) {
        let roll: f32 = atan2f(
            2.0 * (self.q.0 * self.q.1 + self.q.2 * self.q.3),
            self.q.0 * self.q.0 - self.q.1 * self.q.1 - self.q.2 * self.q.2 + self.q.3 * self.q.3,
        );
        let pitch: f32 = -asinf(2.0 * (self.q.1 * self.q.3 - self.q.0 * self.q.2));
        let yaw: f32 = atan2f(
            self.q.1 * self.q.2 + self.q.0 * self.q.3,
            self.q.0 * self.q.0 + self.q.1 * self.q.1 - self.q.2 * self.q.2 - self.q.3 * self.q.3,
        );
        (roll * RAD_TO_DEG, pitch * RAD_TO_DEG, yaw * RAD_TO_DEG)
    }

    fn reset(&mut self) {
        self.q = (1.0, 0.0, 0.0, 0.0);
        self.e = (0.0, 0.0, 0.0);
    }
}
