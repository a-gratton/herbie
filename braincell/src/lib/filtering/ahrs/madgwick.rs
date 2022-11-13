// Madgwick filter implementation
// Ported from https://github.com/drcpattison/ICM-20948/blob/master/src/AHRSAlgorithms.cpp

pub use crate::filtering::ahrs::ahrs_filter::*;
use core::f32::consts::PI;
use libm::{asinf, atan2f, sqrtf};

const RAD_TO_DEG: f32 = 180.0 / PI;
const DEG_TO_RAD: f32 = PI / 180.0;

pub struct MadgwickFilter {
    q: (f32, f32, f32, f32), // quaternion
    beta: f32,               // free parameter
}

impl MadgwickFilter {
    // gyro bias in deg/s
    pub fn new(gyro_bias: f32) -> Self {
        Self {
            q: (1.0, 0.0, 0.0, 0.0),
            beta: gyro_bias * DEG_TO_RAD * 0.866, // where 0.866 = sqrt(3/4),
        }
    }
}

impl AHRSFilter for MadgwickFilter {
    fn update(&mut self, imu_data: ImuData, deltat: f32) {
        let mut q1 = self.q.0;
        let mut q2 = self.q.1;
        let mut q3 = self.q.2;
        let mut q4 = self.q.3;

        let mut ax = imu_data.accel.0;
        let mut ay = imu_data.accel.1;
        let mut az = imu_data.accel.2;
        let gx = imu_data.gyro.0;
        let gy = imu_data.gyro.1;
        let gz = imu_data.gyro.2;
        let mut mx = imu_data.mag.0;
        let mut my = imu_data.mag.1;
        let mut mz = imu_data.mag.2;

        // Auxiliary variables to avoid repeated arithmetic
        let _2q1 = 2.0 * q1;
        let _2q2 = 2.0 * q2;
        let _2q3 = 2.0 * q3;
        let _2q4 = 2.0 * q4;
        let _2q1q3 = 2.0 * q1 * q3;
        let _2q3q4 = 2.0 * q3 * q4;
        let q1q1 = q1 * q1;
        let q1q2 = q1 * q2;
        let q1q3 = q1 * q3;
        let q1q4 = q1 * q4;
        let q2q2 = q2 * q2;
        let q2q3 = q2 * q3;
        let q2q4 = q2 * q4;
        let q3q3 = q3 * q3;
        let q3q4 = q3 * q4;
        let q4q4 = q4 * q4;

        // Normalize accel data
        let mut norm = sqrtf(ax * ax + ay * ay + az * az);
        if norm == 0.0 {
            return;
        }
        norm = 1.0 / norm;
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Normalize mag data
        norm = sqrtf(mx * mx + my * my + mz * mz);
        if norm == 0.0 {
            return;
        }
        norm = 1.0 / norm;
        mx *= norm;
        my *= norm;
        mz *= norm;

        // Reference direction of Earth's magnetic field
        let _2q1mx = 2.0 * q1 * mx;
        let _2q1my = 2.0 * q1 * my;
        let _2q1mz = 2.0 * q1 * mz;
        let _2q2mx = 2.0 * q2 * mx;
        let hx =
            mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4
                - mx * q3q3
                - mx * q4q4;
        let hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2
            + my * q3q3
            + _2q3 * mz * q4
            - my * q4q4;
        let _2bx = sqrtf(hx * hx + hy * hy);
        let _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2
            + _2q3 * my * q4
            - mz * q3q3
            + mz * q4q4;
        let _4bx = 2.0 * _2bx;
        let _4bz = 2.0 * _2bz;

        // Gradient decent algorithm corrective step
        let mut s1 = -_2q3 * (2.0 * q2q4 - _2q1q3 - ax) + _2q2 * (2.0 * q1q2 + _2q3q4 - ay)
            - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
            + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
            + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
        let mut s2 = _2q4 * (2.0 * q2q4 - _2q1q3 - ax) + _2q1 * (2.0 * q1q2 + _2q3q4 - ay)
            - 4.0 * q2 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az)
            + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
            + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
            + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
        let mut s3 = -_2q1 * (2.0 * q2q4 - _2q1q3 - ax) + _2q4 * (2.0 * q1q2 + _2q3q4 - ay)
            - 4.0 * q3 * (1.0 - 2.0 * q2q2 - 2.0 * q3q3 - az)
            + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
            + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
            + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
        let mut s4 = _2q2 * (2.0 * q2q4 - _2q1q3 - ax)
            + _2q3 * (2.0 * q1q2 + _2q3q4 - ay)
            + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx)
            + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my)
            + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz);
        norm = sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); // normalise step magnitude
        if norm == 0.0 {
            return;
        }
        norm = 1.0 / norm;
        s1 *= norm;
        s2 *= norm;
        s3 *= norm;
        s4 *= norm;

        // Compute rate of change of quaternion
        let qdot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1;
        let qdot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2;
        let qdot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3;
        let qdot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4;

        // Integrate to yield quaternion
        q1 += qdot1 * deltat;
        q2 += qdot2 * deltat;
        q3 += qdot3 * deltat;
        q4 += qdot4 * deltat;
        norm = sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); // Normalize quaternion
        if norm == 0.0 {
            return;
        }
        norm = norm / 1.0;
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
    }
}
