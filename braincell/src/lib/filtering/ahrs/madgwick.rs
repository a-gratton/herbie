// Madgwick filter implementation
// Ported from https://github.com/drcpattison/ICM-20948/blob/master/src/AHRSAlgorithms.cpp

use core::f32::consts::PI;
use libm::{asinf, atan2f, sqrtf};

const RAD_TO_DEG: f32 = 180.0 / PI;
pub const DEFAULT_BETA: f32 = 1.5;

pub struct MadgwickFilter {
    q: (f32, f32, f32, f32), // quaternion
    beta: f32,               // free parameter
}

impl MadgwickFilter {
    pub fn new(beta: f32) -> Self {
        Self {
            q: (1.0, 0.0, 0.0, 0.0),
            beta,
        }
    }

    // (ax, ay, az) in any unit
    // (gx, gy, gz) in rad/s
    // (mx, my, mz) in any unit
    // deltat (time delta between update calls) in seconds
    pub fn update(
        &mut self,
        mut ax: f32,
        mut ay: f32,
        mut az: f32,
        gx: f32,
        gy: f32,
        gz: f32,
        mut mx: f32,
        mut my: f32,
        mut mz: f32,
        deltat: f32,
    ) {
        let mut q1 = self.q.0;
        let mut q2 = self.q.1;
        let mut q3 = self.q.2;
        let mut q4 = self.q.3;

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
        let mut norm = 1.0 / sqrtf(ax * ax + ay * ay + az * az);
        ax *= norm;
        ay *= norm;
        az *= norm;

        // Normalize mag data
        norm = 1.0 / sqrtf(mx * mx + my * my + mz * mz);
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
        norm = 1.0 / sqrtf(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); // normalise step magnitude
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
        norm = 1.0 / sqrtf(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); // Normalize quaternion
        self.q.0 = q1 * norm;
        self.q.1 = q2 * norm;
        self.q.2 = q3 * norm;
        self.q.3 = q4 * norm;
    }

    // (roll, pitch, yaw) in degrees
    pub fn get_euler_angles(&self) -> (f32, f32, f32) {
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

    pub fn reset(&mut self) {
        self.q = (1.0, 0.0, 0.0, 0.0);
    }
}
