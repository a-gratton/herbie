use pid::Pid;

use crate::controller::pid_params::TuningParams;
use crate::drivers::motor::mdd3a;
use core::ops::Mul;

pub struct Motors<M1, M2, M3, M4> {
    f_left: MotorController<M1>,
    r_left: MotorController<M2>,
    f_right: MotorController<M3>,
    r_right: MotorController<M4>,
    directions: MotorDirections,
}
impl<M1, M2, M3, M4> Motors<M1, M2, M3, M4>
where
    M1: mdd3a::Start + mdd3a::SetPower,
    M2: mdd3a::Start + mdd3a::SetPower,
    M3: mdd3a::Start + mdd3a::SetPower,
    M4: mdd3a::Start + mdd3a::SetPower,
{
    pub fn new(
        f_left: M1,
        r_left: M2,
        f_right: M3,
        r_right: M4,
        tune: TuningParams,
        directions: MotorDirections,
    ) -> Motors<M1, M2, M3, M4> {
        let mut motors = Motors {
            f_left: MotorController::<M1>::new(f_left, tune),
            r_left: MotorController::<M2>::new(r_left, tune),
            f_right: MotorController::<M3>::new(f_right, tune),
            r_right: MotorController::<M4>::new(r_right, tune),
            directions: directions,
        };
        motors.f_left.start();
        motors.r_left.start();
        motors.f_right.start();
        motors.r_right.start();
        return motors;
    }

    pub fn set_speed_targets(&mut self, targets: &MotorSetPoints) {
        self.f_left
            .set_speed_target(self.directions.f_left * targets.f_left);
        self.r_left
            .set_speed_target(self.directions.r_left * targets.r_left);
        self.f_right
            .set_speed_target(self.directions.f_right * targets.f_right);
        self.r_right
            .set_speed_target(self.directions.r_right * targets.r_right);
    }

    pub fn step(&mut self, vels: &VelocityMeasurement) -> (f32, f32, f32, f32) {
        (
            self.f_left.step(vels.f_left),
            self.r_left.step(vels.r_left),
            self.f_right.step(vels.f_right),
            self.r_right.step(vels.r_right),
        )
    }

    pub fn stop(&mut self) {
        self.f_left.stop();
        self.r_left.stop();
        self.f_right.stop();
        self.r_right.stop();
    }
}

struct MotorController<MotorT> {
    motor: MotorT,
    pid: Pid<f32>,
}
impl<MotorT> MotorController<MotorT>
where
    MotorT: mdd3a::Start + mdd3a::SetPower,
{
    pub fn new(motor: MotorT, t: TuningParams) -> MotorController<MotorT> {
        MotorController {
            motor: motor,
            pid: Pid::new(t.kp, t.ki, t.kd, t.p_lim, t.i_lim, t.d_lim, t.out_lim, 0.0),
        }
    }

    fn start(&mut self) {
        self.motor.start();
    }

    fn set_speed_target(&mut self, target: f32) {
        self.pid.setpoint = target;
    }

    fn step(&mut self, current_velocity: f32) -> f32 {
        let output = self.pid.next_control_output(current_velocity);
        self.motor.set_power(output.output);
        return output.output;
    }

    fn stop(&mut self) {
        self.motor.set_power(0.0);
    }
}

pub struct VelocityMeasurement {
    pub f_left: f32,
    pub r_left: f32,
    pub f_right: f32,
    pub r_right: f32,
}
impl IntoIterator for VelocityMeasurement {
    type Item = f32;
    type IntoIter = core::array::IntoIter<f32, 4>;
    fn into_iter(self) -> Self::IntoIter {
        [self.f_left, self.r_left, self.f_right, self.r_right].into_iter()
    }
}
impl Default for VelocityMeasurement {
    fn default() -> VelocityMeasurement {
        VelocityMeasurement {
            f_left: 0.0,
            r_left: 0.0,
            f_right: 0.0,
            r_right: 0.0,
        }
    }
}

pub struct MotorSetPoints {
    pub f_left: f32,
    pub r_left: f32,
    pub f_right: f32,
    pub r_right: f32,
}
impl IntoIterator for MotorSetPoints {
    type Item = f32;
    type IntoIter = core::array::IntoIter<f32, 4>;
    fn into_iter(self) -> Self::IntoIter {
        [self.f_left, self.r_left, self.f_right, self.r_right].into_iter()
    }
}
impl Default for MotorSetPoints {
    fn default() -> MotorSetPoints {
        MotorSetPoints {
            f_left: 0.0,
            r_left: 0.0,
            f_right: 0.0,
            r_right: 0.0,
        }
    }
}

#[derive(Clone, Copy)]
pub enum Direction {
    Backward = -1,
    Forward = 1,
}
impl Mul<f32> for Direction {
    type Output = f32;
    fn mul(self, rhs: f32) -> Self::Output {
        self as i32 as f32 * rhs
    }
}
impl Mul<i32> for Direction {
    type Output = i32;
    fn mul(self, rhs: i32) -> Self::Output {
        self as i32 * rhs
    }
}

pub struct MotorDirections {
    pub f_left: Direction,
    pub r_left: Direction,
    pub f_right: Direction,
    pub r_right: Direction,
}
