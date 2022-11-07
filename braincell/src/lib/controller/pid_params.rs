#[derive(Clone, Copy)]
pub struct TuningParams {
    pub kp: f32,
    pub ki: f32,
    pub kd: f32,
    pub p_lim: f32,
    pub i_lim: f32,
    pub d_lim: f32,
    pub out_lim: f32,
}
