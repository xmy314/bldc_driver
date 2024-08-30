use crate::common::em;

pub mod bldc_driver_3pwm;

// Modify the "physical" field voltage in rotor reference frame.

pub trait BLDCDriver {
    fn get_voltage_limit(&self) -> f32;
    fn set_srf_voltage(&mut self, v_srf: em::Vabc) -> ();
    fn set_rrf_voltage(&mut self, v_rrf: em::Vqd, rotor_angle: f32) -> ();
    fn off(&mut self) -> ();
}
