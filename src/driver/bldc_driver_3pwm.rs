#![allow(dead_code)]
use crate::common::em;
use crate::driver::BLDCDriver;

use embedded_hal::pwm;

// Modify the "physical" field voltage in rotor reference frame,
// when the three phases of the motor are connected to 3 pairs of complementary mosfet,
// and controlled by pwm offset.

#[derive(Debug)]
pub struct BLDCDriver3PWM<A: pwm::SetDutyCycle, B: pwm::SetDutyCycle, C: pwm::SetDutyCycle> {
    pub vdc: f32,
    pub a: A,
    pub b: B,
    pub c: C,
}

impl<A: pwm::SetDutyCycle, B: pwm::SetDutyCycle, C: pwm::SetDutyCycle> BLDCDriver3PWM<A, B, C> {
    fn set_srf_voltage_unsafe(&mut self, v_srf: em::Vabc) -> () {
        // Warning, this is not safe, use the safe ones instead.
        let minimum_v = if v_srf.a > v_srf.b {
            if v_srf.b > v_srf.c {
                // a>b>c
                v_srf.c
            } else {
                // a>b and c>b
                v_srf.b
            }
        } else {
            if v_srf.a > v_srf.c {
                // b>a>c
                v_srf.c
            } else {
                // b>a and c>a
                v_srf.a
            }
        };

        // This squezes out and extra 15.47% voltage by using the fact
        // the three phases are balanced and 120 degrees apart.

        let duty_a = ((v_srf.a - minimum_v) / self.vdc * 65535.0) as u16;
        let duty_b = ((v_srf.b - minimum_v) / self.vdc * 65535.0) as u16;
        let duty_c = ((v_srf.c - minimum_v) / self.vdc * 65535.0) as u16;

        self.a.set_duty_cycle_fraction(duty_a, 65535).unwrap();
        self.b.set_duty_cycle_fraction(duty_b, 65535).unwrap();
        self.c.set_duty_cycle_fraction(duty_c, 65535).unwrap();
    }
}

impl<A: pwm::SetDutyCycle, B: pwm::SetDutyCycle, C: pwm::SetDutyCycle> BLDCDriver
    for BLDCDriver3PWM<A, B, C>
{
    fn get_voltage_limit(&self) -> f32 {
        // root 3 for 3 phases.
        self.vdc / 1.732
    }

    fn set_srf_voltage(&mut self, v_srf: em::Vabc) -> () {
        let v_srf_limited = v_srf.limit(self.get_voltage_limit());

        self.set_srf_voltage_unsafe(v_srf_limited);
    }

    fn set_rrf_voltage(&mut self, v_rrf: em::Vqd, rotor_angle_rads: f32) -> () {
        let v_srf_limited = v_rrf
            .limit(self.get_voltage_limit())
            .inverse_parks_transformation(rotor_angle_rads);

        self.set_srf_voltage_unsafe(v_srf_limited);
    }

    fn off(&mut self) -> () {
        self.a.set_duty_cycle_fully_off().unwrap();
        self.b.set_duty_cycle_fully_off().unwrap();
        self.c.set_duty_cycle_fully_off().unwrap();
    }
}
