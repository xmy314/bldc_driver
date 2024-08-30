use core::f32::consts;
use defmt::info;
use micromath::F32;

use crate::common::em;
use crate::pid::PID;
use crate::sensor::{RotarySensor, RotorState};
use crate::{driver, FOCMotor};

// Physical parameter of the motor that are useful for more advanced control.
#[derive(Debug, PartialEq)]
pub struct BLDCMotorSpecification {
    // number of electrical cycles per mechanical cycle
    pub pole_pairs: u8,
    // the following are useful for more sophisticated control,
    // but they are not useful without current sensing.
    pub kv: u16,
    pub phase_resistance: f32,
    pub phase_inductance: f32,
}

// One type of motor that can employ FOC are the BLDC motors.
// This is the implementation of it.
pub struct BLDCMotor<'a, B: driver::BLDCDriver, R: RotarySensor> {
    pub specification: BLDCMotorSpecification,
    pub driver: B,
    pub angle: Option<RotorState<'a, R>>,
    pub pid: PID<'a>,
}

// An incomplete and overly specific constructor.
// TODO: complete the constructor with the following three task.
//       detect and compensate orientation of driver wireing
//       detect and compensate orientation of sensor
//       detect and compensate sensor 0 position is different from driver 0 position

//       add cogging compensation.
//       add kalman filtering to sensor.
//       add pid autotune.
impl<'a, B: driver::BLDCDriver, R: RotarySensor> BLDCMotor<'a, B, R> {
    pub fn new(
        specification: BLDCMotorSpecification,
        rotor_angle: Option<RotorState<'a, R>>,
        driver: B,
        pid: PID<'a>,
    ) -> BLDCMotor<'a, B, R> {
        BLDCMotor {
            specification,
            angle: rotor_angle,
            driver,
            pid,
        }
    }

    // calibrate the rotary sensor.
    // requires a rotary sensor and a motor driver.
    pub fn calibrate_rotary_sensor(&mut self) {
        // No point in calibrating the sensor is the sensor doesn't exist.
        if self.angle.is_none() {
            return;
        }

        // linear regression, the formula and explanation can be found here
        // https://en.wikipedia.org/wiki/Simple_linear_regression#Normality_assumption
        // y=mx+b where y is the measured angle and x is the input target angle.
        let mut n: f32 = 0.0;
        let mut x: f32 = 0.0;
        let mut xx: f32 = 0.0;
        let mut y: f32 = 0.0;
        let mut xy: f32 = 0.0;

        // try for "some number" of electrical cycles
        let e_rev = 10;
        // each cycle try "some number" of increments
        let tick_per_e_rev = 18;
        // smaller numbers are faster, larger numbers are more accurate
        for i in 0..(e_rev * tick_per_e_rev) {
            let target_rad = (i as f32) * consts::TAU / (tick_per_e_rev as f32);
            let field_voltage = em::Vqd {
                q: 0.0,
                d: self.driver.get_voltage_limit(),
            };
            self.driver.set_rrf_voltage(field_voltage, target_rad);

            // wait until rotor stops moving
            self.angle.as_mut().unwrap().update();
            let mut previous = self.angle.as_ref().unwrap().get_rads();
            let mut count = 0;
            while count < 50 {
                self.angle.as_mut().unwrap().update();
                let test = self.angle.as_ref().unwrap().get_rads();
                // f32 cannot be exactly the same, but close enough for a period of time would be good enough.
                if F32(test - previous).abs().0 < 0.002 {
                    count += 1;
                } else {
                    count -= if count > 2 { 2 } else { count };
                }
                previous = 0.9 * previous + 0.1 * test;
            }

            self.angle.as_mut().unwrap().update();
            let mech_rad = self.angle.as_ref().unwrap().get_rads();

            n += 1.0;
            x += target_rad;
            xx += target_rad * target_rad;
            y += mech_rad;
            xy += target_rad * mech_rad;
        }

        // repeat the previous loop again take out the effect of hysterisis.
        for i in (0..(e_rev * tick_per_e_rev)).rev() {
            let target_rad = (i as f32) * consts::TAU / (tick_per_e_rev as f32);
            let field_voltage = em::Vqd {
                q: 0.0,
                d: self.driver.get_voltage_limit(),
            };
            self.driver.set_rrf_voltage(field_voltage, target_rad);

            // wait until rotor stops moving
            self.angle.as_mut().unwrap().update();
            let mut previous = self.angle.as_ref().unwrap().get_rads();
            let mut count = 0;
            while count < 50 {
                self.angle.as_mut().unwrap().update();
                let test = self.angle.as_ref().unwrap().get_rads();
                // f32 cannot be exactly the same, but close enough for a period of time would be good enough.
                if F32(test - previous).abs().0 < 0.002 {
                    count += 1;
                } else {
                    count -= if count > 2 { 2 } else { count };
                }
                previous = 0.9 * previous + 0.1 * test;
            }

            self.angle.as_mut().unwrap().update();
            let mech_rad = self.angle.as_ref().unwrap().get_rads();

            n += 1.0;
            x += target_rad;
            xx += target_rad * target_rad;
            y += mech_rad;
            xy += target_rad * mech_rad;
        }

        // save some power
        self.driver.off();

        let m = (n * xy - x * y) / (n * xx - x * x); // this is 1 / pole pair
        let k = ((xx * y - x * xy) / (n * xx - x * x))
            % (consts::TAU / self.specification.pole_pairs as f32); // this is the smallest mechanical angle such that electrical angle is 0.
        info!("s*pp {}, k {}", 1.0 / m, k);
        self.specification.pole_pairs = F32(m).abs().round().0 as u8;
        self.angle.as_mut().unwrap().set_return_mapping(m > 0.0, k);
    }
}

// implement FOC control functions for BLDC motor
impl<B: driver::BLDCDriver, R: RotarySensor> FOCMotor for BLDCMotor<'_, B, R> {
    // target is in radians
    fn goto(&mut self, target: f32) {
        if self.angle.is_some() {
            self.pid.set(target);
        }
    }

    // target is in radians
    fn goto_blocking(&mut self, target: f32) {
        if self.angle.is_some() {
            self.goto(target);

            let mut counter = 0;
            loop {
                self.foc_loop();

                let e = self.angle.as_ref().unwrap().get_rads() - self.pid.sp;

                // only if multiple the motor is close to target for extend time.
                if F32(e).abs().0 < 0.005 {
                    counter += 1;
                } else {
                    counter -= if counter > 1 { 1 } else { counter };
                }

                if counter >= 100 {
                    break;
                }
            }
        }
    }

    fn foc_loop(&mut self) {
        // Update the rotor angle reading
        if self.angle.is_some() {
            self.angle.as_mut().unwrap().update();
        }

        let angle_state = self.angle.as_ref().unwrap();

        // Use the angle differences to get an arbitrary unit of power that is desired to the motors.
        let desired_throttle = self.pid.update_and_get_throttle(angle_state.get_rads());

        // Convert desired throttle to the field voltage desired.

        // TODO: control field current instead as they relate to the torque.
        //       This requires current sensing to be implemented first which does not exist.
        //

        // electrical angle is the rotor angle from electricity's perspective
        let electrical_angle =
            (angle_state.get_fract()) * (self.specification.pole_pairs as f32) * consts::TAU;

        let voltage_limit = 0.2 * self.driver.get_voltage_limit();

        let throttle: f32 = if desired_throttle > voltage_limit {
            voltage_limit
        } else if desired_throttle < -voltage_limit {
            -voltage_limit
        } else {
            desired_throttle
        };

        let mtpv_voltage_angle_rad = F32(angle_state.get_rads_per_s()
            * self.specification.phase_inductance
            / self.specification.phase_resistance)
        .atan()
        .0 + 0.1
            * F32(
                angle_state.get_rads_per_s() * self.specification.phase_inductance
                    / self.specification.phase_resistance
                    / 20.0,
            )
            .atan()
            .powi(3)
            .0;

        let field_voltage = em::Vqd {
            // q should be in the direction of the throttle.
            q: throttle * F32(mtpv_voltage_angle_rad).cos().0,
            // d should be 0 or negative for maximum torque per volt.
            d: -throttle * F32(mtpv_voltage_angle_rad).sin().0,
        };

        self.driver.set_rrf_voltage(field_voltage, electrical_angle);

        info!("{}, {}", self.pid.sp, angle_state.get_rads());
    }
}
