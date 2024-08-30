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
            specification: specification,
            angle: rotor_angle,
            driver: driver,
            pid: pid,
        }
    }
}

// implement FOC control functions for BLDC motor
impl<B: driver::BLDCDriver, R: RotarySensor> FOCMotor for BLDCMotor<'_, B, R> {
    // target is in radians
    fn goto(&mut self, target: f32) -> () {
        if self.angle.is_some() {
            self.pid.set(target);
        }
    }

    // target is in radians
    fn goto_blocking(&mut self, target: f32) -> () {
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

    fn foc_loop(&mut self) -> () {
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
