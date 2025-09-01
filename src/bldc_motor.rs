use core::f32::consts;
use defmt::info;
use embassy_futures::block_on;
use embassy_time::Instant;
use micromath::F32;

use crate::common::clamp;
use crate::common::em::{self, Iabc, Iqd, Vabc, Vqd};
use crate::estimator::{AEstimator, LinearEstimator, ReducedEMLinearEstimator};
use crate::pid::PID;
use crate::sensor::{CurrentSensor, RotarySensor, RotorState};
use crate::{driver, FOCMotor};

enum MotorCommand {
    Vsrf(Vabc),
    Vrrf(Vqd),
    Isrf(Iabc),
    Irrf(Iqd),
    Off,
}

/// Physical parameter of the motor that are useful for foc control.
#[derive(Debug, PartialEq)]
pub struct BLDCMotorSpecification {
    pub current_limit: f32,
    pub voltage_limit: f32,

    /// Mode of operation
    /// True means "maximum torque per amp" which stays in the efficient range
    /// False means "maximum torque" which goes into less efficient range to get a little more torque.
    pub mtpa_only: bool,

    /// number of electrical cycles per mechanical cycle,
    /// can be automatically calibrated with `calibrate_rotary_sensor`
    pub pole_pairs: u8,

    /// \[RPM/V\] provided by manufacturer
    pub kv: f32,
    /// resistance of one phase or half of line to line resistance.
    /// can be automatically calibrated with `calibrate_phase_impedence`
    pub phase_resistance: f32,
    /// inductance of one phase or half of line to line resistance.
    /// can be automatically calibrated with `calibrate_phase_impedence`
    pub phase_inductance: f32,
}

// One type of motor that can employ FOC are the BLDC motors.
// This is the implementation of it.
pub struct BLDCMotor<B: driver::BLDCDriver, R: RotarySensor, T: CurrentSensor> {
    pub driver: B,
    pub specification: BLDCMotorSpecification,
    pub m_angle_tracker: Option<RotorState<R>>,
    pub amperage: Option<T>,
    pub theta_pid: PID,

    previous_command: MotorCommand,
}

impl<B: driver::BLDCDriver, R: RotarySensor, T: CurrentSensor> BLDCMotor<B, R, T> {
    pub fn new(
        driver: B,
        specification: BLDCMotorSpecification,
        m_angle_tracker: Option<RotorState<R>>,
        amperage_sensor: Option<T>,
        theta_pid: PID,
    ) -> BLDCMotor<B, R, T> {
        BLDCMotor {
            driver,
            specification,
            m_angle_tracker,
            amperage: amperage_sensor,
            theta_pid,
            previous_command: MotorCommand::Off,
        }
    }

    /// Calibrate the phase impedence of the BLDC motor.
    ///
    /// Responsibilities:
    /// 1. Detect and overwrite phase resistance
    /// 2. Detect and overwrite phase inductance.
    ///
    /// Requirements:
    /// 1. 3 phase current sensing is present
    /// 2. The motor driver channels ABC are aligned with current sensor channels ABC.
    ///
    /// Explanation:
    /// 1. Apply progressively higher voltage in one phase and record the stead state current.
    /// 2. Apply linear regression to find the mapping between current and voltage
    ///     which is V ~= 1.5*R*I.
    /// 4. Apply a step voltage close to the limites in one phase and record how
    ///     the current changes with time.
    /// 5. Apply linear regression to find the mapping between current and time
    ///     which is ln(I-I_t) ~= -(R/L)t + ln(I).
    ///
    /// If inductance is low, the inductance won't be accurate, maybe try using an lrc meter.
    pub fn calibrate_phase_impedence(&mut self) {
        if self.amperage.is_none() {
            return;
        }

        let amperage = self.amperage.as_mut().unwrap();

        let step_count = 200;

        let mut r_estimator = LinearEstimator::new();

        let voltage_increment = 0.1 * self.driver.get_voltage_limit() / step_count as f32;
        for i in 0..=step_count {
            let va = i as f32 * voltage_increment;
            let field_voltage = em::Vabc {
                a: va,
                b: 0.0,
                c: 0.0,
            };
            self.driver.set_srf_voltage(field_voltage);

            // loop until the slope is within 1 std of 0
            // aka so slope is basically flat
            // delay.delay_ms(10);
            let mut ma: ReducedEMLinearEstimator = ReducedEMLinearEstimator::new(0.02);
            loop {
                ma.add(block_on(amperage.get_currents()).unwrap().a);
                if let Some(correlation) = ma.get_square_pearson_correlation() {
                    if correlation < 0.02 && ma.get_n() > 20 {
                        break;
                    }
                }
            }

            let ia = ma.get_stablized_y().unwrap();
            r_estimator.add(ia, va);
            // info!("r step {}, current {}, n{}", i, ia, ma.get_n());

            if ia >= 0.8 * self.specification.current_limit {
                // this is for too much current
                self.driver.off();
                break;
            }
        }

        self.specification.phase_resistance = r_estimator.get_m().unwrap() / 1.5;

        let mut outer_l_estimator = AEstimator::new();
        for _ in 0..step_count {
            self.driver.off();

            // delay until current doesn't change
            let mut ma: ReducedEMLinearEstimator = ReducedEMLinearEstimator::new(0.02);
            loop {
                ma.add(block_on(amperage.get_currents()).unwrap().a);
                if let Some(correlation) = ma.get_square_pearson_correlation() {
                    if correlation < 0.02 && ma.get_n() > 20 {
                        break;
                    }
                }
            }

            self.driver.set_srf_voltage(em::Vabc {
                a: self.specification.current_limit * 1.5 * self.specification.phase_resistance,
                b: 0.0,
                c: 0.0,
            });
            let start_time = Instant::now();
            let mut inner_l_estimator = LinearEstimator::new();
            loop {
                let ia = block_on(amperage.get_currents()).unwrap().a;
                let t = (Instant::now() - start_time).as_micros() as f32 / 1_000_000.0;
                if ia > 0.95 * self.specification.current_limit {
                    self.driver.off();
                    break;
                }
                inner_l_estimator.add(t, F32(self.specification.current_limit - ia).ln().0);
            }
            // m = -1.5 * R / 1.5 * L
            // m = -R / L
            // L = -R / m
            let m_option = inner_l_estimator.get_m();
            match m_option {
                Some(m) => outer_l_estimator.add(-self.specification.phase_resistance / m),
                None => {}
            }
        }

        self.specification.phase_inductance = outer_l_estimator.get_mean().unwrap();
    }

    /// Calibrate the rotary sensor.
    /// Responsibilities:
    /// 1. Detect orientation of sensor wrt driver channels and overwrite the
    ///      offset and directionin the rotor state struct. The result is that
    ///      the Mechanical angle origin would align with electrical angle
    ///      origin and they would increase in the same direciton.
    /// 2. Detect and overwrite the number of poles.
    ///
    /// Requirements:
    /// 1. requires a rotary sensor.
    /// 2. motor can rotate with not that much power.
    ///
    /// Explanation:
    /// 1. Rotates the power supply in voltage or electrical space and collect its
    ///     measured mechanical angle.
    /// 2. Apply linear regression to find the mapping between electrical angls
    ///     and mechanical angles which is theta_m = direction*theta_e/pp+o.
    ///
    /// The reason that pole count is here too is that it needs to be calculated
    /// in the linear regression anyway.
    ///
    /// TODO:
    /// Determine what "not that much power" actually is or how it will be chosen.
    pub fn calibrate_rotary_sensor(&mut self) {
        // No point in calibrating the sensor is the sensor doesn't exist.
        if self.m_angle_tracker.is_none() {
            return;
        }

        info!("estimating is_reverse and theta0 ");

        let mut estimator = LinearEstimator::new();

        // try for rev_count number of electrical revolutions
        // each rev try tick_count number of ticks
        // and then travel in reverse to take out hysterisis.
        // smaller numbers are faster, larger numbers are more accurate
        // the following is just a what worked well enough.

        let rev_count = 10;
        let tick_count = 8;
        for reverse in [false, true].iter() {
            for rev in 0..rev_count {
                for tick in 0..tick_count {
                    let target_rad = match reverse {
                        false => {
                            ((tick + rev * tick_count) as f32) * consts::TAU / (tick_count as f32)
                        }
                        true => {
                            ((rev_count * tick_count - tick - rev * tick_count) as f32)
                                * consts::TAU
                                / (tick_count as f32)
                        }
                    };

                    let field_voltage = em::Vqd {
                        q: 0.0,
                        // d: 1.5
                        //     * self.specification.current_limit
                        //     * self.specification.phase_resistance,
                        d: 0.6,
                    };
                    self.driver.set_rrf_voltage(field_voltage, target_rad);

                    // delay until it isn't turning
                    let mut ma: ReducedEMLinearEstimator = ReducedEMLinearEstimator::new(0.02);
                    loop {
                        self.m_angle_tracker.as_mut().unwrap().update();
                        ma.add(self.m_angle_tracker.as_ref().unwrap().get_rads());
                        if let Some(correlation) = ma.get_square_pearson_correlation() {
                            if correlation < 0.02 && ma.get_n() > 1000 {
                                break;
                            }
                        }
                    }

                    self.m_angle_tracker.as_mut().unwrap().update();
                    let mech_rad = self.m_angle_tracker.as_ref().unwrap().get_rads();

                    estimator.add(target_rad, mech_rad);
                }
            }
        }

        // save some power
        self.driver.off();

        let m = estimator.get_m().unwrap(); // this is 1 / pole pair
        let k = estimator.get_k().unwrap() % (consts::TAU / self.specification.pole_pairs as f32); // this is the smallest mechanical angle such that electrical angle is 0.
        info!("pp {}, k {}", 1.0 / m, k);
        self.specification.pole_pairs = F32(1.0 / m).abs().round().0 as u8;
        self.m_angle_tracker
            .as_mut()
            .unwrap()
            .set_return_mapping(m > 0.0, k);
    }
}

/// implement FOC control functions for BLDC motor
impl<B: driver::BLDCDriver, R: RotarySensor, T: CurrentSensor> FOCMotor for BLDCMotor<B, R, T> {
    /// set target in revolutions
    /// but doesn't execute the action
    fn goto(&mut self, target: f32) {
        if self.m_angle_tracker.is_some() {
            self.theta_pid.set(target);
        }
    }

    /// set target is in revolutions
    /// and execute the action until target is achieved
    fn goto_blocking(&mut self, target: f32) {
        if self.m_angle_tracker.is_some() {
            self.goto(target);

            let mut counter = 0;
            while counter < 100 {
                self.foc_loop();

                let e = self.theta_pid.inspect_p_error();

                match e {
                    Some(p_error) => {
                        // only if the motor is close to target for extend time.
                        if F32(p_error).abs().0 < 0.001 {
                            counter += 1;
                        } else {
                            counter -= if counter > 2 { 2 } else { counter };
                        }
                    }
                    None => {}
                }
            }
        }
    }

    /// send voltage command to the driver based on
    ///     supply voltage,
    ///     motor specification,
    ///     target position,
    ///     pid controller coefficients and state,
    fn foc_loop(&mut self) {
        // Update the rotor angle reading
        if self.m_angle_tracker.is_some() {
            self.m_angle_tracker.as_mut().unwrap().update();
        }

        let m_angle_tracker = self.m_angle_tracker.as_mut().unwrap();

        // Use the angle differences to get an arbitrary unit of power that is desired to the motors.
        m_angle_tracker.update();
        let mech_revs = m_angle_tracker.get_revs();
        let mech_radps = m_angle_tracker.get_rads_per_s();

        // Convert current to the rrf field voltage with the said current as stable point.
        let rotor_angle = (mech_revs) * (self.specification.pole_pairs as f32) * consts::TAU;
        let w = mech_radps * self.specification.pole_pairs as f32;
        let back_emf =
            w / (0.10471975512 * self.specification.kv * self.specification.pole_pairs as f32);
        let v_max = self.specification.voltage_limit;
        let i_max = self.specification.current_limit;
        let reactance = w * self.specification.phase_inductance;
        let resistance = self.specification.phase_resistance;
        let z2 = reactance * reactance + resistance * resistance;
        let z = F32(z2).sqrt().0;

        if let Ok(iabc) = block_on(self.amperage.as_mut().unwrap().get_currents()) {
            let measured_iqd = iabc.parks_transformation(rotor_angle);
            if let MotorCommand::Irrf(set_iqd) = self.previous_command {
                info!(
                    "sq:{} mq:{} sd:{} md:{}",
                    set_iqd.q, measured_iqd.q, set_iqd.d, measured_iqd.d
                );
            }
        }
        // info!("theta {}", mech_revs);

        // treat as desired quature current.
        let desired_throttle = clamp(
            self.theta_pid.update_and_get_throttle(mech_revs),
            -i_max,
            i_max,
        );

        let area_intersection = v_max + z * i_max - back_emf > 0.0;

        // control circles don't over lap.
        if !area_intersection {
            // there is basically nothing that can be done.
            // would be good to disable all gates but that is not possible to 3pwm drivers.
            self.driver.off();
            self.previous_command = MotorCommand::Off;
            // info!("dc {}", w);
            return;
        }

        let mtpa_any = z2 * v_max * v_max - (back_emf * reactance) * (back_emf * reactance) > 0.0;

        // if action is possible along id=0 line
        if mtpa_any {
            // if any action along id=0 is possible, clamp action to within the id=0
            let max_mtpa_iq =
                clamp(
                    -back_emf * resistance / z2
                        + F32(v_max * v_max / z2
                            - (back_emf * reactance) * (back_emf * reactance) / z2)
                        .sqrt()
                        .0,
                    -i_max,
                    i_max,
                );
            let min_mtpa_iq =
                clamp(
                    -back_emf * resistance / z2
                        - F32(v_max * v_max / z2
                            - (back_emf * reactance) * (back_emf * reactance) / z2)
                        .sqrt()
                        .0,
                    -i_max,
                    i_max,
                );

            if desired_throttle >= min_mtpa_iq && desired_throttle <= max_mtpa_iq
                || self.specification.mtpa_only
            {
                let iqd = Iqd {
                    q: clamp(desired_throttle, min_mtpa_iq, max_mtpa_iq),
                    d: 0.0,
                };
                let field_voltage = em::Vqd {
                    q: resistance * iqd.q + reactance * iqd.d + back_emf,
                    d: resistance * iqd.d - reactance * iqd.q,
                };
                self.driver.set_rrf_voltage(field_voltage, rotor_angle);
                self.previous_command = MotorCommand::Irrf(iqd);
                // info!("mtpa {} {} {}", w, iqd.q, iqd.d);
                return;
            }
        }

        // if mtpa mode only
        if self.specification.mtpa_only {
            // fall back if there are no valid control point along id=0.
            // there are a few options:
            // 1. turn switches off to zero the currents.
            //      would be bad but if it wan't already happening due to pwm switching
            // 2. minimize id.
            //      would slow down the motor but that is what is needed to actively stop the motor
            // 3. minimize i.
            //      a worse idea than 1 but the only option maybe since cutting power is not exposed to 3pwm drivers.
            // 4. maximize efficiency to get control back but that would still be a lot of current.
            //      a more elaborate version of idea 2 but unnecessary.
            // the following is idea 2 with iqd being the right most point of the voltage circle.
            let iqd = Iqd {
                q: -back_emf * resistance / z2,
                d: -back_emf * reactance / z2 + v_max / z,
            };
            let field_voltage = em::Vqd {
                q: resistance * iqd.q + reactance * iqd.d + back_emf,
                d: resistance * iqd.d - reactance * iqd.q,
            };
            self.driver.set_rrf_voltage(field_voltage, rotor_angle);
            self.previous_command = MotorCommand::Irrf(iqd);
            // info!("mtpa back up {} {} {}", w, iqd.q, iqd.d);
            return;
        };

        let edge_intersection = 2.0 * (v_max * v_max + z2 * i_max * i_max)
            - ((v_max * v_max - z2 * i_max * i_max) * (v_max * v_max - z2 * i_max * i_max)
                / (back_emf * back_emf)
                + back_emf * back_emf)
            > 0.0;

        // what to do when one control circle envelops the other.
        if !edge_intersection {
            if v_max / z > i_max {
                // voltage envelops current, simply mtpa
                let iqd = Iqd {
                    q: clamp(desired_throttle, -i_max, i_max),
                    d: 0.0,
                };
                let field_voltage = em::Vqd {
                    q: resistance * iqd.q + reactance * iqd.d + back_emf,
                    d: resistance * iqd.d - reactance * iqd.q,
                };
                self.driver.set_rrf_voltage(field_voltage, rotor_angle);
                self.previous_command = MotorCommand::Irrf(iqd);
                // info!("voltage envelop {} {} {}", w, iqd.q, iqd.d);
                return;
            } else {
                // current envelops voltage, follow voltage circle
                let voltage_limited_throttle = clamp(
                    desired_throttle,
                    -back_emf * resistance / z2 - v_max / z,
                    -back_emf * resistance / z2 + v_max / z,
                );
                let iqd = Iqd {
                    q: voltage_limited_throttle,
                    d: -back_emf * reactance / z2
                        + F32(v_max * v_max / z2
                            - (voltage_limited_throttle - back_emf * resistance / z2)
                                * (voltage_limited_throttle - back_emf * resistance / z2))
                        .sqrt()
                        .0,
                };
                let field_voltage = em::Vqd {
                    q: resistance * iqd.q + reactance * iqd.d + back_emf,
                    d: resistance * iqd.d - reactance * iqd.q,
                };
                self.driver.set_rrf_voltage(field_voltage, rotor_angle);
                self.previous_command = MotorCommand::Irrf(iqd);
                // info!("current envelop {} {} {}", w, iqd.q, iqd.d);
                return;
            };
        }

        // if there is a intersection
        let positive_torque_is_current_limited =
            v_max * v_max - z2 * i_max * i_max - back_emf * back_emf
                > 2.0 * i_max * resistance * back_emf;
        let negative_torque_is_current_limited =
            v_max * v_max - z2 * i_max * i_max - back_emf * back_emf
                > -2.0 * i_max * resistance * back_emf;
        let positive_torque_is_voltage_limited = i_max * z2 - v_max * v_max - back_emf * back_emf
            > -2.0 * back_emf * v_max * resistance / z;
        let negative_torque_is_voltage_limited = i_max * z2 - v_max * v_max - back_emf * back_emf
            > 2.0 * back_emf * v_max * resistance / z;
        let positive_torque_intersection = resistance / (2.0 * z2 * back_emf)
            * (v_max * v_max - z2 * i_max * i_max - back_emf * back_emf)
            + reactance / (2.0 * z2)
                * F32(
                    -(v_max * v_max - z2 * i_max * i_max) * (v_max * v_max - z2 * i_max * i_max)
                        / (back_emf * back_emf)
                        + 2.0 * (v_max * v_max + z2 * i_max * i_max)
                        - back_emf * back_emf,
                )
                .sqrt()
                .0;
        let negative_torque_intersection = resistance / (2.0 * z2 * back_emf)
            * (v_max * v_max - z2 * i_max * i_max - back_emf * back_emf)
            - reactance / (2.0 * z2)
                * F32(
                    -(v_max * v_max - z2 * i_max * i_max) * (v_max * v_max - z2 * i_max * i_max)
                        / (back_emf * back_emf)
                        + 2.0 * (v_max * v_max + z2 * i_max * i_max)
                        - back_emf * back_emf,
                )
                .sqrt()
                .0;

        let positive_current_limit = match (
            positive_torque_is_current_limited,
            positive_torque_is_voltage_limited,
        ) {
            (true, true) => {
                panic!(
                    "math is wrong, shouldn't be possible that two circles are in one another w:{}",
                    w
                )
            }
            (true, false) => i_max,
            (false, true) => -back_emf * resistance / z2 - v_max / z,
            (false, false) => positive_torque_intersection,
        };
        let negative_current_limit = match (
            negative_torque_is_current_limited,
            negative_torque_is_voltage_limited,
        ) {
            (true, true) => {
                panic!(
                    "math is wrong, shouldn't be possible that two circles are in one another w:{}",
                    w
                )
            }
            (true, false) => -i_max,
            (false, true) => -back_emf * resistance / z2 + v_max / z,
            (false, false) => negative_torque_intersection,
        };

        let voltage_limited_throttle = clamp(
            desired_throttle,
            negative_current_limit,
            positive_current_limit,
        );
        let iqd = Iqd {
            q: voltage_limited_throttle,
            d: -back_emf * reactance / z2
                + F32(v_max * v_max / z2
                    - (voltage_limited_throttle + back_emf * resistance / z2)
                        * (voltage_limited_throttle + back_emf * resistance / z2))
                .sqrt()
                .0,
        };
        let field_voltage = em::Vqd {
            q: resistance * iqd.q + reactance * iqd.d + back_emf,
            d: resistance * iqd.d - reactance * iqd.q,
        };
        self.driver.set_rrf_voltage(field_voltage, rotor_angle);
        self.previous_command = MotorCommand::Irrf(iqd);
        // info!("other case {} {} {}", w, iqd.q, iqd.d);
    }
}
