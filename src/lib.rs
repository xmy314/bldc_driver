#![no_std]
#![no_main]
#![allow(dead_code)]

/// Meant to be more useful things but currently just voltage and current in srf and rrf.
pub mod common;

/// Useful Statistics
pub mod estimator;
/// PID controller
pub mod pid;

/// Auto calibration algorithum and the main foc control algorithum.
pub mod bldc_motor;
/// Convert desired voltage to pwm signals depending on driver type.
pub mod driver;
/// Get angle and current by sending sensors correct signals.
pub mod sensor;

/// The main control interface to 3 phase motors.
/// Once instantiated, the internal components can be hidden away.
pub trait FOCMotor {
    fn goto(&mut self, target: f32);
    fn goto_blocking(&mut self, target: f32);
    fn foc_loop(&mut self);
}
