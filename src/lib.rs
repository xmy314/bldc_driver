#![no_std]
#![no_main]
#![allow(dead_code)]
pub mod common; // shared behaviour of different modules

pub mod driver; // from logic to physics
pub mod sensor; // from physics to logic

pub mod bldc_motor; // implement foc for bldc.

pub mod pid; // logic for pid

// The control interface to 3 phase motors.
// Once initialized, the internal components can be hidden away.
pub trait FOCMotor {
    fn goto(&mut self, target: f32) -> ();
    fn goto_blocking(&mut self, target: f32) -> ();
    fn foc_loop(&mut self) -> ();
}

// TODO: cogging torque compensation
// pub struct CoggingCompensation {
//     // 4096 increments for one revolution of the rotor.
//     // a compromise because making it bigger feel unnecessary.
//     compensation_voltage: [f32; 4096],
// }

// impl CoggingCompensation {
//     fn test() -() {

//     }
// }

//
// https://drive.google.com/file/d/13rkv5P4SPrwZmB9B4wkn2wWThXyDSLwa/view?usp=sharing

// Planning
// there are multiple torque that work with or against the motor. In this project,
// they can be divided in three terms, acceleration, load, and friction/cogging.
// Friction/cogging are dealt with through cogging compensation. Acceleration and
// load torque would be mostly dealt with pid.
