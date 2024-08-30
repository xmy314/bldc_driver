use defmt::*;

use core::f32::consts;
use micromath::F32;

use rp2040_hal::timer::Instant;
use rp2040_hal::Timer;

pub mod magnetic_i2c;

// Sensor is something that returns the rotor angle sensed by something relative to somewhere.
// TODO: current sensing.

pub trait RotarySensor {
    fn get_mechanical_angle(&mut self) -> Result<u16, embedded_hal::i2c::ErrorKind>;
}

// RotorTracker is a wrapper around sensors to track the number of turns, fraction of turns and an angular speed.
pub struct RotorState<'a, RSensor: RotarySensor> {
    // source of rotor information
    sensor: RSensor,
    // source of temporal information
    timer: &'a Timer,

    // number of full revolutions, rounded to negative infinity
    full_revs: i16,
    // fractions of a revolution out of wrap.
    fractions: u16,
    // last updated
    prior_update: Instant,

    // angular position
    rads: f32,
    // angular velocity
    rads_per_s: f32,

    // modification to the return value
    is_correct_direction: bool,
    //
    reading_to_origin: f32,
}

impl<'a, RSensor: RotarySensor> RotorState<'a, RSensor> {
    pub fn new(timer: &'a Timer, mut sensor: RSensor) -> Self {
        let initial_reading;
        let now: fugit::Instant<u64, 1, 1000000>;
        loop {
            let result = sensor.get_mechanical_angle();
            match result {
                Ok(i) => {
                    initial_reading = i;
                    now = timer.get_counter();
                    break;
                }
                Err(_) => {
                    info!("angle reading initialization failed");
                }
            };
        }
        RotorState {
            timer,
            sensor,

            full_revs: 0,
            fractions: initial_reading,
            prior_update: now,

            rads: initial_reading as f32 / 65535.0,
            rads_per_s: 0.0,

            is_correct_direction: true,
            reading_to_origin: 0.0,
        }
    }

    pub fn set_return_mapping(&mut self, is_correct_direction: bool, reading_to_origin: f32) {
        // the following logic combines the existing transformation and the new transformation into a new transformation.
        if self.is_correct_direction {
            self.reading_to_origin += reading_to_origin;
        } else {
            self.reading_to_origin -= reading_to_origin;
        }
        self.is_correct_direction ^= !is_correct_direction;
    }

    pub fn update(&mut self) {
        let now: rp2040_hal::fugit::Instant<u64, 1, 1000000> = self.timer.get_counter();
        let delta_s = ((now - self.prior_update).to_micros() as f32) / 1000000.0;

        let potential_reading = self.sensor.get_mechanical_angle();
        match potential_reading {
            Ok(rotor_angle) => {
                let prior_rads = self.rads;

                let quadrant: u16 = 4 * (rotor_angle / 16384) + self.fractions / 16384;

                match quadrant {
                    // previously end of cycle, overflowed to start of cycle
                    // add one to the number of revolutions
                    3 => {
                        self.full_revs += 1;
                    }
                    // previously start of cycle, underflowed to end of cycle
                    // subtract one to the number of revolutions
                    12 => {
                        self.full_revs -= 1;
                    }
                    _ => {}
                }

                self.fractions = rotor_angle;

                // update number of rads
                self.rads =
                    consts::TAU * (self.full_revs as f32 + (self.fractions as f32 / 65536.0));

                // quick exponential filter to get
                self.rads_per_s =
                    0.99 * self.rads_per_s + 0.01 * (self.rads - prior_rads) / delta_s;
            }
            Err(_) => {
                // still update, just based on the prior results.
                self.rads += self.rads_per_s * delta_s;
                let revs = F32(self.rads / consts::TAU);
                self.full_revs = revs.floor().0 as i16;
                self.fractions = (65536.0 * (revs.0 - revs.floor().0)) as u16;
            }
        }
        self.prior_update = now;
    }

    // return the number of radians with respect to the selected origin and direction
    pub fn get_rads(&self) -> f32 {
        if self.is_correct_direction {
            self.rads - self.reading_to_origin
        } else {
            -(self.rads - self.reading_to_origin)
        }
    }

    // return the angular velocity with respect to the selected direction
    pub fn get_rads_per_s(&self) -> f32 {
        if self.is_correct_direction {
            self.rads_per_s
        } else {
            -self.rads_per_s
        }
    }

    // return the number of turns, whole and fractional, with respect to the selected origin and direction
    pub fn get_revs(&self) -> f32 {
        self.get_rads() / consts::TAU
    }

    // return the fraction of a turn with respect to the selected origin and direction
    pub fn get_fract(&self) -> f32 {
        self.get_revs() % 1.0
    }
}
