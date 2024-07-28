#![allow(dead_code)]

use core::cmp::Ordering;
use core::ops::Sub;

#[derive(Debug, PartialEq, PartialOrd, Eq, Ord)]
pub struct PositiontTracker {
    // number of full revolutions
    pub revolutions: i16,
    // fractions out of wrap
    pub fractions: u16,
    // a setting that describes number of increment in a cycle
    pub wrap: u16,
}

impl PositiontTracker {
    pub fn new(reading: u16, wrap: u16) -> PositiontTracker {
        PositiontTracker {
            revolutions: 0,
            fractions: reading,
            wrap: wrap,
        }
    }

    pub fn update(&mut self, reading: u16) -> () {
        if reading < self.wrap / 4 {
            if self.fractions < self.wrap / 4 {
                self.fractions = reading;
            } else if self.wrap / 4 <= self.fractions && self.fractions < self.wrap * 3 / 4 {
                self.fractions = reading;
            } else {
                // self.wrap * 3 / 4<=self.fractions
                self.fractions = reading;
                self.revolutions += 1;
            }
        } else if self.wrap / 4 <= reading && reading < self.wrap * 3 / 4 {
            self.fractions = reading
        } else {
            // self.wrap * 3 / 4<=reading
            if self.fractions < self.wrap / 4 {
                self.revolutions -= 1;
                self.fractions = reading;
            } else if self.wrap / 4 <= self.fractions && self.fractions < self.wrap * 3 / 4 {
                self.fractions = reading;
            } else {
                // self.wrap * 3 / 4<=self.fractions
                self.fractions = reading;
            }
        }
    }

    pub fn cmp(&self, other: &Self) -> Ordering {
        if self.revolutions > other.revolutions {
            return Ordering::Greater;
        } else if self.revolutions < other.revolutions {
            return Ordering::Less;
        } else {
            let f1 = (self.fractions as f32) / (self.wrap as f32);
            let f2 = (other.fractions as f32) / (other.wrap as f32);
            if f1 > f2 {
                return Ordering::Greater;
            } else if f1 < f2 {
                return Ordering::Less;
            } else {
                return Ordering::Equal;
            }
        }
    }

    pub fn as_float(&self) -> f32 {
        self.revolutions as f32 + (self.fractions as f32 / self.wrap as f32)
    }
}
