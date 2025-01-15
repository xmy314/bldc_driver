use embedded_hal_0_2::adc::{Channel, OneShot};

use super::{CurrentSensor, SensorError};
use crate::common::em::*;

/// Uses an ADC with three of its ADC channels
pub struct HallEffectADC<Adc, T1, T2, T3>
where
    Adc: OneShot<Adc, u16, T1> + OneShot<Adc, u16, T2> + OneShot<Adc, u16, T3>,
    T1: Channel<Adc, ID = u8>,
    T2: Channel<Adc, ID = u8>,
    T3: Channel<Adc, ID = u8>,
{
    pub adc: Adc,
    pub pin_a: T1,
    pub pin_b: T2,
    pub pin_c: T3,

    /// \[A/count\], typically calculate using
    /// \[A/V\] conversion rate in the hall effect sensor datasheet, and
    /// \[V/count\] in the adc or mcu datasheet.
    pub amplification: f32,
    /// \[A\] typically calculate using
    /// \[A/V\] conversion rate, and
    /// \[V\] when no current is passing through both in the hall effect sensor datasheet.
    pub offset: f32,
}

impl<Adc, T1, T2, T3> HallEffectADC<Adc, T1, T2, T3>
where
    Adc: OneShot<Adc, u16, T1> + OneShot<Adc, u16, T2> + OneShot<Adc, u16, T3>,
    T1: Channel<Adc, ID = u8>,
    T2: Channel<Adc, ID = u8>,
    T3: Channel<Adc, ID = u8>,
{
    pub fn new(adc: Adc, pin_a: T1, pin_b: T2, pin_c: T3, amplification: f32, offset: f32) -> Self {
        Self {
            adc,
            pin_a,
            pin_b,
            pin_c,
            amplification,
            offset,
        }
    }

    pub fn release(self) -> (Adc, T1, T2, T3) {
        (self.adc, self.pin_a, self.pin_b, self.pin_c)
    }
}

impl<Adc, T1, T2, T3> CurrentSensor for HallEffectADC<Adc, T1, T2, T3>
where
    Adc: OneShot<Adc, u16, T1> + OneShot<Adc, u16, T2> + OneShot<Adc, u16, T3>,
    T1: Channel<Adc, ID = u8>,
    T2: Channel<Adc, ID = u8>,
    T3: Channel<Adc, ID = u8>,
{
    fn get_currents(&mut self) -> Result<Iabc, SensorError> {
        let ia = match self.adc.read(&mut self.pin_a) {
            Ok(ia) => ia,
            Err(_) => return Err(SensorError::COMMUNICATION),
        };
        let ib = match self.adc.read(&mut self.pin_b) {
            Ok(ib) => ib,
            Err(_) => return Err(SensorError::COMMUNICATION),
        };
        let ic = match self.adc.read(&mut self.pin_c) {
            Ok(ic) => ic,
            Err(_) => return Err(SensorError::COMMUNICATION),
        };

        Ok(Iabc {
            a: ia as f32 * self.amplification - self.offset,
            b: ib as f32 * self.amplification - self.offset,
            c: ic as f32 * self.amplification - self.offset,
        })
    }
}
