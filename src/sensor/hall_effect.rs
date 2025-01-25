use super::{CurrentSensor, SensorError};
use crate::common::em::*;
use embassy_futures::block_on;
use embassy_rp::adc;

/// Uses an ADC with three of its ADC channels
pub struct HallEffectADC<'a, M: adc::Mode> {
    pub adc: adc::Adc<'a, M>,
    pub pin_a: adc::Channel<'a>,
    pub pin_b: adc::Channel<'a>,
    pub pin_c: adc::Channel<'a>,

    /// \[A/count\], typically calculate using
    /// \[A/V\] conversion rate in the hall effect sensor datasheet, and
    /// \[V/count\] in the adc or mcu datasheet.
    pub amplification: f32,
    /// \[A\] typically calculate using
    /// \[A/V\] conversion rate, and
    /// \[V\] when no current is passing through both in the hall effect sensor datasheet.
    pub offset: f32,
}

impl<'a, M: adc::Mode> HallEffectADC<'a, M> {
    pub fn new(
        adc: adc::Adc<'a, M>,
        pin_a: adc::Channel<'a>,
        pin_b: adc::Channel<'a>,
        pin_c: adc::Channel<'a>,
        amplification: f32,
        offset: f32,
    ) -> Self {
        Self {
            adc,
            pin_a,
            pin_b,
            pin_c,
            amplification,
            offset,
        }
    }

    pub fn release(
        self,
    ) -> (
        adc::Adc<'a, M>,
        adc::Channel<'a>,
        adc::Channel<'a>,
        adc::Channel<'a>,
    ) {
        (self.adc, self.pin_a, self.pin_b, self.pin_c)
    }
}

impl<'a> CurrentSensor for HallEffectADC<'a, adc::Async> {
    fn get_currents(&mut self) -> Result<Iabc, SensorError> {
        let ia = match block_on(self.adc.read(&mut self.pin_a)) {
            Ok(ia) => ia,
            Err(_) => return Err(SensorError::COMMUNICATION),
        };
        let ib = match block_on(self.adc.read(&mut self.pin_b)) {
            Ok(ib) => ib,
            Err(_) => return Err(SensorError::COMMUNICATION),
        };
        let ic = match block_on(self.adc.read(&mut self.pin_c)) {
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
