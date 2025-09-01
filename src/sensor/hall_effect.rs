use super::{CurrentSensor, SensorError};
use crate::common::em::*;
use embassy_rp::adc;
use embassy_rp::{self, dma};

/// Uses an ADC with three of its ADC channels
pub struct HallEffectADC<'a, M: adc::Mode> {
    pub adc: adc::Adc<'a, M>,
    pub pins: [adc::Channel<'a>; 3],
    pub dma: embassy_rp::Peri<'a, dma::AnyChannel>,

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
        dma: embassy_rp::Peri<'a, dma::AnyChannel>,
        pin_a: adc::Channel<'a>,
        pin_b: adc::Channel<'a>,
        pin_c: adc::Channel<'a>,
        amplification: f32,
        offset: f32,
    ) -> Self {
        Self {
            adc,
            dma,
            pins: [pin_a, pin_b, pin_c],
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
        let [pin_a, pin_b, pin_c] = self.pins;
        (self.adc, pin_a, pin_b, pin_c)
    }
}

impl<'a> CurrentSensor for HallEffectADC<'a, adc::Async> {
    async fn get_currents(&mut self) -> Result<Iabc, SensorError> {
        let mut buf = [0_u16; 3];
        let div = 0; // 100kHz sample rate (48Mhz / 100kHz * 4ch - 1)
        self.adc
            .read_many_multichannel(&mut self.pins, &mut buf, div, self.dma.reborrow())
            .await
            .unwrap();

        Ok(Iabc {
            a: ((buf[0]) as f32) * self.amplification - self.offset,
            b: ((buf[1]) as f32) * self.amplification - self.offset,
            c: ((buf[2]) as f32) * self.amplification - self.offset,
        })
    }
}
