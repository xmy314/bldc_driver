use super::{RotarySensor, SensorError};

use as5600_driver::As5600;

/// Sample Implementation over as5600
impl<I2C> RotarySensor for As5600<I2C>
where
    I2C: embedded_hal::i2c::I2c,
{
    fn get_mechanical_angle(&mut self) -> Result<u16, SensorError> {
        match self.angle() {
            Ok(angle) => Ok(angle << 2),
            Err(_) => Err(SensorError::COMMUNICATION),
        }
    }
}
