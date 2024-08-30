// Isolate the behaviour that connects RotorTracker to the specific as5600 sensor.
// TODO:    separate sensor by means of communication, (eg pwm, quadruture, i2c, voltage)
//          and have specific sensors be represented a constant struct that describes the communication.
//          This is already done is the exisitng arduino foc library.

use super::RotarySensor;
use embedded_hal::i2c::{ErrorKind, I2c};

pub struct MageticI2CConfig {
    pub chip_address: u8,
    pub bit_resolution: u8,
    pub angle_register: u8,
    pub data_start_bit: u8,
}

// NOT the complete set of features for as5600, but these are the ones simple foc have.
pub const AS5600_CONFIG: MageticI2CConfig = MageticI2CConfig {
    chip_address: 0x36,
    bit_resolution: 12,
    angle_register: 0x0e, // 0x0c is unfiltered value.
    data_start_bit: 11,
};

pub struct MageticI2C<I: I2c> {
    bus: I,
    config: MageticI2CConfig,
}

impl<I, E> MageticI2C<I>
where
    I: I2c<Error = E>,
{
    pub fn new(bus: I, config: MageticI2CConfig) -> Self {
        Self { bus, config }
    }

    pub fn release(self) -> I {
        self.bus
    }

    /// Helper function for write-reading 2 bytes from the given register.
    fn read_u16(&mut self, command: u8) -> Result<u16, ErrorKind> {
        let mut buffer = [0u8; 2];
        let result = self
            .bus
            .write_read(self.config.chip_address, &[command], &mut buffer);
        match result {
            Ok(_) => Ok(u16::from_be_bytes(buffer)),
            Err(_) => Err(ErrorKind::Bus),
        }
    }

    /// Helper function for writing 2 bytes to the given register.
    fn write_u16(&mut self, command: u8, bytes: u16) -> Result<(), ErrorKind> {
        let bytes: [u8; 2] = bytes.to_be_bytes();
        let buffer = [command, bytes[0], bytes[1]];
        let result = self.bus.write(self.config.chip_address, &buffer);
        match result {
            Ok(some) => Ok(some),
            Err(_) => Err(ErrorKind::Bus),
        }
    }
}

impl<I, E> RotarySensor for MageticI2C<I>
where
    I: I2c<Error = E>,
{
    fn get_mechanical_angle(&mut self) -> Result<u16, ErrorKind> {
        let register_value = self.read_u16(self.config.angle_register)?;
        let masked_value = register_value
            & ((1 << (self.config.data_start_bit + 1)) - 1)
            & !((1 << (1 + self.config.data_start_bit - self.config.bit_resolution)) - 1);
        let scaled_value = masked_value << (16 - self.config.bit_resolution);
        Ok(scaled_value)
    }
}
