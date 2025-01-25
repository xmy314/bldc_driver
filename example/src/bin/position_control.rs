#![no_std]
#![no_main]
// #![allow(unused_imports)]

// some debug stuff
use defmt::*;
use defmt_rtt as _;
use embassy_rp::gpio::Pull;
use panic_probe as _;

// specify the board
use cortex_m_rt::entry;
use embassy_rp::adc::{Adc, Channel, Config, InterruptHandler};
use embassy_rp::bind_interrupts;
use embassy_rp::i2c::{self, Config as I2cConfig};
use embassy_rp::pwm::{Config as PwmConfig, Pwm};
use embassy_time::{block_for, Duration, Timer};

// made drivers
use foc_motor_control::bldc_motor;
use foc_motor_control::driver::{self, BLDCDriver};
use foc_motor_control::pid;
use foc_motor_control::sensor::{self};
use foc_motor_control::FOCMotor;

bind_interrupts!(struct Irqs {
    ADC_IRQ_FIFO => InterruptHandler;
});

#[entry]
fn main() -> ! {
    info!("Program start");

    // get all peripherals defined by hal.
    let p = embassy_rp::init(Default::default());

    // setup i2c0
    let sda_pin = p.PIN_16;
    let scl_pin = p.PIN_17;
    let i2c = i2c::I2c::new_blocking(p.I2C0, scl_pin, sda_pin, I2cConfig::default());

    // Configure PWM slices
    // "top" is proportional to period of pwm, so change it if there is an constant undesired high frequency noise.
    let mut c = PwmConfig::default();
    c.invert_a = true;
    c.phase_correct = true;
    c.top = 0x04E1; // 100khz, but phase correct so 50 effective khz triangle wave
    c.enable = true;

    // get individual PWM channels

    let slice4 = Pwm::new_output_ab(p.PWM_SLICE4, p.PIN_24, p.PIN_25, c.clone()).split();
    let slice3 = Pwm::new_output_ab(p.PWM_SLICE3, p.PIN_22, p.PIN_23, c.clone()).split();
    let slice2 = Pwm::new_output_ab(p.PWM_SLICE2, p.PIN_20, p.PIN_21, c.clone()).split();

    let channel4a = slice4.0.unwrap();
    let channel4b = slice4.1.unwrap();
    let channel3a = slice3.0.unwrap();
    let channel3b = slice3.1.unwrap();
    let channel2a = slice2.0.unwrap();
    let channel2b = slice2.1.unwrap();

    // setup adc
    let adc = Adc::new(p.ADC, Irqs, Config::default());

    let adc_a = Channel::new_pin(p.PIN_26, Pull::None);
    let adc_b = Channel::new_pin(p.PIN_27, Pull::None);
    let adc_c = Channel::new_pin(p.PIN_28, Pull::None);

    let mut motor = bldc_motor::BLDCMotor::new(
        driver::bldc_driver_6pwm::BLDCDriver6PWM {
            vdc: 11.1,        // supply, can be calculated and monitored, but didn't consider doing so.
            half_deadtime: 6, // [7/65535] constrain
            ah: channel4b,
            al: channel4a,
            bh: channel3b,
            bl: channel3a,
            ch: channel2b,
            cl: channel2a,
        },
        bldc_motor::BLDCMotorSpecification {
            mtpa_only: false,         // control scheme,
            current_limit: 7.0,       // constrain
            voltage_limit: 7.0,       // constrain
            kv: 1000.0,               // given by motor manufaturer
            pole_pairs: 7,            // calibratable
            phase_resistance: 0.129,  // calibratable
            phase_inductance: 0.0002, // calibratable
        },
        Some(sensor::RotorState::new(as5600_driver::As5600::new(i2c))),
        Some(sensor::hall_effect::HallEffectADC::new(
            adc,
            adc_a,
            adc_b,
            adc_c,
            (1.0 / 0.132) * (3.3 / 4096.0), // amp per volt * volts per count = amp/count = G
            (1.0 / 0.132) * 1.65,           // amp per volt * volts at no current = amp
        )),
        pid::PID::new(40.0, 5.0, 0.8, false, false), // no autotune
    );

    motor.driver.off();

    // measures resistance and inductance
    // but inductance may not be accurate.
    // motor.calibrate_phase_impedence();
    // motor.driver.off();

    // either use the former of the following two lines to calibrate.
    // or use the lattar to directly directly set the values as calibration takes a moment..
    // motor.calibrate_rotary_sensor();
    motor
        .m_angle_tracker
        .as_mut()
        .unwrap()
        .set_return_mapping(false, 0.46);

    info!("main loop");

    info!(
        "{} {} {} {}",
        motor.specification.voltage_limit,
        motor.specification.current_limit,
        motor.specification.phase_resistance,
        motor.specification.phase_inductance,
    );
    block_for(Duration::from_micros(10));

    info!("to 100");
    motor.goto_blocking(0.375);
    motor.driver.off();

    loop {
        block_for(Duration::from_micros(100));
    }
}
