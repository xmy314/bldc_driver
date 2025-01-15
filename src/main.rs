#![no_std]
#![no_main]
// #![allow(unused_imports)]

/// The linker will place this boot block at the start of our program image. We
/// need this to help the ROM bootloader get our code up and running.
/// Note: This boot block is not necessary when using a rp-hal based BSP
/// as the BSPs already perform this step.
#[link_section = ".boot2"]
#[used]
pub static BOOT2: [u8; 256] = rp2040_boot2::BOOT_LOADER_GENERIC_03H;

// some debug stuff
use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

// specify the board
use rp2040_hal::{self as hal};
// access the hardware
use hal::{
    adc::Adc, adc::AdcPin, clocks::init_clocks_and_plls, fugit::RateExtU32, pac, sio::Sio,
    watchdog::Watchdog, Clock,
};

// made drivers
use foc_motor_control::driver::{self, BLDCDriver};
use foc_motor_control::pid;
use foc_motor_control::sensor::{self, CurrentSensor};
use foc_motor_control::FOCMotor;
use foc_motor_control::{bldc_motor, sensor::magnetic_i2c};

#[entry]
fn main() -> ! {
    info!("Program start");

    // get all peripherals defined by hal.
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();
    let timer: hal::Timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // get pins and setting up the external harware.
    let pins = rp2040_hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // setup i2c
    // Configure two pins as being I²C, not GPIO
    let sda_pin = pins.gpio16.reconfigure();
    let scl_pin = pins.gpio17.reconfigure();

    // Create the I²C drive, using the two pre-configured pins. This will fail
    // at compile time if the pins are in the wrong mode, or if this I²C
    // peripheral isn't available on these pins!
    let i2c = hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin, // Try `not_an_scl_pin` here
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    // Init PWMs
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM slices
    // "top" is proportional to period of pwm, so change it if there is an constant undesired high frequency noise.
    let pwm4 = &mut pwm_slices.pwm4;
    let pwm3 = &mut pwm_slices.pwm3;
    let pwm2 = &mut pwm_slices.pwm2;
    pwm4.set_ph_correct();
    pwm3.set_ph_correct();
    pwm2.set_ph_correct();
    pwm4.set_top(0x04E1); // 100khz but due to phase correct is actually 50khz
    pwm3.set_top(0x04E1); // 100khz but due to phase correct is actually 50khz
    pwm2.set_top(0x04E1); // 100khz but due to phase correct is actually 50khz
    pwm4.enable();
    pwm3.enable();
    pwm2.enable();

    // get individual PWM channels
    let channel4a = &mut pwm4.channel_a;
    let channel4b = &mut pwm4.channel_b;
    let channel3a = &mut pwm3.channel_a;
    let channel3b = &mut pwm3.channel_b;
    let channel2a = &mut pwm2.channel_a;
    let channel2b = &mut pwm2.channel_b;

    // set the pwm channels to pins
    channel4b.output_to(pins.gpio25);
    channel4a.output_to(pins.gpio24);
    channel3b.output_to(pins.gpio23);
    channel3a.output_to(pins.gpio22);
    channel2b.output_to(pins.gpio21);
    channel2a.output_to(pins.gpio20);

    // invert low side to avoid shorting
    channel4a.set_inverted();
    channel3a.set_inverted();
    channel2a.set_inverted();

    let mut motor = bldc_motor::BLDCMotor::new(
        &timer,
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
        Some(sensor::RotorState::new(
            &timer,
            magnetic_i2c::MageticI2C::new(i2c, magnetic_i2c::AS5600_CONFIG),
        )),
        Some(sensor::hall_effect::HallEffectADC::new(
            Adc::new(pac.ADC, &mut pac.RESETS),
            AdcPin::new(pins.gpio26).unwrap(),
            AdcPin::new(pins.gpio27).unwrap(),
            AdcPin::new(pins.gpio28).unwrap(),
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
    delay.delay_ms(10);

    info!("to 100");
    motor.goto_blocking(0.375);
    motor.driver.off();

    loop {
        delay.delay_ms(100);
    }
}
