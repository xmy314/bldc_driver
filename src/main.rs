#![no_std]
#![no_main]
#![allow(unused_imports)]

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

// the shared library of all embeded systems.
use embedded_hal::{
    self,
    delay::{self, DelayNs},
    digital::{ErrorType, OutputPin},
};
// specify the board
use rp2040_hal::{self as hal, pac::watchdog::tick};
// access the hardware
use hal::{
    clocks::init_clocks_and_plls, fugit::RateExtU32, pac, sio::Sio, watchdog::Watchdog, Clock,
};

// Some useful core and math functionality
use core::cmp;
use core::cmp::Ordering;
use core::f32::consts;
use micromath::F32;

// made drivers
use foc_port::driver::{self, BLDCDriver};
use foc_port::pid;
use foc_port::sensor::{self, RotarySensor, RotorState};
use foc_port::FOCMotor;
use foc_port::{bldc_motor, sensor::magnetic_i2c};

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
    let sda_pin = pins.gpio0.reconfigure();
    let scl_pin = pins.gpio1.reconfigure();

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
    let pwm0 = &mut pwm_slices.pwm0;
    pwm0.clr_ph_correct();
    pwm0.set_top(0x00ff);
    pwm0.enable();
    let pwm1 = &mut pwm_slices.pwm1;
    pwm1.clr_ph_correct();
    pwm1.set_top(0x00ff);
    pwm1.enable();

    // get PWM channels
    let channel0a = &mut pwm0.channel_a;
    let channel0b = &mut pwm0.channel_b;
    let channel1a = &mut pwm1.channel_a;

    // set the pwm channels to pins
    channel0a.output_to(pins.gpio16);
    channel0b.output_to(pins.gpio17);
    channel1a.output_to(pins.gpio18);

    let mut motor = bldc_motor::BLDCMotor::new(
        bldc_motor::BLDCMotorSpecification {
            pole_pairs: 7,
            kv: 1000,
            phase_resistance: 0.1,
            phase_inductance: 0.1,
        },
        Some(sensor::RotorState::new(
            &timer,
            magnetic_i2c::MageticI2C::new(i2c, magnetic_i2c::AS5600_CONFIG),
        )),
        driver::bldc_driver_3pwm::BLDCDriver3PWM {
            vdc: 7.0,
            a: channel0a,
            b: channel0b,
            c: channel1a,
        },
        pid::PID::new(&timer, 10.0, 100.0, 0.1, 0.0),
    );

    motor
        .angle
        .as_mut()
        .unwrap()
        .set_return_mapping(true, 0.455);

    // motor.calibrate_rotary_sensor();

    info!("Open Loop Testing");
    loop {
        motor.goto_blocking(314.15926);
        info!("SETTLED");
        delay.delay_ms(1000); // Don't comment out this line or RTT blows up
        motor.goto_blocking(-314.15926);
        info!("SETTLED");
        delay.delay_ms(1000); // Don't comment out this line or RTT blows up
    }
}
