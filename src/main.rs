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

use core::cmp;
use core::cmp::Ordering;

use bldc::BLDCDriver;
// some debug stuff
use cortex_m_rt::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

// the shared library of all embeded systems.
use embedded_hal::{delay::DelayNs, digital::OutputPin};
// specify the board
use rp2040_hal as hal;
// access the hardware
use hal::{
    clocks::init_clocks_and_plls, fugit::RateExtU32, pac, sio::Sio, watchdog::Watchdog, Clock,
};

// other drivers
use as5600::As5600;
mod bldc;

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
    let _timer: hal::Timer = hal::Timer::new(pac.TIMER, &mut pac.RESETS, &clocks);
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    // get pins and setting up the external harware.
    let pins = rp2040_hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    // Init PWMs
    let mut pwm_slices = hal::pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    // Configure PWM slices
    let pwm0 = &mut pwm_slices.pwm0;
    pwm0.clr_ph_correct();
    pwm0.set_top(0x0fff);
    pwm0.enable();
    let pwm1 = &mut pwm_slices.pwm1;
    pwm1.clr_ph_correct();
    pwm1.set_top(0x0fff);
    pwm1.enable();

    // get PWM channels
    let channel0a = &mut pwm0.channel_a;
    let channel0b = &mut pwm0.channel_b;
    let channel1a = &mut pwm1.channel_a;

    // set the pwm channels to pins
    channel0a.output_to(pins.gpio16);
    channel0b.output_to(pins.gpio17);
    channel1a.output_to(pins.gpio18);

    // put in the motor specifications, not used for now.
    let motor_specification = bldc::BLDCMotor {
        pole_pairs: 4,
        kv: 1000,
        phase_resistance: 0.1667,
    };

    // give the pwm channels to a motor driver.
    let mut motor_driver = bldc::BLDCDriver3PWM {
        motor_specification: motor_specification,
        a: channel0a,
        b: channel0b,
        c: channel1a,
    };

    let mut led_pin = pins.gpio15.into_push_pull_output();

    let mut toggle = true;

    let mut phase: u16 = 0;
    loop {
        toggle = !toggle;
        if toggle {
            led_pin.set_high().unwrap();
        } else {
            led_pin.set_low().unwrap();
        }

        motor_driver.set_phase(phase);
        phase = (phase + 1) % 360;

        delay.delay_us(100); // Don't comment out this line or RTT blows up
    }
}
