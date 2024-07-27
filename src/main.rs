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
use embedded_hal::digital::OutputPin;
// specify the board
use rp2040_hal as hal;
// access the hardware
use hal::{
    clocks::init_clocks_and_plls, fugit::RateExtU32, pac, sio::Sio, watchdog::Watchdog, Clock,
};

// other drivers
use as5600::As5600;

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

    // setup i2c
    // Configure two pins as being I²C, not GPIO
    let sda_pin = pins.gpio0.reconfigure();
    let scl_pin = pins.gpio1.reconfigure();
    // let not_an_scl_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio20.reconfigure();

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

    let mut as5600 = As5600::new(i2c);
    // wait a little for setting up?
    delay.delay_ms(2000);

    let agc = as5600.automatic_gain_control().unwrap();
    let mag = as5600.magnitude().unwrap();
    let zmco = as5600.zmco().unwrap();

    info!("{:?}", agc);
    info!("{:?}", mag);
    info!("{:?}", zmco);

    delay.delay_ms(2000);
    info!("Hello, World");
    loop {
        let value = as5600.angle().unwrap();
        println!("{:?}", value);

        delay.delay_ms(10); // Don't comment out this line or RTT blows up
    }
}
