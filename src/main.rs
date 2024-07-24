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
    let _pins = rp2040_hal::gpio::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    info!("Hello, World");

    loop {
        delay.delay_ms(500); // Don't comment out this line or RTT blows up
        info!("yeah");
        delay.delay_ms(500);
        info!("nooo");
    }
}
