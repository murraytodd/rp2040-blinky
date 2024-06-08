//! Blinks the LED on a Pico board
//!
//! This will blink an LED attached to GP22,
#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use embedded_hal::digital::OutputPin;
// Embedded HAL 1.0.0 doesn't have an ADC trait, so use the one from 0.2
use embedded_hal_0_2::adc::OneShot;

use mcp9808::{
    reg_conf::Configuration, reg_res::ResolutionVal, reg_temp_generic::ReadableTempRegister,
    MCP9808,
};
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
use rp_pico as bsp;

use bsp::hal::{
    adc::AdcPin,
    clocks::{init_clocks_and_plls, Clock},
    fugit::RateExtU32,
    gpio::{FunctionI2C, Pin, PullUp},
    pac,
    sio::Sio,
    watchdog::Watchdog,
};

fn tmp36_f(adc_reading: u16) -> f32 {
    let voltage: f32 = adc_reading as f32 * 3.3 / 4096.0;
    let c = (100.0 * voltage) - 50.0;
    return (c * 9.0 / 5.0) + 32.0;
}

fn chip_f(adc_reading: u16) -> f32 {
    let voltage: f32 = adc_reading as f32 * 3.3 / 4096.0;
    let c: f32 = 27.0 - ((voltage - 0.706) / 0.001721);
    return (c * 9.0 / 5.0) + 32.0;
}

#[entry]
fn main() -> ! {
    info!("Program start");
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

    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let sda_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio16.reconfigure();
    let scl_pin: Pin<_, FunctionI2C, PullUp> = pins.gpio17.reconfigure();
    let i2c = bsp::hal::I2C::i2c0(
        pac.I2C0,
        sda_pin,
        scl_pin,
        400.kHz(),
        &mut pac.RESETS,
        &clocks.system_clock,
    );

    let mut led_pin = pins.gpio22.into_push_pull_output();

    let mut mcp9808 = MCP9808::new(i2c);
    let mut mcp9808_conf = mcp9808.read_configuration().unwrap();
    mcp9808_conf.set_shutdown_mode(mcp9808::reg_conf::ShutdownMode::Shutdown);
    mcp9808.write_register(mcp9808_conf).unwrap();

    let mut adc = bsp::hal::Adc::new(pac.ADC, &mut pac.RESETS);
    let mut rp2040_temp_sensor = adc.take_temp_sensor().unwrap();
    let mut adc_pin_0 = AdcPin::new(pins.gpio26).unwrap();

    // let chip_temp = adc.read()

    loop {
        info!("on!");
        led_pin.set_high().unwrap();
        delay.delay_ms(500);
        info!("off!");
        led_pin.set_low().unwrap();
        delay.delay_ms(500);

        let temp = mcp9808.read_temperature().unwrap();
        let reading: f32 = temp.get_celsius(ResolutionVal::Deg_0_0625C);
        let ftemp: f32 = (reading * (9.0 / 5.0)) + 32.0;
        info!("Temperature reading: {}℉", ftemp);

        // note: top voltage = 4096 because readings are 12-bit
        let chip_temp: u16 = adc.read(&mut rp2040_temp_sensor).unwrap();
        let tmp36_temp: u16 = adc.read(&mut adc_pin_0).unwrap();
        info!("TMP36 readering: {}℉", tmp36_f(tmp36_temp));
        info!("Chip Temp reading: {}℉", chip_f(chip_temp));
    }
}

// End of file
