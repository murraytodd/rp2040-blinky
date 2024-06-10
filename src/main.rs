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

const REFERENCE_VOLTAGE: f32 = 3.3;
const STEPS_12BIT: f32 = 4096 as f32;

/// Basic Celsius-to-Fahrenheit conversion
fn c_to_f(c: f32) -> f32 {
    (c * 9.0 / 5.0) + 32.0
}

/// Convert ADC binary value to a float voltage value.
///
/// The ADC has a 12-bit resolution of voltage, meaning that there
/// are 2^12 or 4096 unique levels from OFF (0V) to FULL (3V). This
/// function converts the ADC reading into a float measurement in volts.
fn adc_reading_to_voltage(reading_12bit: u16) -> f32 {
    (reading_12bit as f32 / STEPS_12BIT) * REFERENCE_VOLTAGE
}

/// Convert the voltage from a TMP36 sensor into a temperature reading.
///
/// The sensor returns 0.5V at 0°C and voltage changes ±0.01V for every
/// degree Celcius with higher temps resolting in higher voltages within
/// the range of -40°C to 125°C.
fn tmp36_f(adc_reading: u16) -> f32 {
    let voltage: f32 = adc_reading_to_voltage(adc_reading);
    let c = (100.0 * voltage) - 50.0;
    c_to_f(c)
}

/// Convert the voltage from the onboard temp sensor into a temp reading.
///
/// From §4.9.5 from the rp2040-datasheet.pdf, the temperature can be
/// approximated as T = 27 - (ADC_voltage - 0.706) / 0.001721.
fn chip_f(adc_reading: u16) -> f32 {
    let voltage: f32 = adc_reading_to_voltage(adc_reading);
    let c: f32 = 27.0 - ((voltage - 0.706) / 0.001721);
    c_to_f(c)
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
    mcp9808_conf.set_shutdown_mode(mcp9808::reg_conf::ShutdownMode::Continuous);
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

        let mcp9808_reading_c: f32 = mcp9808
            .read_temperature()
            .unwrap()
            .get_celsius(ResolutionVal::Deg_0_0625C);
        let chip_voltage_24bit: u16 = adc.read(&mut rp2040_temp_sensor).unwrap();
        let tmp36_voltage_24bit: u16 = adc.read(&mut adc_pin_0).unwrap();
        info!(
            "Temp readings:  MCP9808: {}°F, TMP36: {}°F, OnChip: {}°F",
            c_to_f(mcp9808_reading_c),
            chip_f(chip_voltage_24bit),
            tmp36_f(tmp36_voltage_24bit)
        );
    }
}

// End of file
