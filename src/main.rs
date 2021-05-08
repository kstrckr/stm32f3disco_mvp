#![no_std]
#![no_main]

extern crate panic_itm;
use cortex_m_rt::entry;
use cortex_m::{iprintln};
use stm32f3_discovery::stm32f3xx_hal::delay::Delay;
use stm32f3_discovery::stm32f3xx_hal::prelude::*;
use stm32f3_discovery::stm32f3xx_hal::stm32;
use stm32f3_discovery::stm32f3xx_hal::i2c::I2c;

use stm32f3_discovery::stm32f3xx_hal::serial::Serial;
use stm32f3_discovery::stm32f3xx_hal::time::Bps;
// use stm32f3_discovery::leds::Leds;
// use stm32f3_discovery::switch_hal::{OutputSwitch, ToggleableOutputSwitch};
use lsm303agr::Lsm303agr;
use lsm303agr::MagOutputDataRate;
use lsm303agr::UnscaledMeasurement;

// use stm32f3_discovery::stm32f3xx_hal::{self as hal, pac, prelude::*};

const X_OFFSET:f32 = -47.0;
const Y_OFFSET:f32 = 222.5;
const Z_OFFSET:f32 = -15.5;

fn apply_calibration_offset(measurement: UnscaledMeasurement) -> (f32, f32, f32) {
    let calibrated_x = measurement.x as f32 - X_OFFSET;
    let calibrated_y = measurement.y as f32 - Y_OFFSET;
    let calibrated_z = measurement.z as f32 - Z_OFFSET;
    (calibrated_x, calibrated_y, calibrated_z)
}

#[entry]
fn main() -> ! {
    let device_periphs = stm32::Peripherals::take().unwrap();
    let mut rcc = device_periphs.RCC.constrain();
    let core_periphs = cortex_m::Peripherals::take().unwrap();
    let mut itm = core_periphs.ITM;
    let mut flash = device_periphs.FLASH.constrain();

    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay = Delay::new(core_periphs.SYST, clocks);

    // i2c setup
    let mut gpiob = device_periphs.GPIOB.split(&mut rcc.ahb);
    let scl = gpiob.pb6.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
    let sda = gpiob.pb7.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
    let i2c = I2c::new(
        device_periphs.I2C1,
        (scl, sda),
        100u32.khz(),
        clocks,
        &mut rcc.apb1
    );
    // end i2c setup

    // serial setup
    let mut gpioa = device_periphs.GPIOA.split(&mut rcc.ahb);
    // for usart1
    // tx = gpioa A9
    // rx = gpioa A10
    let tx_pin = gpioa.pa9.into_af7(&mut gpioa.moder, &mut gpioa.afrh);
    let rx_pin = gpioa.pa10.into_af7(&mut gpioa.moder, &mut gpioa.afrh);

    let serial_pins = Serial::usart1(
        device_periphs.USART1,
        (tx_pin, rx_pin),
        115200u32.bps(),
        clocks,
        &mut rcc.apb2,
    );

    let (tx, rx) = serial_pins.split();

    let dma1 = device_periphs.DMA1.split(&mut rcc.ahb);
    // the data we are going to send over serial
    let val1 = 100u8;
    let val1_bytes = val1.to_le_bytes();
    let tx_buf = cortex_m::singleton!(: [u8; 1] = val1_bytes).unwrap();
    // the buffer we are going to receive the transmitted data in
    // let rx_buf = cortex_m::singleton!(: [u8; 9] = [0; 9]).unwrap();

    // DMA channel selection depends on the peripheral:
    // - USART1: TX = 4, RX = 5
    // - USART2: TX = 6, RX = 7
    // - USART3: TX = 3, RX = 2
    let (tx_channel, rx_channel) = (dma1.ch4, dma1.ch5);

    // start separate DMAs for sending and receiving the data
    let sending = tx.write_all(tx_buf, tx_channel);
    // let receiving = rx.read_exact(rx_buf, rx_channel);

    // block until all data was transmitted and received
    let (tx_buf, tx_channel, tx) = sending.wait();
    // let (rx_buf, rx_channel, rx) = receiving.wait();
    // end serial setup

    // let val2 = 101u8;
    // let val2_bytes = val2.to_le_bytes();
    // tx_buf.copy_from_slice(&val2_bytes);
    // let sending = tx.write_all(tx_buf, tx_channel);
    // let (tx_buf, ..) = sending.wait();

    iprintln!(&mut itm.stim[0], "I2C enabled...");
    let mut sensor = Lsm303agr::new_with_i2c(i2c);
    sensor.init().unwrap();
    sensor.set_mag_odr(MagOutputDataRate::Hz100).unwrap();
    let mut sensor = sensor.into_mag_continuous().ok().unwrap();
    loop {
        // if sensor.mag_status().unwrap().xyz_new_data {
        //     let data = sensor.mag_data().unwrap();
        //     let (x, y, z) = apply_calibration_offset(data);
        //     iprintln!(&mut itm.stim[0], "{}, {}, {}", x, y, z);
        // }
        delay.delay_ms(50u8);
    }
}