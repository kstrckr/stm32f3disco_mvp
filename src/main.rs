#![no_std]
#![no_main]
#![cfg_attr(not(test), no_std)]

extern crate panic_itm;
use cortex_m::{singleton};
use cortex_m_rt::entry;
use stm32f3_discovery::stm32f3xx_hal as hal;

use hal::{pac, prelude::*, stm32, serial::Serial, rcc, i2c::I2c, dma::ReadBuffer};
use lsm303agr::{Lsm303agr, MagOutputDataRate, UnscaledMeasurement};

use stm32f3_discovery::stm32f3xx_hal::interrupt;
use stm32f3_discovery::wait_for_interrupt;
use stm32f3_discovery::button;
use stm32f3_discovery::button::interrupt::TriggerMode;
use stm32f3_discovery::leds::Leds;
use stm32f3_discovery::switch_hal::ToggleableOutputSwitch;

use core::sync::atomic::{AtomicBool, Ordering};

static USER_BUTTON_PRESSED: AtomicBool = AtomicBool::new(false);

const X_OFFSET:f32 = -47.0;
const Y_OFFSET:f32 = 222.5;
const Z_OFFSET:f32 = -15.5;

fn apply_calibration_offset(measurement: UnscaledMeasurement) -> (f32, f32, f32) {
    let calibrated_x = measurement.x as f32 - X_OFFSET;
    let calibrated_y = measurement.y as f32 - Y_OFFSET;
    let calibrated_z = measurement.z as f32 - Z_OFFSET;
    (calibrated_x, calibrated_y, calibrated_z)
}

fn smooth(new: f32, smoothed: f32) -> f32 {
    smoothed + ((new - smoothed + 0.5) / 5.0)
}

struct IOStruct {
    tx: hal::serial::Tx<stm32::USART1>,
    rx: hal::serial::Rx<stm32::USART1>,
    tx_channel: hal::dma::dma1::C4,
    rx_cahnnel: hal::dma::dma1::C5,
    delay: hal::delay::Delay,
    magnetometer: lsm303agr::Lsm303agr<lsm303agr::interface::I2cInterface<stm32f3_discovery::stm32f3xx_hal::i2c::I2c<stm32f3_discovery::stm32f3xx_hal::pac::I2C1, (stm32f3_discovery::stm32f3xx_hal::gpio::gpiob::PB6<stm32f3_discovery::stm32f3xx_hal::gpio::AF4>, stm32f3_discovery::stm32f3xx_hal::gpio::gpiob::PB7<stm32f3_discovery::stm32f3xx_hal::gpio::AF4>)>>, lsm303agr::mode::MagContinuous>,
    gpioe: hal::gpio::gpioe::Parts,
    exti: hal::pac::EXTI,
    syscfg: hal::pac::SYSCFG,
}

impl IOStruct {
    fn new() -> Self {
        // get peripherals
        let device_periphs = pac::Peripherals::take().unwrap();
        let core_periphs = cortex_m::Peripherals::take().unwrap();
        let pac::Peripherals {
            RCC: destruct_RCC,
            DMA1: destruct_DMA1,
            FLASH: destruct_FLASH,
            GPIOA: destruct_gpioa,
            GPIOB: destruct_gpiob,
            GPIOE: destruct__gpioe,
            EXTI: destruct_exti,
            SYSCFG: destruct_syscfg,
            .. } = device_periphs;


        // setup rcc
        let rcc = destruct_RCC.constrain();
        let rcc::Rcc { mut ahb, mut apb2, mut apb1, .. } = rcc;



        // get dma1
        let dma1 = destruct_DMA1.split(&mut ahb);
        let  hal::dma::dma1::Channels { ch4, ch5, ..} = dma1;

        let mut flash = destruct_FLASH.constrain();
        let mut gpioa = destruct_gpioa.split(&mut ahb);
        let mut gpioe = destruct__gpioe.split(&mut ahb);

        let clocks = rcc.cfgr.freeze(&mut flash.acr);
        let delay = hal::delay::Delay::new(core_periphs.SYST, clocks);

        let pins = (
            gpioa.pa9.into_af7(&mut gpioa.moder, &mut gpioa.afrh),
            gpioa.pa10.into_af7(&mut gpioa.moder, &mut gpioa.afrh),
        );

        let serial = Serial::usart1(
            device_periphs.USART1,
            pins,
            115200.bps(),
            clocks,
            &mut apb2);

        let (tx, rx) = serial.split();

        let mut gpiob = destruct_gpiob.split(&mut ahb);
        let scl = gpiob.pb6.into_af4(&mut gpiob.moder, &mut gpiob.afrl);
        let sda = gpiob.pb7.into_af4(&mut gpiob.moder, &mut gpiob.afrl);


        let i2c = I2c::new(
            device_periphs.I2C1,
            (scl, sda),
            100u32.khz(),
            clocks,
            &mut apb1
        );

        let mut sensor = Lsm303agr::new_with_i2c(i2c);
        sensor.init().unwrap();
        sensor.set_mag_odr(MagOutputDataRate::Hz100).unwrap();
        let sensor = sensor.into_mag_continuous().ok().unwrap();

        IOStruct {
            tx: tx,
            rx: rx,
            tx_channel: ch4,
            rx_cahnnel: ch5,
            delay: delay,
            magnetometer: sensor,
            gpioe: gpioe,
            exti: destruct_exti,
            syscfg: destruct_syscfg,
        }
    }
}

fn format_data(count: u32, x_data: [u8; 4], y_data: [u8; 4], z_data: [u8; 4]) -> [u8; 16] {
    let count_bytes = count.to_le_bytes();
    [x_data[0], x_data[1], x_data[2], x_data[3], y_data[0], y_data[1], y_data[2], y_data[3], z_data[0], z_data[1], z_data[2], z_data[3], count_bytes[0], count_bytes[1], count_bytes[2], count_bytes[3]]
    // [x_data[0], x_data[1], x_data[2], x_data[3]]
}

fn cobs_encode(all_data_bytes: [u8; 16]) -> [u8; 18] {
    let mut encoded = [0; 18];
    let mut prev_zero_index: Option<usize> = None;

    if all_data_bytes[0] == 0 {
        encoded[0] = 1;
    }

    for i in 0..all_data_bytes.len() {
        match all_data_bytes[i] {
            0 => {
                match prev_zero_index {
                    None => {
                        prev_zero_index = Some(i);
                        encoded[0] = i as u8 + 1;
                    },
                    Some(v) => {
                        let offset: usize = i - v;
                        encoded[v + 1] = offset as u8;
                        prev_zero_index = Some(i);
                    }
                }
            }
            _ => {
                encoded[i + 1] = all_data_bytes[i];
            }
        }

        if i == all_data_bytes.len() - 1 {
            let mut final_index = prev_zero_index.unwrap_or(0);
            match final_index {
                0 => {
                    let offset: usize = encoded.len() - 1 - final_index;
                    encoded[final_index] = offset as u8;
                },
                _ => {
                    final_index += 1;
                    let offset: usize = encoded.len() - 1 - final_index;
                    encoded[final_index] = offset as u8;
                }
            }

        }
    }
    encoded
}

#[interrupt]
fn EXTI0() {
    //If we don't clear the interrupt to signal it's been serviced, it will continue to fire.
    button::interrupt::clear();
    // pa0 has a low pass filter on it, so no need to debounce in software
    USER_BUTTON_PRESSED.store(true, Ordering::Relaxed);
}

#[entry]
fn main() -> ! {

    let mut io_struct = IOStruct::new();

    // setup button
    button::interrupt::enable(
        &io_struct.exti,
        &io_struct.syscfg,
        TriggerMode::Rising,
    );

    // setup leds;
    let leds = Leds::new(
        io_struct.gpioe.pe8,
        io_struct.gpioe.pe9,
        io_struct.gpioe.pe10,
        io_struct.gpioe.pe11,
        io_struct.gpioe.pe12,
        io_struct.gpioe.pe13,
        io_struct.gpioe.pe14,
        io_struct.gpioe.pe15,
        &mut io_struct.gpioe.moder,
        &mut io_struct.gpioe.otyper,
    );
    let mut status_led = leds.ld3;


    let mut initial_buf = singleton!(: [u8; 18] = [0; 18]).unwrap();

    let max_measurements = 1000;
    let mut count:u32 = 0;
    let mut smoothed_x = 0.0;
    let mut smoothed_y = 0.0;
    let mut smoothed_z = 0.0;

    let mut reading = false;

    loop {
        // io_struct.delay.delay_ms(100u16);

        if reading {

            let (buf, ch, tx) = io_struct.tx.write_all(initial_buf, io_struct.tx_channel).wait();
            io_struct.tx_channel = ch;
            io_struct.tx = tx;
            initial_buf = buf;
            io_struct.delay.delay_ms(5u16);
            if io_struct.magnetometer.mag_status().unwrap().xyz_new_data {
                let data = io_struct.magnetometer.mag_data().unwrap();
                count += 1;
                let (x, y, z) = apply_calibration_offset(data);
                smoothed_x = smooth(x, smoothed_x);
                smoothed_y = smooth(y, smoothed_y);
                smoothed_z = smooth(z, smoothed_z);
                let x_bytes = smoothed_x.to_le_bytes();
                let y_bytes = smoothed_y.to_le_bytes();
                let z_bytes = smoothed_z.to_le_bytes();

                let formatted_data = format_data(count, x_bytes, y_bytes, z_bytes);
                let framed_data = cobs_encode(formatted_data);
                initial_buf.copy_from_slice(&framed_data);
                let sending = io_struct.tx.write_all(initial_buf, io_struct.tx_channel);
                let (buf, ch, tx) = sending.wait();
                io_struct.tx_channel = ch;
                io_struct.tx = tx;
                initial_buf = buf;
            }
        }
        // check to see if flag was active and clear it
        if USER_BUTTON_PRESSED.swap(false, Ordering::AcqRel) {
            status_led.toggle().ok();
            reading = !reading;
        }

        // wait_for_interrupt();
    }
}