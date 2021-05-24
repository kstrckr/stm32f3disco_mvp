#![no_std]
#![no_main]
#![cfg_attr(not(test), no_std)]

extern crate panic_itm;
use cortex_m::{singleton};
use cortex_m_rt::entry;
use stm32f3_discovery::stm32f3xx_hal as hal;
use hal::{pac, prelude::*, stm32, serial::Serial, rcc, i2c::I2c, dma::ReadBuffer};
use lsm303agr::{Lsm303agr, MagOutputDataRate, UnscaledMeasurement};

const X_OFFSET:f32 = -47.0;
const Y_OFFSET:f32 = 222.5;
const Z_OFFSET:f32 = -15.5;

fn apply_calibration_offset(measurement: UnscaledMeasurement) -> (f32, f32, f32) {
    let calibrated_x = measurement.x as f32 - X_OFFSET;
    let calibrated_y = measurement.y as f32 - Y_OFFSET;
    let calibrated_z = measurement.z as f32 - Z_OFFSET;
    (calibrated_x, calibrated_y, calibrated_z)
}

struct IOStruct {
    tx: hal::serial::Tx<stm32::USART1>,
    rx: hal::serial::Rx<stm32::USART1>,
    tx_channel: hal::dma::dma1::C4,
    rx_cahnnel: hal::dma::dma1::C5,
    delay: hal::delay::Delay,
    magnetometer: lsm303agr::Lsm303agr<lsm303agr::interface::I2cInterface<stm32f3_discovery::stm32f3xx_hal::i2c::I2c<stm32f3_discovery::stm32f3xx_hal::pac::I2C1, (stm32f3_discovery::stm32f3xx_hal::gpio::gpiob::PB6<stm32f3_discovery::stm32f3xx_hal::gpio::AF4>, stm32f3_discovery::stm32f3xx_hal::gpio::gpiob::PB7<stm32f3_discovery::stm32f3xx_hal::gpio::AF4>)>>, lsm303agr::mode::MagContinuous>,
}

impl IOStruct {
    fn new() -> Self {
        // get peripherals
        let device_periphs = pac::Peripherals::take().unwrap();
        let core_periphs = cortex_m::Peripherals::take().unwrap();
        let pac::Peripherals { RCC: destruct_RCC, DMA1: destruct_DMA1, FLASH: destruct_FLASH, GPIOA: destruct_gpioa, GPIOB: destruct_gpiob, .. } = device_periphs;

        // setup rcc
        let rcc = destruct_RCC.constrain();
        let rcc::Rcc { mut ahb, mut apb2, mut apb1, .. } = rcc;

        // get dma1
        let dma1 = destruct_DMA1.split(&mut ahb);
        let  hal::dma::dma1::Channels { ch4, ch5, ..} = dma1;

        let mut flash = destruct_FLASH.constrain();
        let mut gpioa = destruct_gpioa.split(&mut ahb);

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
        }
    }
}

fn format_data(x_data: [u8; 4], y_data: [u8; 4], z_data: [u8; 4]) -> [u8; 12] {
    [x_data[0], x_data[1], x_data[2], x_data[3], y_data[0], y_data[1], y_data[2], y_data[3], z_data[0], z_data[1], z_data[2], z_data[3]]
    // [x_data[0], x_data[1], x_data[2], x_data[3]]
}

fn cobs_encode(all_data_bytes: [u8; 12]) -> [u8; 14] {
    let mut encoded = [0; 14];
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

#[entry]
fn main() -> ! {

    let mut io_struct = IOStruct::new();
    let mut initial_buf = singleton!(: [u8; 14] = [0; 14]).unwrap();

    loop {
        // io_struct.delay.delay_ms(100u16);
        // let (buf, ch, tx) = io_struct.tx.write_all(initial_buf, io_struct.tx_channel).wait();
        // io_struct.tx_channel = ch;
        // io_struct.tx = tx;
        // initial_buf = buf;

        if io_struct.magnetometer.mag_status().unwrap().xyz_new_data {
            let data = io_struct.magnetometer.mag_data().unwrap();
            let (x, y, z) = apply_calibration_offset(data);
            let x_bytes = x.to_le_bytes();
            let y_bytes = y.to_le_bytes();
            let z_bytes = z.to_le_bytes();

            let formatted_data = format_data(x_bytes, y_bytes, z_bytes);
            let framed_data = cobs_encode(formatted_data);
            initial_buf.copy_from_slice(&framed_data);
            let sending = io_struct.tx.write_all(initial_buf, io_struct.tx_channel);
            let (buf, ch, tx) = sending.wait();
            io_struct.tx_channel = ch;
            io_struct.tx = tx;
            initial_buf = buf;
            // io_struct.delay.delay_ms(50u16);
        }
    }
}