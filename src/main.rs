#![no_std]
#![no_main]

extern crate panic_itm;
use cortex_m::{singleton};
use cortex_m_rt::entry;
use stm32f3_discovery::stm32f3xx_hal as hal;
use hal::{pac, prelude::*, stm32, serial::Serial, rcc};

struct SerialStruct {
    tx: hal::serial::Tx<stm32::USART1>,
    rx: hal::serial::Rx<stm32::USART1>,
    tx_channel: hal::dma::dma1::C4,
    rx_cahnnel: hal::dma::dma1::C5,
    delay: hal::delay::Delay,
}

impl SerialStruct {
    fn new() -> Self {
        // get peripherals
        let device_periphs = pac::Peripherals::take().unwrap();
        let core_periphs = cortex_m::Peripherals::take().unwrap();
        let pac::Peripherals { RCC: destruct_RCC, DMA1: destruct_DMA1, FLASH: destruct_FLASH, GPIOA: destruct_gpioa, .. } = device_periphs;

        // setup rcc
        let rcc = destruct_RCC.constrain();
        let rcc::Rcc { mut ahb, mut apb2, .. } = rcc;

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

        SerialStruct {
            tx: tx,
            rx: rx,
            tx_channel: ch4,
            rx_cahnnel: ch5,
            delay: delay,
        }
    }
}

#[entry]
fn main() -> ! {

    let mut serial_handler = SerialStruct::new();
    let mut initial_buf = singleton!(: [u8; 9] = *b"hi DMA!\r\n").unwrap();

    loop {
        let sending = serial_handler.tx.write_all(initial_buf, serial_handler.tx_channel);
        let (buf, ch, tx) = sending.wait();
        serial_handler.tx_channel = ch;
        serial_handler.tx = tx;
        initial_buf = buf;
        serial_handler.delay.delay_ms(1000u16);
    }
}