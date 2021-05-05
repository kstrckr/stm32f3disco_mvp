#![no_std]
#![no_main]

extern crate panic_itm;

use cortex_m_rt::entry;
use cortex_m::{iprint, iprintln, peripheral::ITM};
use stm32f3_discovery::stm32f3xx_hal::delay::Delay;
use stm32f3_discovery::stm32f3xx_hal::prelude::*;
use stm32f3_discovery::stm32f3xx_hal::stm32;
use stm32f3_discovery::stm32f3xx_hal::i2c::I2c;

use stm32f3_discovery::leds::Leds;
use stm32f3_discovery::switch_hal::{OutputSwitch, ToggleableOutputSwitch};
use lsm303agr::Lsm303agr;
use lsm303agr::MagOutputDataRate;

// use stm32f3_discovery::stm32f3xx_hal::{self as hal, pac, prelude::*};

#[entry]
fn main() -> ! {
    let device_periphs = stm32::Peripherals::take().unwrap();
    let mut rcc = device_periphs.RCC.constrain();

    let core_periphs = cortex_m::Peripherals::take().unwrap();
    let mut itm = core_periphs.ITM;
    let mut flash = device_periphs.FLASH.constrain();
    let clocks = rcc.cfgr.freeze(&mut flash.acr);
    let mut delay = Delay::new(core_periphs.SYST, clocks);
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

    iprintln!(&mut itm.stim[0], "I2C enabled...");
    let mut sensor = Lsm303agr::new_with_i2c(i2c);
    sensor.init().unwrap();
    sensor.set_mag_odr(MagOutputDataRate::Hz100).unwrap();
    let mut sensor = sensor.into_mag_continuous().ok().unwrap();
    loop {
        if sensor.mag_status().unwrap().xyz_new_data {
            let data = sensor.mag_data().unwrap();
            iprintln!(&mut itm.stim[0], "{}, {}, {}", data.x, data.y, data.z)
        }
        delay.delay_ms(1000u16);
    }
}