#![no_main]
#![no_std]

use panic_halt as _;

use defmt_rtt as _;

use py32f0xx_hal as hal;

use crate::hal::{i2c::I2c, pac, prelude::*};

use cortex_m_rt::entry;
use defmt::info;
use embedded_hal_02::blocking::i2c::Write;

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();
    let mut flash = p.FLASH;
    let rcc = p
        .RCC
        .configure()
        .sysclk(24.MHz())
        .pclk(24.MHz())
        .freeze(&mut flash);

    let gpioa = p.GPIOA.split();

    // Configure pins for I2C
    let scl = gpioa.pa3.into_alternate_af12();
    let sda = gpioa.pa2.into_alternate_af12();

    // Configure I2C with 100kHz rate
    let mut i2c = I2c::i2c(p.I2C, (scl, sda), 100.kHz(), &rcc.clocks);

    let mut devices = 0;
    // I2C addresses are 7-bit wide, covering the 0-127 range
    for add in 0..=127 {
        // The write method sends the specified address and checks for acknowledgement;
        // if no ack is given by the slave device the result is Err(), otherwise Ok()
        // Since we only care for an acknowledgement the data sent can be empty
        if i2c.write(add, &[]).is_ok() {
            devices += 1;
        }
    }

    // Here the variable "_devices" counts how many i2c addresses were a hit
    info!("{} devices find.\r\n", devices);

    loop {
        continue;
    }
}
