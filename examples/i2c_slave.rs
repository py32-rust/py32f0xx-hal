#![no_main]
#![no_std]

use panic_halt as _;

use defmt_rtt as _;

use py32f0xx_hal as hal;

use crate::hal::{pac, prelude::*, i2c::SlaveAddressMode};

use cortex_m_rt::entry;
use defmt::{error, info};

#[entry]
fn main() -> ! {
    let p = pac::Peripherals::take().unwrap();
    let mut flash = p.FLASH;
    let mut rcc = p
        .RCC
        .configure()
        .sysclk(24.MHz())
        .pclk(24.MHz())
        .freeze(&mut flash);

    let gpioa = p.GPIOA.split();

    // Configure pins for I2C
    let scl = gpioa.pa3.into_alternate_af12();
    let sda = gpioa.pa2.into_alternate_af12();

    // Configure I2C as slave with 100kHz rate
    let mut i2c = p.I2C.i2c_slave((scl, sda), 100.kHz(), &mut rcc);

    // set the address
    i2c.prepare_wait(0x1c).unwrap();
    info!("Waiting for connection, our i2c addr: 0x1c");

    // wait for connection
    while !i2c.is_address_match().unwrap() {}

    // clear the address match and send back hello
    match i2c.clear_address_match() {
        Ok(SlaveAddressMode::Write) => i2c.slave_write_bytes(b"Hello!").unwrap(),
        Ok(SlaveAddressMode::Read) => error!("Expected to write bytes back to master"),
        Err(e) => error!("Error during address match: {}", e),
    }

    info!("Wrote 'Hello!' back to master");

    loop {
        continue;
    }
}
