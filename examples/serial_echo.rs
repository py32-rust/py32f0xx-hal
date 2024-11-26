#![no_main]
#![no_std]

use core::fmt::Write;

use panic_halt as _;

use py32f0xx_hal as hal;

use crate::hal::{
    pac,
    prelude::*,
    rcc::{HSIFreq, MCODiv, MCOSrc},
};

use cortex_m_rt::entry;
use embedded_hal_02::serial::{Read, Write as OtherWrite};

#[entry]
fn main() -> ! {
    if let Some(p) = pac::Peripherals::take() {
        let mut flash = p.FLASH;
        let rcc = p
            .RCC
            .configure()
            .hsi(HSIFreq::Freq24mhz)
            .sysclk(24.mhz())
            .freeze(&mut flash);

        rcc.configure_mco(MCOSrc::Sysclk, MCODiv::NotDivided);

        let gpioa = p.GPIOA.split();

        let (tx, rx) = (
            gpioa.pa2.into_alternate_af1(),
            gpioa.pa3.into_alternate_af1(),
        );

        let mut serial = p.USART1.serial((tx, rx), 115_200.bps(), &rcc.clocks);
        serial.write_str("Input any key:\n").ok();

        loop {
            // Wait for reception of a single byte
            let received: u8 = nb::block!(serial.read()).unwrap();

            // Send back previously received byte and wait for completion
            nb::block!(serial.write(received)).ok();
        }
    }

    loop {
        continue;
    }
}
