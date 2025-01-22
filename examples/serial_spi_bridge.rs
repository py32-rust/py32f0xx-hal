#![no_main]
#![no_std]

use panic_halt as _;

use py32f0xx_hal as hal;

use crate::hal::{
    pac,
    prelude::*,
    spi::{Mode, Phase, Polarity},
};

use nb::block;

use cortex_m_rt::entry;
use embedded_hal::spi::SpiBus;

/// A basic serial to spi example
///
/// If you connect MOSI & MISO pins together, you'll see all characters
/// that you typed in your serial terminal echoed back
///
/// If you connect MISO to GND, you'll see nothing coming back
#[entry]
fn main() -> ! {
    const MODE: Mode = Mode {
        polarity: Polarity::IdleHigh,
        phase: Phase::CaptureOnSecondTransition,
    };

    let p = pac::Peripherals::take().unwrap();
    let mut flash = p.FLASH;
    let rcc = p.RCC.configure().freeze(&mut flash);

    let gpioa = p.GPIOA.split();

    let (sck, miso, mosi, tx, rx) = (
        // SPI pins
        gpioa.pa5.into_alternate_af0(),
        gpioa.pa6.into_alternate_af0(),
        gpioa.pa7.into_alternate_af0(),
        // USART pins
        gpioa.pa2.into_alternate_af1(),
        gpioa.pa3.into_alternate_af1(),
    );

    // Configure SPI with 1MHz rate
    let mut spi = p.SPI1.spi(
        (Some(sck), Some(miso), Some(mosi)),
        MODE,
        1.MHz(),
        &rcc.clocks,
    );

    let mut serial = p.USART1.serial((tx, rx), 115_200.bps(), &rcc.clocks);

    let mut datatx = [0];
    let datarx = [0];

    loop {
        let serial_received = block!(serial.rx.read()).unwrap();
        spi.write(&[serial_received]).ok();
        spi.transfer(&mut datatx, &datarx).unwrap();
        block!(serial.tx.write_u8(datarx[0])).ok();
    }
}
