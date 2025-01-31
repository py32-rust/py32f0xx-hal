#![no_main]
#![no_std]

use panic_halt as _;

use py32f0xx_hal as hal;

use crate::hal::{
    pac,
    prelude::*,
    spi::{Mode, Phase, Polarity},
};

use cortex_m_rt::entry;

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

    // Configure pins for SPI
    let (sck, miso, mosi) = (
        gpioa.pa5.into_alternate_af0(),
        gpioa.pa6.into_alternate_af0(),
        gpioa.pa7.into_alternate_af0(),
    );

    // Configure SPI with 100kHz rate
    let mut spi = p.SPI1.spi(
        (Some(sck), Some(miso), Some(mosi)),
        MODE,
        100_000.Hz(),
        &rcc.clocks,
    );

    // Cycle through colors on 16 chained APA102C LEDs
    loop {
        for r in 0..255 {
            let _ = spi.write(&[0, 0, 0, 0]);
            for _i in 0..16 {
                let _ = spi.write(&[0b1110_0001, 0, 0, r]);
            }
            let _ = spi.write(&[0xFF, 0xFF, 0xFF, 0xFF]);
        }
        for b in 0..255 {
            let _ = spi.write(&[0, 0, 0, 0]);
            for _i in 0..16 {
                let _ = spi.write(&[0b1110_0001, b, 0, 0]);
            }
            let _ = spi.write(&[0xFF, 0xFF, 0xFF, 0xFF]);
        }
        for g in 0..255 {
            let _ = spi.write(&[0, 0, 0, 0]);
            for _i in 0..16 {
                let _ = spi.write(&[0b1110_0001, 0, g, 0]);
            }
            let _ = spi.write(&[0xFF, 0xFF, 0xFF, 0xFF]);
        }
    }
}
