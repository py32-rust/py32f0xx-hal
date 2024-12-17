#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

use py32f0xx_hal as hal;

use crate::hal::{
    pac,
    prelude::*,
    spi::{Mode, Phase, Polarity},
};
use cortex_m_rt::entry;
use defmt::{error, info};
use embedded_hal_02::blocking::delay::DelayMs;

use mfrc522::comm::eh02::spi::SpiInterface;
use mfrc522::Mfrc522;

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

    if let (Some(p), Some(cp)) = (
        pac::Peripherals::take(),
        cortex_m::peripheral::Peripherals::take(),
    ) {
        let mut flash = p.FLASH;
        let rcc = p.RCC.configure().freeze(&mut flash);

        let mut delay = cp.SYST.delay(&rcc.clocks);

        let gpioa = p.GPIOA.split();

        let (sck, miso, mosi, nss, mut rst) = (
            // SPI pins
            gpioa.pa5.into_alternate_af0(),
            gpioa.pa6.into_alternate_af0(),
            gpioa.pa7.into_alternate_af0(),
            // Aux pins
            gpioa.pa4.into_push_pull_output(),
            gpioa.pa1.into_push_pull_output(),
        );
        rst.set_low();

        // Configure SPI with 1MHz rate
        let spi = p.SPI1.spi(
            (Some(sck), Some(miso), Some(mosi)),
            MODE,
            1.MHz().into(),
            &rcc.clocks,
        );
        let itf = SpiInterface::new(spi).with_nss(nss).with_delay(|| {
            delay.delay_ms(1_u16);
        });
        rst.set_high();

        let mut mfrc522 = Mfrc522::new(itf).init().unwrap();

        let ver = mfrc522.version().unwrap();
        info!("MFRC522 version: 0x{:02x}", ver);
        assert!(ver == 0x91 || ver == 0x92);

        let mut timer = p.TIM1.counter_hz(&rcc.clocks);
        timer.start(1.Hz()).unwrap();

        loop {
            info!("Waiting for card...");
            match mfrc522.reqa() {
                Ok(atqa) => {
                    if let Ok(uid) = mfrc522.select(&atqa) {
                        info!("Selected card, UID: {:02X}", uid.as_bytes());
                    } else {
                        error!("Failed to select card");
                    }
                }
                Err(_e) => error!("Error when requesting ATQA!"),
            }

            nb::block!(timer.wait()).unwrap();
        }
    }

    loop {
        continue;
    }
}
