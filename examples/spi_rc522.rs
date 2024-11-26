#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

use py32f0xx_hal as hal;

use crate::hal::{
    delay::Delay,
    pac,
    prelude::*,
    spi::Spi,
    spi::{Mode, Phase, Polarity},
    time::Hertz,
    timers::Timer,
};
use cortex_m_rt::entry;
use defmt::{error, info};
use embedded_hal_02::blocking::delay::DelayMs;
use embedded_hal_02::digital::v2::OutputPin;
use embedded_hal_02::timer::CountDown;

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

        let mut delay = Delay::new(cp.SYST, &rcc);

        let gpioa = p.GPIOA.split();

        let (sck, miso, mosi, nss, mut rst) = (
            // SPI pins
            gpioa.pa5.into_alternate_af0(),
            gpioa.pa6.into_alternate_af0(),
            gpioa.pa7.into_alternate_af0(),
            // Aux pins
            gpioa.pa4.into_push_pull_output().downgrade(),
            gpioa.pa1.into_push_pull_output().downgrade(),
        );
        rst.set_low().ok();

        // Configure SPI with 1MHz rate
        let spi = Spi::new(
            p.SPI1,
            (Some(sck), Some(miso), Some(mosi)),
            MODE,
            1.mhz(),
            &rcc.clocks,
        );
        let itf = SpiInterface::new(spi).with_nss(nss).with_delay(|| {
            delay.delay_ms(1_u16);
        });
        rst.set_high().ok();

        let mut mfrc522 = Mfrc522::new(itf).init().unwrap();

        let ver = mfrc522.version().unwrap();
        info!("MFRC522 version: 0x{:02x}", ver);
        assert!(ver == 0x91 || ver == 0x92);

        let mut timer = Timer::tim1(p.TIM1, Hertz(1), &rcc.clocks);

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
