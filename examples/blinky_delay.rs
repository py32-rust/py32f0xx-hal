#![no_main]
#![no_std]

use panic_halt as _;

use py32f0xx_hal as hal;

use crate::hal::{delay::Delay, pac, prelude::*};
use cortex_m::peripheral::Peripherals;
use cortex_m_rt::entry;
use embedded_hal_02::blocking::delay::DelayMs;
use embedded_hal_02::digital::v2::ToggleableOutputPin;

#[entry]
fn main() -> ! {
    if let (Some(mut p), Some(cp)) = (pac::Peripherals::take(), Peripherals::take()) {
        let rcc = p.RCC.configure().sysclk(8.mhz()).freeze(&mut p.FLASH);

        let gpioa = p.GPIOA.split();

        // (Re-)configure PA5 as output
        let mut led = gpioa.pa5.into_push_pull_output().downgrade();

        // Get delay provider
        let mut delay = Delay::new(cp.SYST, &rcc);

        loop {
            led.toggle().ok();
            delay.delay_ms(1_000_u16);
        }
    }

    loop {
        continue;
    }
}
