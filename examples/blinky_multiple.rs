#![no_main]
#![no_std]

use panic_halt as _;

use py32f0xx_hal as hal;

use crate::hal::{delay::Delay, pac, prelude::*};

use cortex_m::peripheral::Peripherals;
use cortex_m_rt::entry;
use embedded_hal::digital::OutputPin;
use embedded_hal_02::blocking::delay::DelayMs;

#[entry]
fn main() -> ! {
    if let (Some(mut p), Some(cp)) = (pac::Peripherals::take(), Peripherals::take()) {
        let rcc = p.RCC.configure().sysclk(8.mhz()).freeze(&mut p.FLASH);

        let gpioa = p.GPIOA.split();
        let gpiob = p.GPIOB.split();

        // (Re-)configure PA5 as output
        let led1 = gpioa.pa5.into_push_pull_output();
        // (Re-)configure PB1 as output
        let led2 = gpiob.pb1.into_push_pull_output();

        // Get delay provider
        let mut delay = Delay::new(cp.SYST, &rcc);

        // Store them together after erasing the type
        let mut leds = [led1.downgrade(), led2.downgrade()];
        loop {
            for l in &mut leds {
                l.set_high().ok();
            }
            delay.delay_ms(1_000_u16);

            for l in &mut leds {
                l.set_low().ok();
            }
            delay.delay_ms(1_000_u16);
        }
    }

    loop {
        continue;
    }
}
