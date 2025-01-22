#![no_main]
#![no_std]

use panic_halt as _;

use py32f0xx_hal as hal;

use crate::hal::{pac, prelude::*};

use cortex_m::peripheral::Peripherals;
use cortex_m_rt::entry;
use embedded_hal_02::blocking::delay::DelayMs;

#[entry]
fn main() -> ! {
    let mut p = pac::Peripherals::take().unwrap();
    let cp = Peripherals::take().unwrap();
    let rcc = p.RCC.configure().freeze(&mut p.FLASH);

    let gpioa = p.GPIOA.split();
    let gpiob = p.GPIOB.split();

    // (Re-)configure PA5 as output
    let led1 = gpioa.pa5.into_push_pull_output();
    // (Re-)configure PB1 as output
    let led2 = gpiob.pb1.into_push_pull_output();

    // Get delay provider
    let mut delay = cp.SYST.delay(&rcc.clocks);

    // Store them together after erasing the type
    let mut leds = [led1.erase(), led2.erase()];
    loop {
        for l in &mut leds {
            l.set_high();
        }
        delay.delay_ms(1_000_u16);

        for l in &mut leds {
            l.set_low();
        }
        delay.delay_ms(1_000_u16);
    }
}
