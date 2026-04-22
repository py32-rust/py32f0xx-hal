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
    let mut rcc = p.RCC.configure().freeze(&mut p.FLASH);

    let gpioa = p.GPIOA.split(&mut rcc);

    // (Re-)configure PA5 as output
    let mut led = gpioa.pa5.into_push_pull_output();

    // Get delay provider
    let mut delay = cp.SYST.delay(&mut rcc);

    loop {
        led.toggle();
        delay.delay_ms(1_000_u16);
    }
}
