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

    // (Re-)configure PA5 as output
    let mut led = gpioa.pa5.into_push_pull_output();

    // Get delay provider
    let mut delay = cp.SYST.delay(&rcc.clocks);

    loop {
        led.toggle();
        delay.delay_ms(1_000_u16);
    }
}
