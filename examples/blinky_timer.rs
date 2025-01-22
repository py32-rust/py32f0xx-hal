#![no_main]
#![no_std]

use panic_halt as _;

use py32f0xx_hal as hal;

use crate::hal::{pac, prelude::*};

use cortex_m_rt::entry;

#[entry]
fn main() -> ! {
    let mut p = pac::Peripherals::take().unwrap();
    let rcc = p.RCC.configure().freeze(&mut p.FLASH);

    let gpioa = p.GPIOA.split();

    // (Re-)configure PA5 as output
    let mut led = gpioa.pa5.into_push_pull_output();

    // Set up a timer expiring after 200ms
    let mut timer = p.TIM1.counter_hz(&rcc.clocks);
    timer.start(5.Hz()).unwrap();

    loop {
        led.toggle();

        // Wait for the timer to expire
        nb::block!(timer.wait()).ok();
    }
}
