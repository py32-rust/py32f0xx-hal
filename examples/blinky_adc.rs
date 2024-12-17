#![no_main]
#![no_std]

use panic_halt as _;

use py32f0xx_hal as hal;

use crate::hal::{adc::Adc, pac, prelude::*};

use cortex_m::peripheral::Peripherals;
use cortex_m_rt::entry;
use embedded_hal_02::adc::OneShot;
use embedded_hal_02::blocking::delay::DelayMs;

#[entry]
fn main() -> ! {
    if let (Some(mut p), Some(cp)) = (pac::Peripherals::take(), Peripherals::take()) {
        let rcc = p.RCC.configure().sysclk(8.MHz()).freeze(&mut p.FLASH);

        let gpioa = p.GPIOA.split();

        // (Re-)configure PA5 as output
        let mut led = gpioa.pa5.into_push_pull_output();
        // (Re-)configure PA0 as analog input
        let mut an_in = gpioa.pa0.into_analog();

        // Get delay provider
        let mut delay = cp.SYST.delay(&rcc.clocks);

        // Get access to the ADC
        let mut adc = Adc::new(p.ADC, hal::adc::AdcClockMode::Pclk);

        loop {
            led.toggle();

            let time: u16 = if let Ok(val) = adc.read(&mut an_in) as Result<u16, _> {
                /* shift the value right by 3, same as divide by 8, reduces
                the 0-4095 range into something approximating 1-512 */
                (val >> 3) + 1
            } else {
                1000
            };

            delay.delay_ms(time);
        }
    }

    loop {
        continue;
    }
}
