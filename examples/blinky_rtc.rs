//! Blinks an LED
//!
//! This assumes that a LED is connected to pa12

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_halt as _;

use py32f0xx_hal::{pac, prelude::*, rtc::Rtc};

use cortex_m_rt::entry;
use nb::block;

#[entry]
fn main() -> ! {
    let mut dp = pac::Peripherals::take().unwrap();
    let mut rcc = dp.RCC.configure().freeze(&mut dp.FLASH);

    // Set up the GPIO pin
    let gpioa = dp.GPIOA.split(&mut rcc);
    let mut led = gpioa.pa12.into_push_pull_output();

    // Set up the RTC
    // Start the RTC
    let mut rtc = Rtc::new(dp.RTC, &mut rcc, &mut dp.PWR);

    let mut led_on = false;
    loop {
        // Set the current time to 0
        rtc.set_time(0);
        // Trigger the alarm in 5 seconds
        rtc.set_alarm(5);
        block!(rtc.wait_alarm()).unwrap();
        if led_on {
            led.set_low();
            led_on = false;
        } else {
            led.set_high();
            led_on = true;
        }
    }
}
