#![deny(unsafe_code)]
#![no_main]
#![no_std]

// Halt on panic
use panic_halt as _;

use cortex_m;
use cortex_m_rt::entry;

use py32f0xx_hal as hal;

use embedded_hal_02::blocking::delay::DelayMs;
use hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    let mut dp = pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();
    // Set up the system clock.
    let rcc = dp.RCC.configure().sysclk(24.MHz()).freeze(&mut dp.FLASH);

    let gpioa = dp.GPIOA.split();
    let channels = (
        gpioa.pa8.into_alternate_af2(), // on TIM1_CH1
        gpioa.pa7.into_alternate_af2(), // on TIM1_CH1N
    );

    let pwm = dp.TIM1.pwm_hz(channels, 20.kHz(), &rcc.clocks);
    let (mut ch1, mut ch1n) = pwm.split();
    let max_duty = ch1.get_max_duty();
    ch1.set_duty(max_duty / 2);
    ch1.enable();
    ch1n.enable();

    // simple duty sweep
    let mut delay = cp.SYST.delay(&rcc.clocks);

    let steps = 100;

    loop {
        for i in 0..steps {
            ch1.set_duty(max_duty / steps * i);
            delay.delay_ms(30u16);
        }

        for i in (1..steps).rev() {
            ch1.set_duty(max_duty / steps * i);
            delay.delay_ms(30u16);
        }
    }
}
