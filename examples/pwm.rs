#![deny(unsafe_code)]
#![no_main]
#![no_std]

// Halt on panic
use panic_halt as _;

use cortex_m_rt::entry;

use py32f0xx_hal as hal;

use hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    let mut dp = pac::Peripherals::take().unwrap();
    // Set up the system clock.
    let rcc = dp.RCC.configure().sysclk(24.MHz()).freeze(&mut dp.FLASH);

    let gpioa = dp.GPIOA.split();
    let channels = (
        gpioa.pa8.into_alternate_af2(), // on TIM1_CH1
        gpioa.pa9.into_alternate_af2(), // on TIM1_CH2
    );

    let mut pwm = dp.TIM1.pwm_hz(channels, 20.kHz(), &rcc.clocks);
    let (ref mut ch1, ref mut ch2) = pwm.channels();
    let max_duty = ch1.get_max_duty();
    ch1.set_duty(max_duty / 4);
    ch1.enable();
    ch2.set_duty(max_duty * 9 / 10);
    ch2.enable();

    loop {
        cortex_m::asm::nop();
    }
}
