#![deny(unsafe_code)]
#![no_main]
#![no_std]

use embedded_hal::delay::DelayNs;
// Halt on panic
use panic_halt as _;

use cortex_m::peripheral::Peripherals;
use cortex_m_rt::entry;

use py32f0xx_hal as hal;

use defmt::info;
use defmt_rtt as _;
use hal::{pac, prelude::*};

#[entry]
fn main() -> ! {
    info!("pwm mutable frequency example");
    let mut dp = pac::Peripherals::take().unwrap();

    let cp = Peripherals::take().unwrap();

    // Set up the system clock.
    let rcc = dp.RCC.configure().sysclk(24.MHz()).freeze(&mut dp.FLASH);

    let gpioa = dp.GPIOA.split();

    let channel = gpioa.pa0.into_alternate_af13();

    let mut pwm = dp.TIM1.pwm_hz_mutable_frequency(channel, &rcc.clocks);

    let mut ch = pwm.channels();

    // Get delay provider
    let mut delay = cp.SYST.delay(&rcc.clocks);

    loop {
        info!("setting frequency 660 Hz");
        pwm.set_frequency(660.Hz());
        let max_duty = pwm.get_max_duty();
        ch.set_duty(max_duty / 2);
        ch.enable();
        delay.delay_ms(1000);
        ch.disable();

        info!("setting frequency 990 Hz");
        pwm.set_frequency(990.Hz());
        let max_duty = pwm.get_max_duty();
        ch.set_duty(max_duty / 2);
        ch.enable();
        delay.delay_ms(1000);
        ch.disable();
    }
}
