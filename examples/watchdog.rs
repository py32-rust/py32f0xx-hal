#![no_main]
#![no_std]

use panic_halt as _;

use py32f0xx_hal as hal;

use crate::hal::{delay::Delay, pac, prelude::*, serial::Serial, time::Hertz, watchdog};
use core::fmt::Write;
use cortex_m::peripheral::Peripherals;
use cortex_m_rt::entry;
use embedded_hal_02::blocking::delay::DelayMs;
use embedded_hal_02::watchdog::{Watchdog, WatchdogEnable};

#[entry]
fn main() -> ! {
    if let (Some(p), Some(cp)) = (pac::Peripherals::take(), Peripherals::take()) {
        let mut flash = p.FLASH;
        let mut rcc = p.RCC.configure().sysclk(8.mhz()).freeze(&mut flash);

        let gpioa = p.GPIOA.split();
        let dbg = p.DBG;

        // Disable the watchdog when the cpu is stopped under debug
        dbg.apb_fz1.modify(|_, w| w.dbg_iwdg_stop().set_bit());

        let mut watchdog = watchdog::Watchdog::new(&mut rcc, p.IWDG);

        // Get delay provider
        let mut delay = Delay::new(cp.SYST, &rcc);

        // Configure serial TX pin
        let tx = gpioa.pa2.into_alternate_af1();

        // Obtain a serial peripheral with for unidirectional communication
        let mut serial = Serial::new(p.USART1, (tx, ()), 115_200.bps(), &rcc.clocks);

        serial.write_str("RESET \r\n").ok();

        watchdog.start(Hertz(1));
        delay.delay_ms(500_u16);
        watchdog.feed();
        serial.write_str("Feed the dog 1st\r\n").ok();
        delay.delay_ms(500_u16);
        watchdog.feed();
        serial.write_str("Feed the dog 2nd\r\n").ok();
        delay.delay_ms(500_u16);
        watchdog.feed();
        serial.write_str("Feed the dog 3rd\r\n").ok();

        // Now a reset happens while delaying
        delay.delay_ms(1500_u16);
        serial.write_str("This won't\r\n").ok();
    }

    loop {
        continue;
    }
}
