//! Turns the user LED on
//!
//! Listens for interrupts on the pa11 pin. On any rising or falling edge, toggles
//! the pa12 pin (which is connected to the LED on the dev board, hence the `led` name).

#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use panic_halt as _;

use core::mem::MaybeUninit;
use cortex_m_rt::entry;
use pac::interrupt;
use py32f0xx_hal::gpio::*;
use py32f0xx_hal::{pac, prelude::*};

// These two are owned by the ISR. main() may only access them during the initialization phase,
// where the interrupt is not yet enabled (i.e. no concurrent accesses can occur).
// After enabling the interrupt, main() may not have any references to these objects any more.
// For the sake of minimalism, we do not use RTIC here, which would be the better way.
static mut LED: MaybeUninit<py32f0xx_hal::gpio::gpioa::PA12<Output<PushPull>>> =
    MaybeUninit::uninit();
static mut INT_PIN: MaybeUninit<py32f0xx_hal::gpio::gpioa::PA11<Input<Floating>>> =
    MaybeUninit::uninit();

#[interrupt]
fn EXTI4_15() {
    let led = unsafe { &mut *LED.as_mut_ptr() };
    let int_pin = unsafe { &mut *INT_PIN.as_mut_ptr() };

    if int_pin.check_interrupt() {
        led.toggle();

        // if we don't clear this bit, the ISR would trigger indefinitely
        int_pin.clear_interrupt_pending_bit();
    }
}

#[entry]
fn main() -> ! {
    // initialization phase
    let mut p = pac::Peripherals::take().unwrap();
    let _cp = cortex_m::peripheral::Peripherals::take().unwrap();
    let _rcc = p.RCC.configure().sysclk(8.MHz()).freeze(&mut p.FLASH);
    {
        // the scope ensures that the int_pin reference is dropped before the first ISR can be executed.

        let gpioa = p.GPIOA.split();

        let led = unsafe { &mut *LED.as_mut_ptr() };
        *led = gpioa.pa12.into_push_pull_output();

        let int_pin = unsafe { &mut *INT_PIN.as_mut_ptr() };
        *int_pin = gpioa.pa11.into_floating_input();
        int_pin.make_interrupt_source(&mut p.EXTI);
        int_pin.trigger_on_edge(&mut p.EXTI, Edge::RisingFalling);
        int_pin.enable_interrupt(&mut p.EXTI);
    } // initialization ends here

    unsafe {
        pac::NVIC::unmask(pac::Interrupt::EXTI4_15);
    }

    loop {}
}
