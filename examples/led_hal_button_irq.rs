#![no_main]
#![no_std]

use panic_halt as _;

use py32f0xx_hal as hal;

use crate::hal::{
    gpio::*,
    pac::{interrupt, Interrupt, Peripherals},
    prelude::*,
    timer::SysDelay,
};

use cortex_m::{interrupt::Mutex, peripheral::Peripherals as c_m_Peripherals};
use cortex_m_rt::entry;

use core::{cell::RefCell, ops::DerefMut};
use embedded_hal_02::blocking::delay::DelayMs;

// Make our LED globally available
static LED: Mutex<RefCell<Option<gpioa::PA5<Output<PushPull>>>>> = Mutex::new(RefCell::new(None));

// Make our delay provider globally available
static DELAY: Mutex<RefCell<Option<SysDelay>>> = Mutex::new(RefCell::new(None));

// Make button pin globally available
static INT: Mutex<RefCell<Option<gpiob::PB2<Input<PullDown>>>>> = Mutex::new(RefCell::new(None));

#[entry]
fn main() -> ! {
    let mut p = Peripherals::take().unwrap();
    let cp = c_m_Peripherals::take().unwrap();
    // Enable clock for SYSCFG
    let rcc = p.RCC;
    rcc.apbenr2.modify(|_, w| w.syscfgen().set_bit());

    let mut flash = p.FLASH;
    let rcc = rcc.configure().sysclk(24.MHz()).freeze(&mut flash);

    let gpioa = p.GPIOA.split();
    let gpiob = p.GPIOB.split();

    // Configure PB2 as input (button)
    let mut int_pin = gpiob.pb2.into_pull_down_input();
    int_pin.make_interrupt_source(&mut p.EXTI);
    int_pin.trigger_on_edge(&mut p.EXTI, Edge::Rising);
    int_pin.enable_interrupt(&mut p.EXTI);

    // Configure PA5 as output (LED)
    let mut led = gpioa.pa5.into_push_pull_output();

    // Turn off LED
    led.set_low();

    // Initialise delay provider
    let delay = cp.SYST.delay(&rcc.clocks);

    // Move control over LED and DELAY and EXTI into global mutexes
    cortex_m::interrupt::free(move |cs| {
        *LED.borrow(cs).borrow_mut() = Some(led);
        *DELAY.borrow(cs).borrow_mut() = Some(delay);
        *INT.borrow(cs).borrow_mut() = Some(int_pin);
    });

    // Enable EXTI IRQ, set prio 1 and clear any pending IRQs
    let mut nvic = cp.NVIC;
    unsafe {
        nvic.set_priority(Interrupt::EXTI2_3, 1);
        cortex_m::peripheral::NVIC::unmask(Interrupt::EXTI2_3);
    }
    cortex_m::peripheral::NVIC::unpend(Interrupt::EXTI2_3);

    loop {
        continue;
    }
}

// Define an interrupt handler, i.e. function to call when interrupt occurs. Here if our external
// interrupt trips when the button is pressed and will light the LED for a second
#[interrupt]
fn EXTI2_3() {
    // Enter critical section
    cortex_m::interrupt::free(|cs| {
        // Obtain all Mutex protected resources
        if let (&mut Some(ref mut led), &mut Some(ref mut delay), &mut Some(ref mut int_pin)) = (
            LED.borrow(cs).borrow_mut().deref_mut(),
            DELAY.borrow(cs).borrow_mut().deref_mut(),
            INT.borrow(cs).borrow_mut().deref_mut(),
        ) {
            if int_pin.check_interrupt() {
                // Turn on LED
                led.set_high();

                // Wait a second
                delay.delay_ms(1_000_u16);

                // Turn off LED
                led.set_low();

                // Clear event triggering the interrupt
                int_pin.clear_interrupt_pending_bit();
            }
        }
    });
}
