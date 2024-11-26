//! API for the integrated timers
//!
//! This only implements basic functions, a lot of things are missing
//!
//! # Example
//! Blink the led with 1Hz
//! ``` no_run
//! use py32f0xx_hal as hal;
//!
//! use crate::hal::pac;
//! use crate::hal::prelude::*;
//! use crate::hal::time::*;
//! use crate::hal::timers::*;
//! use nb::block;
//!
//! let mut p = pac::Peripherals::take().unwrap();
//! let rcc = p.RCC.configure().freeze(&mut p.FLASH);
//!
//! let gpioa = p.GPIOA.split();
//!
//! let mut led = gpioa.pa1.into_push_pull_pull_output();
//!
//! let mut timer = Timer::tim1(p.TIM1, Hertz(1), &rcc.clocks);
//! loop {
//!     led.toggle();
//!     block!(timer.wait()).ok();
//! }
//! ```
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;

use crate::pac::RCC;
use crate::rcc::{BusTimerClock, Clocks, Enable, Rcc, Reset};
use crate::time::Hertz;
use embedded_hal_02::timer::{CountDown, Periodic};
use void::Void;

/// Hardware timers
pub struct Timer<TIM> {
    clocks: Clocks,
    tim: TIM,
}

/// Interrupt events
pub enum Event {
    /// Timer timed out / count down ended
    TimeOut,
}

impl Timer<SYST> {
    /// Configures the SYST clock as a periodic count down timer
    pub fn syst<T>(mut syst: SYST, timeout: T, rcc: &Rcc) -> Self
    where
        T: Into<Hertz>,
    {
        syst.set_clock_source(SystClkSource::Core);
        let mut timer = Timer {
            tim: syst,
            clocks: rcc.clocks,
        };
        timer.start(timeout);
        timer
    }

    /// Starts listening for an `event`
    pub fn listen(&mut self, event: &Event) {
        match event {
            Event::TimeOut => self.tim.enable_interrupt(),
        }
    }

    /// Stops listening for an `event`
    pub fn unlisten(&mut self, event: &Event) {
        match event {
            Event::TimeOut => self.tim.disable_interrupt(),
        }
    }
}

/// Use the systick as a timer
///
/// Be aware that intervals less than 4 Hertz may not function properly
impl CountDown for Timer<SYST> {
    type Time = Hertz;

    /// Start the timer with a `timeout`
    fn start<T>(&mut self, timeout: T)
    where
        T: Into<Hertz>,
    {
        let rvr = self.clocks.sysclk().0 / timeout.into().0 - 1;

        assert!(rvr < (1 << 24));

        self.tim.set_reload(rvr);
        self.tim.clear_current();
        self.tim.enable_counter();
    }

    /// Return `Ok` if the timer has wrapped
    /// Automatically clears the flag and restarts the time
    fn wait(&mut self) -> nb::Result<(), Void> {
        if self.tim.has_wrapped() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }
}

impl Periodic for Timer<SYST> {}

pub trait Instance: crate::Sealed + Enable + Reset + BusTimerClock {}

macro_rules! timers {
    ($($TIM:ident: $tim:ident,)+) => {
        $(
            use crate::pac::$TIM;
            impl Timer<$TIM> {
                // XXX(why not name this `new`?) bummer: constructors need to have different names
                // even if the `$TIM` are non overlapping (compare to the `free` function below
                // which just works)
                /// Configures a TIM peripheral as a periodic count down timer
                pub fn $tim<T>(tim: $TIM, timeout: T, clocks: &Clocks) -> Self
                where
                    T: Into<Hertz>,
                {
                    // enable and reset peripheral to a clean slate state
                    let rcc = unsafe { &(*RCC::ptr()) };
                    $TIM::enable(rcc);
                    $TIM::reset(rcc);

                    let mut timer = Timer {
                        clocks: *clocks,
                        tim,
                    };
                    timer.start(timeout);

                    timer
                }

                /// Starts listening for an `event`
                pub fn listen(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().set_bit());
                        }
                    }
                }

                /// Stops listening for an `event`
                pub fn unlisten(&mut self, event: Event) {
                    match event {
                        Event::TimeOut => {
                            // Enable update event interrupt
                            self.tim.dier.write(|w| w.uie().clear_bit());
                        }
                    }
                }

                /// Releases the TIM peripheral
                pub fn release(self) -> $TIM {
                    let rcc = unsafe { &(*crate::pac::RCC::ptr()) };
                    // Pause counter
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                    // Disable timer
                    $TIM::disable(rcc);
                    self.tim
                }

                /// Clears interrupt flag
                pub fn clear_irq(&mut self) {
                    self.tim.sr.modify(|_, w| w.uif().clear_bit());
                }
            }

            impl CountDown for Timer<$TIM> {
                type Time = Hertz;

                /// Start the timer with a `timeout`
                fn start<T>(&mut self, timeout: T)
                where
                    T: Into<Hertz>,
                {
                    // pause
                    self.tim.cr1.modify(|_, w| w.cen().clear_bit());
                    // restart counter
                    self.tim.cnt.reset();

                    let frequency = timeout.into().0;
                    let ticks = $TIM::timer_clock(&self.clocks).0 / frequency;

                    let psc = cast::u16((ticks - 1) / (1 << 16)).unwrap();
                    self.tim.psc.write(|w| unsafe { w.psc().bits(psc) });

                    let arr = cast::u16(ticks / cast::u32(psc + 1)).unwrap();
                    self.tim.arr.write(|w| unsafe { w.bits(cast::u32(arr)) });

                    // start counter
                    self.tim.cr1.modify(|_, w| w.cen().set_bit());
                }

                /// Return `Ok` if the timer has wrapped
                /// Automatically clears the flag and restarts the time
                fn wait(&mut self) -> nb::Result<(), Void> {
                    if self.tim.sr.read().uif().bit_is_clear() {
                        Err(nb::Error::WouldBlock)
                    } else {
                        self.tim.sr.modify(|_, w| w.uif().clear_bit());
                        Ok(())
                    }
                }
            }

            impl Periodic for Timer<$TIM> {}
        )+
    }
}

timers! {
    TIM1: tim1,
}

#[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002a"))]
timers! {
    TIM16: tim16,
}

#[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002b"))]
timers! {
    TIM14: tim14,
}

#[cfg(any(feature = "py32f030", feature = "py32f003"))]
timers! {
    TIM3: tim3,
    TIM17: tim17,
}

use crate::gpio::AF2;
use crate::gpio::{gpioa::*, gpiob::*, Alternate};

#[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002a"))]
use crate::gpio::{AF1, AF13, AF14};

#[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002b"))]
use crate::gpio::AF5;

#[cfg(any(feature = "py32f030", feature = "py32f003"))]
use crate::gpio::{gpiof::*, AF0, AF4};

#[cfg(feature = "py32f002b")]
use crate::gpio::{gpioc::*, AF3};

// Output channels marker traits
pub trait PinC1<TIM> {}
pub trait PinC1N<TIM> {}
pub trait PinC2<TIM> {}
pub trait PinC2N<TIM> {}
pub trait PinC3<TIM> {}
pub trait PinC3N<TIM> {}
pub trait PinC4<TIM> {}

macro_rules! channel_impl {
    ( $( $TIM:ident, $PINC:ident, $PINX:ident, $MODE:ident<$AF:ident>; )+ ) => {
        $(
            impl $PINC<$TIM> for $PINX<$MODE<$AF>> {}
        )+
    };
}

#[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002a"))]
channel_impl!(
    TIM1, PinC3, PA0, Alternate<AF13>;
    TIM1, PinC1N, PA0, Alternate<AF14>;
    TIM1, PinC4, PA1, Alternate<AF13>;
    TIM1, PinC2N, PA1, Alternate<AF14>;
    TIM1, PinC1, PA3, Alternate<AF13>;
    TIM1, PinC1N, PA7, Alternate<AF2>;
    TIM1, PinC2, PA13, Alternate<AF13>;
    TIM1, PinC2N, PB0, Alternate<AF2>;
    TIM1, PinC3N, PB1, Alternate<AF2>;
    TIM1, PinC3, PB6, Alternate<AF1>;
);

#[cfg(any(feature = "py32f030", feature = "py32f002a"))]
channel_impl!(
    TIM1, PinC1, PA8, Alternate<AF2>;
    TIM1, PinC2, PA9, Alternate<AF2>;
    TIM1, PinC3, PA10, Alternate<AF2>;
    TIM1, PinC4, PA11, Alternate<AF2>;
    TIM1, PinC2, PB3, Alternate<AF1>;
);

#[cfg(any(feature = "py32f030", feature = "py32f003"))]
channel_impl!(
    TIM3, PinC1, PA2, Alternate<AF13>;
    TIM3, PinC3, PA4, Alternate<AF13>;
    TIM3, PinC2, PA5, Alternate<AF13>;
    TIM3, PinC1, PA6, Alternate<AF1>;
    TIM3, PinC2, PA7, Alternate<AF1>;
    TIM3, PinC3, PB0, Alternate<AF1>;
    TIM3, PinC4, PB1, Alternate<AF1>;
    TIM3, PinC2, PB5, Alternate<AF1>;

    TIM14, PinC1, PA4, Alternate<AF4>;
    TIM14, PinC1, PA7, Alternate<AF4>;
    TIM14, PinC1, PB1, Alternate<AF0>;
    TIM14, PinC1, PF0, Alternate<AF2>;
    TIM14, PinC1, PF1, Alternate<AF13>;

    TIM16, PinC1, PA6, Alternate<AF5>;
    TIM16, PinC1N, PB6, Alternate<AF2>;

    TIM17, PinC1, PA7, Alternate<AF5>;
    TIM17, PinC1N, PB7, Alternate<AF2>;
);

#[cfg(feature = "py32f030")]
channel_impl!(
    TIM3, PinC1, PB4, Alternate<AF1>;

    TIM16, PinC1, PB8, Alternate<AF2>;

    TIM17, PinC1, PB8, Alternate<AF13>;
);

#[cfg(feature = "py32f002b")]
channel_impl!(
    TIM1, PinC1, PA0, Alternate<AF2>;
    TIM1, PinC2, PA1, Alternate<AF2>;
    TIM1, PinC4, PA2, Alternate<AF2>;
    TIM1, PinC2, PA3, Alternate<AF2>;
    TIM1, PinC3, PA4, Alternate<AF2>;
    TIM1, PinC1, PA5, Alternate<AF2>;
    TIM1, PinC4, PA7, Alternate<AF2>;
    TIM1, PinC2, PB0, Alternate<AF2>;
    TIM1, PinC3N, PB0, Alternate<AF3>;
    TIM1, PinC2N, PB1, Alternate<AF2>;
    TIM1, PinC4, PB1, Alternate<AF3>;
    TIM1, PinC1N, PB2, Alternate<AF2>;
    TIM1, PinC3, PB2, Alternate<AF3>;
    TIM1, PinC3, PB5, Alternate<AF2>;
    TIM1, PinC1N, PC0, Alternate<AF2>;

    TIM14, PinC1, PA4, Alternate<AF5>;
    TIM14, PinC1, PA5, Alternate<AF5>;
    TIM14, PinC1, PB5, Alternate<AF5>;
    TIM14, PinC1, PB7, Alternate<AF5>;
);
