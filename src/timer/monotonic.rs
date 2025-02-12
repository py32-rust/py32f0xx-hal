//! RTIC Monotonic implementation
//!
//! To use this monotonic timer implementation, use the feature `rtic`

use super::{FTimer, Instance};
use crate::rcc::Clocks;
use core::ops::{Deref, DerefMut};
use rtic_monotonic::Monotonic;

/// Monotonic Timer for RTIC
pub struct MonoTimer<TIM, const FREQ: u32> {
    timer: FTimer<TIM, FREQ>,
    ovf: u32,
}

impl<TIM, const FREQ: u32> Deref for MonoTimer<TIM, FREQ> {
    type Target = FTimer<TIM, FREQ>;
    fn deref(&self) -> &Self::Target {
        &self.timer
    }
}

impl<TIM, const FREQ: u32> DerefMut for MonoTimer<TIM, FREQ> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.timer
    }
}

/// `MonoTimer` with precision of 1 μs (1 MHz sampling)
pub type MonoTimerUs<TIM> = MonoTimer<TIM, 1_000_000>;

impl<TIM: Instance, const FREQ: u32> MonoTimer<TIM, FREQ> {
    /// Releases the TIM peripheral
    pub fn release(mut self) -> FTimer<TIM, FREQ> {
        // stop counter
        self.tim.cr1_reset();
        self.timer
    }
}

/// Extension trait for Timer peripheral
pub trait MonoTimerExt: Sized {
    /// Make a [MonoTimer] instance
    fn monotonic<const FREQ: u32>(self, clocks: &Clocks) -> MonoTimer<Self, FREQ>;
    /// Make a [MonoTimer] instance running at 1 μs
    fn monotonic_us(self, clocks: &Clocks) -> MonoTimer<Self, 1_000_000> {
        self.monotonic::<1_000_000>(clocks)
    }
}

macro_rules! mono {
    ($TIM:ty) => {
        impl MonoTimerExt for $TIM {
            fn monotonic<const FREQ: u32>(self, clocks: &Clocks) -> MonoTimer<Self, FREQ> {
                FTimer::new(self, clocks).monotonic()
            }
        }

        impl<const FREQ: u32> FTimer<$TIM, FREQ> {
            /// Make a [MonoTimer] instance
            pub fn monotonic(self) -> MonoTimer<$TIM, FREQ> {
                MonoTimer::<$TIM, FREQ>::_new(self)
            }
        }

        impl<const FREQ: u32> MonoTimer<$TIM, FREQ> {
            fn _new(timer: FTimer<$TIM, FREQ>) -> Self {
                // Set auto-reload value.
                timer.tim.arr.write(|w| unsafe { w.arr().bits(u16::MAX) });
                // Generate interrupt on overflow.
                timer.tim.egr.write(|w| w.ug().set_bit());

                // Start timer.
                // Clear interrupt flag.
                timer.tim.sr.modify(|_, w| w.uif().clear_bit());
                timer.tim.cr1.modify(|_, w| {
                    // Enable counter.
                    w.cen().set_bit();
                    // Overflow should trigger update event.
                    w.udis().clear_bit();
                    // Only overflow triggers interrupt.
                    w.urs().set_bit()
                });

                Self { timer, ovf: 0 }
            }
        }

        impl<const FREQ: u32> Monotonic for MonoTimer<$TIM, FREQ> {
            type Instant = fugit::TimerInstantU32<FREQ>;
            type Duration = fugit::TimerDurationU32<FREQ>;

            unsafe fn reset(&mut self) {
                self.tim.dier.modify(|_, w| w.cc1ie().set_bit());
            }

            #[inline(always)]
            fn now(&mut self) -> Self::Instant {
                let cnt = self.tim.cnt.read().cnt().bits() as u32;

                // If the overflow bit is set, we add this to the timer value. It means the `on_interrupt`
                // has not yet happened, and we need to compensate here.
                let ovf = if self.tim.sr.read().uif().bit_is_set() {
                    0x10000
                } else {
                    0
                };

                Self::Instant::from_ticks(cnt.wrapping_add(ovf).wrapping_add(self.ovf))
            }

            fn set_compare(&mut self, instant: Self::Instant) {
                let now = self.now();
                let cnt = self.tim.cnt.read().cnt().bits();

                // Since the timer may or may not overflow based on the requested compare val, we check
                // how many ticks are left.
                let val = match instant.checked_duration_since(now) {
                    None => cnt.wrapping_add(0xffff), // In the past, RTIC will handle this
                    Some(x) if x.ticks() <= 0xffff => instant.duration_since_epoch().ticks() as u16, // Will not overflow
                    Some(_) => cnt.wrapping_add(0xffff), // Will overflow, run for as long as possible
                };

                self.tim.ccr[0].write(|w| unsafe { w.ccr1().bits(val) });
            }

            fn clear_compare_flag(&mut self) {
                self.tim.sr.modify(|_, w| w.cc1if().clear_bit());
            }

            fn on_interrupt(&mut self) {
                // If there was an overflow, increment the overflow counter.
                if self.tim.sr.read().uif().bit_is_set() {
                    self.tim.sr.modify(|_, w| w.uif().clear_bit());

                    self.ovf += 0x10000;
                }
            }

            #[inline(always)]
            fn zero() -> Self::Instant {
                Self::Instant::from_ticks(0)
            }
        }
    };
}

mono!(crate::pac::TIM1);

#[cfg(any(feature = "py32f030", feature = "py32f003"))]
mono!(crate::pac::TIM3);
#[cfg(any(feature = "py32f030", feature = "py32f003"))]
mono!(crate::pac::TIM17);

#[cfg(any(feature = "py32f030", feature = "py32f003"))]
mono!(crate::pac::TIM16);

#[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002b"))]
mono!(crate::pac::TIM14);
