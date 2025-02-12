//! API for Timer Delays
//! # Delay
//!
//! The general purpose timers and the SysTick timer can be used to
//! implement delay timers
//!
//! ```
//! let mut delay = p.TIM16.delay_us(&rcc.clocks);
//! loop {
//!     // ... do something here
//!     delay.delay(10.micros());
//! }
//! ```
//!
//! Using the SysTick timer
//! ```
//! let cp = cortex_m::peripheral::Peripherals::take().unwrap();
//! let mut delay = cp.SYST.delay(&rcc.clocks);
//! loop {
//!     // ... do something here
//!     delay.delay(10.millis());
//! }
//! ```
//!

use super::{Event, FTimer, Instance, OnePulseMode, Timer};
use core::ops::{Deref, DerefMut};
use cortex_m::peripheral::SYST;
use fugit::{MicrosDurationU32, TimerDurationU32};

/// Timer as a delay provider (SysTick by default)
pub struct SysDelay(Timer<SYST>);

impl Deref for SysDelay {
    type Target = Timer<SYST>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for SysDelay {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl SysDelay {
    /// Releases the timer resource
    pub fn release(self) -> Timer<SYST> {
        self.0
    }
}

impl Timer<SYST> {
    /// Create a [SysDelay] instance
    pub fn delay(self) -> SysDelay {
        SysDelay(self)
    }
}

impl SysDelay {
    /// Delay in μs, blocking
    pub fn delay(&mut self, us: MicrosDurationU32) {
        // The SysTick Reload Value register supports values between 1 and 0x00FFFFFF.
        const MAX_RVR: u32 = 0x00FF_FFFF;

        let mut total_rvr = us.ticks() * (self.clk.raw() / 1_000_000);

        while total_rvr != 0 {
            let current_rvr = total_rvr.min(MAX_RVR);

            self.tim.set_reload(current_rvr);
            self.tim.clear_current();
            self.tim.enable_counter();

            // Update the tracking variable while we are waiting...
            total_rvr -= current_rvr;

            while !self.tim.has_wrapped() {}

            self.tim.disable_counter();
        }
    }
}

/// Periodic non-blocking timer that implements [embedded_hal_02::blocking::delay] traits
pub struct Delay<TIM, const FREQ: u32>(pub(super) FTimer<TIM, FREQ>);

impl<T, const FREQ: u32> Deref for Delay<T, FREQ> {
    type Target = FTimer<T, FREQ>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T, const FREQ: u32> DerefMut for Delay<T, FREQ> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

/// `Delay` with precision of 1 μs (1 MHz sampling)
pub type DelayUs<TIM> = Delay<TIM, 1_000_000>;

/// `Delay` with precision of 1 ms (1 kHz sampling)
///
/// NOTE: don't use this if your system frequency more than 65 MHz
pub type DelayMs<TIM> = Delay<TIM, 1_000>;

impl<TIM: Instance, const FREQ: u32> Delay<TIM, FREQ> {
    /// Sleep for given time
    pub fn delay(&mut self, time: TimerDurationU32<FREQ>) {
        let mut ticks = time.ticks().max(1) - 1;
        while ticks != 0 {
            let reload = ticks.min(TIM::max_auto_reload());

            // Write Auto-Reload Register (ARR)
            unsafe {
                self.tim.set_auto_reload_unchecked(reload);
            }

            // Trigger update event (UEV) in the event generation register (EGR)
            // in order to immediately apply the config
            self.tim.trigger_update();

            // enable the counter.
            self.tim.enable_counter();

            // Update the tracking variable while we are waiting...
            ticks -= reload;
            // Wait for update to happen
            while !self.tim.has_interrupt_flag(Event::Update) { /* wait */ }
            // disable the counter
            self.tim.disable_counter();
        }
    }

    /// Get the Maximum delay possible, for use in `delay`
    pub fn max_delay(&self) -> TimerDurationU32<FREQ> {
        TimerDurationU32::from_ticks(TIM::max_auto_reload())
    }

    /// Releases the TIM peripheral
    pub fn release(mut self) -> FTimer<TIM, FREQ> {
        // stop counter
        self.tim.cr1_reset();
        self.0
    }
}

/// Periodic non-blocking timer that implements [embedded_hal_02::blocking::delay] traits
pub struct OpmDelay<TIM, const FREQ: u32>(pub(super) FTimer<TIM, FREQ>);

impl<T, const FREQ: u32> Deref for OpmDelay<T, FREQ> {
    type Target = FTimer<T, FREQ>;
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<T, const FREQ: u32> DerefMut for OpmDelay<T, FREQ> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

/// `Delay` with precision of 1 μs (1 MHz sampling)
pub type OpmDelayUs<TIM> = OpmDelay<TIM, 1_000_000>;

/// `Delay` with precision of 1 ms (1 kHz sampling)
///
/// NOTE: don't use this if your system frequency more than 65 MHz
pub type OpmDelayMs<TIM> = OpmDelay<TIM, 1_000>;

impl<TIM: Instance + OnePulseMode, const FREQ: u32> OpmDelay<TIM, FREQ> {
    fn delay(&mut self, time: TimerDurationU32<FREQ>) {
        let mut ticks = time.ticks().max(1) - 1;
        while ticks != 0 {
            let reload = ticks.min(TIM::max_auto_reload());

            // Write Auto-Reload Register (ARR)
            unsafe {
                self.tim.set_auto_reload_unchecked(reload);
            }

            // Trigger update event (UEV) in the event generation register (EGR)
            // in order to immediately apply the config
            self.tim.trigger_update();

            // Configure the counter in one-pulse mode (counter stops counting at
            // the next updateevent, clearing the CEN bit) and enable the counter.
            self.tim.start_one_pulse();

            // Update the tracking variable while we are waiting...
            ticks -= reload;
            // Wait for CEN bit to clear
            while self.tim.is_counter_enabled() { /* wait */ }
        }
    }

    /// Get the maximum delay possible, for use in `delay`
    pub fn max_delay(&self) -> TimerDurationU32<FREQ> {
        TimerDurationU32::from_ticks(TIM::max_auto_reload())
    }

    /// Releases the TIM peripheral
    pub fn release(mut self) -> FTimer<TIM, FREQ> {
        // stop counter
        self.tim.cr1_reset();
        self.0
    }
}

impl<TIM: Instance, const FREQ: u32> fugit_timer::Delay<FREQ> for Delay<TIM, FREQ> {
    type Error = core::convert::Infallible;

    fn delay(&mut self, duration: TimerDurationU32<FREQ>) -> Result<(), Self::Error> {
        self.delay(duration);
        Ok(())
    }
}

impl<TIM: Instance + OnePulseMode, const FREQ: u32> fugit_timer::Delay<FREQ>
    for OpmDelay<TIM, FREQ>
{
    type Error = core::convert::Infallible;

    fn delay(&mut self, duration: TimerDurationU32<FREQ>) -> Result<(), Self::Error> {
        self.delay(duration);
        Ok(())
    }
}

impl fugit_timer::Delay<1_000_000> for SysDelay {
    type Error = core::convert::Infallible;

    fn delay(&mut self, duration: MicrosDurationU32) -> Result<(), Self::Error> {
        self.delay(duration);
        Ok(())
    }
}
