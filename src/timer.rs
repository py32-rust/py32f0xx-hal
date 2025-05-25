//! API for the integrated timers
//!
//! # Example
//! Blink the led with 1Hz
//! ``` no_run
//! use py32f0xx_hal as hal;
//!
//! use crate::hal::pac;
//! use crate::hal::prelude::*;
//! use crate::hal::timer::*;
//! use nb::block;
//!
//! let mut p = pac::Peripherals::take().unwrap();
//! let rcc = p.RCC.configure().freeze(&mut p.FLASH);
//!
//! let gpioa = p.GPIOA.split();
//!
//! let mut led = gpioa.pa1.into_push_pull_pull_output();
//!
//! let mut timer = p.TIM1.counter_hz(&rcc.clocks);
//! timer.start(1.Hz()).unwrap();
//! loop {
//!     led.toggle();
//!     block!(timer.wait()).ok();
//! }
//! ```

#![allow(non_upper_case_globals)]

use crate::pac::{self, DBG};
use crate::rcc::{BusTimerClock, Clocks, Enable, Reset};
use crate::time::Hertz;
use core::convert::TryFrom;
use cortex_m::peripheral::syst::SystClkSource;
use cortex_m::peripheral::SYST;

#[cfg(feature = "rtic")]
pub mod monotonic;
#[cfg(feature = "rtic")]
pub use monotonic::*;
pub(crate) mod pins;
//pub mod pwm_input;
pub use pins::*;
pub mod delay;
pub use delay::*;
pub mod counter;
pub use counter::*;
pub mod pwm;
pub use pwm::*;

mod hal_02;
mod hal_1;

/// Hardware timers
pub struct Timer<TIM> {
    clk: Hertz,
    tim: TIM,
}

/// Enum for Timer Channels
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Channel {
    /// Timer Channel 1
    C1 = 0,
    /// Timer Channel 2
    C2 = 1,
    /// Timer Channel 3
    C3 = 2,
    /// Timer Channel 4
    C4 = 3,
}

/// Interrupt events for SYST
#[derive(Clone)]
pub enum SysEvent {
    /// Timer timed out / count down ended
    Update,
}

/// Interrupt events for general timers
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Event {
    /// Timer Update event
    Update,
    /// Timer Capture/Compare 1
    C1,
    /// Timer Capture/Compare 2
    C2,
    /// Timer Capture/Compare 3
    C3,
    /// Timer Capture/Compare 4
    C4,
}

impl Event {
    /// is the Event set in a timer flags register value
    pub fn contains(&self, flags: u32) -> bool {
        let evt_val: u32 = (*self).into();
        evt_val & flags != 0
    }
}

impl From<Event> for u32 {
    fn from(evt: Event) -> u32 {
        match evt {
            Event::Update => 1 << 0,
            Event::C1 => 1 << 1,
            Event::C2 => 1 << 2,
            Event::C3 => 1 << 3,
            Event::C4 => 1 << 4,
        }
    }
}

/// Error for a Timer peripheral
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Timer is disabled
    Disabled,
    /// Attempting to set auto reload value to incorrect value
    WrongAutoReload,
}

/// Extension trait for Timer peripheral
pub trait TimerExt: Sized {
    /// Non-blocking [Counter] with custom fixed precision
    fn counter<const FREQ: u32>(self, clocks: &Clocks) -> Counter<Self, FREQ>;
    /// Non-blocking [Counter] with fixed precision of 1 ms (1 kHz sampling)
    ///
    /// Can wait from 2 ms to 65 sec for 16-bit timer and from 2 ms to 49 days for 32-bit timer.
    ///
    /// NOTE: don't use this if your system frequency more than 65 MHz
    fn counter_ms(self, clocks: &Clocks) -> CounterMs<Self> {
        self.counter::<1_000>(clocks)
    }
    /// Non-blocking [Counter] with fixed precision of 1 μs (1 MHz sampling)
    ///
    /// Can wait from 2 μs to 65 ms for 16-bit timer and from 2 μs to 71 min for 32-bit timer.
    fn counter_us(self, clocks: &Clocks) -> CounterUs<Self> {
        self.counter::<1_000_000>(clocks)
    }
    /// Non-blocking [Counter] with dynamic precision which uses `Hertz` as Duration units
    fn counter_hz(self, clocks: &Clocks) -> CounterHz<Self>;

    /// Blocking [Delay] with custom fixed precision
    fn delay<const FREQ: u32>(self, clocks: &Clocks) -> Delay<Self, FREQ>;
    /// Blocking [Delay] with fixed precision of 1 ms (1 kHz sampling)
    ///
    /// Can wait from 2 ms to 49 days.
    ///
    /// NOTE: don't use this if your system frequency more than 65 MHz
    fn delay_ms(self, clocks: &Clocks) -> DelayMs<Self> {
        self.delay::<1_000>(clocks)
    }
    /// Blocking [Delay] with fixed precision of 1 μs (1 MHz sampling)
    ///
    /// Can wait from 2 μs to 71 min.
    fn delay_us(self, clocks: &Clocks) -> DelayUs<Self> {
        self.delay::<1_000_000>(clocks)
    }
}

impl<TIM: Instance> TimerExt for TIM {
    fn counter<const FREQ: u32>(self, clocks: &Clocks) -> Counter<Self, FREQ> {
        FTimer::new(self, clocks).counter()
    }
    fn counter_hz(self, clocks: &Clocks) -> CounterHz<Self> {
        Timer::new(self, clocks).counter_hz()
    }
    fn delay<const FREQ: u32>(self, clocks: &Clocks) -> Delay<Self, FREQ> {
        FTimer::new(self, clocks).delay()
    }
}

/// Extension trait for SysTimer peripheral
pub trait SysTimerExt: Sized {
    /// Creates timer which takes [Hertz] as Duration
    fn counter_hz(self, clocks: &Clocks) -> SysCounterHz;

    /// Creates timer with custom precision (core frequency recommended is known)
    fn counter<const FREQ: u32>(self, clocks: &Clocks) -> SysCounter<FREQ>;
    /// Creates timer with precision of 1 μs (1 MHz sampling)
    fn counter_us(self, clocks: &Clocks) -> SysCounterUs {
        self.counter::<1_000_000>(clocks)
    }
    /// Blocking [Delay] with custom precision
    fn delay(self, clocks: &Clocks) -> SysDelay;
}

impl SysTimerExt for SYST {
    fn counter_hz(self, clocks: &Clocks) -> SysCounterHz {
        Timer::syst(self, clocks).counter_hz()
    }
    fn counter<const FREQ: u32>(self, clocks: &Clocks) -> SysCounter<FREQ> {
        Timer::syst(self, clocks).counter()
    }
    fn delay(self, clocks: &Clocks) -> SysDelay {
        Timer::syst_external(self, clocks).delay()
    }
}

impl Timer<SYST> {
    /// Initialize SysTick timer
    pub fn syst(mut tim: SYST, clocks: &Clocks) -> Self {
        tim.set_clock_source(SystClkSource::Core);
        Self {
            tim,
            clk: clocks.sysclk(),
        }
    }

    /// Initialize SysTick timer and set it frequency to `HCLK`
    pub fn syst_external(mut tim: SYST, clocks: &Clocks) -> Self {
        tim.set_clock_source(SystClkSource::External);
        Self {
            tim,
            clk: clocks.hclk(),
        }
    }

    /// Set `SYST` timer to [Clocks.sysclk]
    pub fn configure(&mut self, clocks: &Clocks) {
        self.tim.set_clock_source(SystClkSource::Core);
        self.clk = clocks.sysclk();
    }

    /// Set `SYST` timer to [Clocks.hclk]
    pub fn configure_external(&mut self, clocks: &Clocks) {
        self.tim.set_clock_source(SystClkSource::External);
        self.clk = clocks.hclk();
    }

    /// Release the timer resource
    pub fn release(self) -> SYST {
        self.tim
    }

    /// Starts listening for an `SysEvent`
    pub fn listen(&mut self, event: SysEvent) {
        match event {
            SysEvent::Update => self.tim.enable_interrupt(),
        }
    }

    /// Stops listening for an `event`
    pub fn unlisten(&mut self, event: SysEvent) {
        match event {
            SysEvent::Update => self.tim.disable_interrupt(),
        }
    }

    /// Resets the timer counter
    pub fn reset(&mut self) {
        // According to the Cortex-M3 Generic User Guide, the interrupt request is only generated
        // when the counter goes from 1 to 0, so writing zero should not trigger an interrupt
        self.tim.clear_current();
    }
}

/// Output compare mode for Timer
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum Ocm {
    /// No effect on outputs
    Frozen = 0,
    /// Set channel to active on match
    ActiveOnMatch = 1,
    /// set channel to inactive on match
    InactiveOnMatch = 2,
    /// Set channel to toggle active/inactive on match
    Toggle = 3,
    /// Force channel to inactive
    ForceInactive = 4,
    /// Force channel to active
    ForceActive = 5,
    /// PWM mode 1
    PwmMode1 = 6,
    /// PWM mode 2
    PwmMode2 = 7,
}

/// Output compare polarity
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum OcmPolarity {
    /// Output compare high level
    High = 0,
    /// Output compare low level
    Low = 1,
}

/// Output compare complementary polarity
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum OcmNPolarity {
    /// output compare high level
    High = 0,
    /// output compare low level
    Low = 1,
}

impl From<OcmPolarity> for bool {
    fn from(val: OcmPolarity) -> bool {
        match val {
            OcmPolarity::Low => true,
            OcmPolarity::High => false,
        }
    }
}

impl From<OcmNPolarity> for bool {
    fn from(val: OcmNPolarity) -> bool {
        match val {
            OcmNPolarity::Low => true,
            OcmNPolarity::High => false,
        }
    }
}

mod sealed {
    use super::{Channel, Event, Ocm, OcmNPolarity, OcmPolarity, DBG};
    pub trait General {
        type Width: Into<u32> + From<u16>;
        fn max_auto_reload() -> u32;
        unsafe fn set_auto_reload_unchecked(&mut self, arr: u32);
        fn set_auto_reload(&mut self, arr: u32) -> Result<(), super::Error>;
        fn read_auto_reload() -> u32;
        fn enable_preload(&mut self, b: bool);
        fn enable_counter(&mut self);
        fn disable_counter(&mut self);
        fn is_counter_enabled(&self) -> bool;
        fn reset_counter(&mut self);
        fn set_prescaler(&mut self, psc: u16);
        fn read_prescaler(&self) -> u16;
        fn trigger_update(&mut self);
        fn clear_interrupt_flag(&mut self, event: Event);
        fn listen_interrupt(&mut self, event: Event, b: bool);
        fn has_interrupt_flag(&self, event: Event) -> bool;
        fn read_count(&self) -> Self::Width;
        fn cr1_reset(&mut self);
        fn stop_in_debug(&mut self, dbg: &mut DBG, state: bool);
    }

    pub trait WithPwm: General {
        // Number of Channels
        const CH_NUM: u8;
        // Number of Complementary Channels
        const CHN_NUM: u8;
        fn read_cc_value(channel: u8) -> u32;
        fn set_cc_value(channel: u8, value: u32);
        // set the complementary output compare polarity
        fn set_output_compn_polarity(&mut self, channel: Channel, polarity: OcmNPolarity);
        fn set_comp_off_state_run_mode(&mut self, state: bool);
        fn preload_output_channel_in_mode(
            &mut self,
            channel: Channel,
            mode: Ocm,
            polarity: OcmPolarity,
        );
        fn start_pwm(&mut self);
        fn stop_pwm(&mut self);
        fn enable_channel(channel: u8, b: bool);
        fn enable_comp(channel: u8, b: bool);
    }

    pub trait MasterTimer: General {
        type Mms;
        fn master_mode(&mut self, mode: Self::Mms);
    }

    pub trait OnePulseMode: General {
        fn start_one_pulse(&mut self);
    }
}

pub(crate) use sealed::{General, MasterTimer, OnePulseMode, WithPwm};

/// Instance of Timer peripheral
pub trait Instance: crate::Sealed + Enable + Reset + BusTimerClock + General {}

macro_rules! hal {
    ($($TIM:ty: [
        $Timer:ident,
        $bits:ty,
        $dbg_timX_reg:ident,
        $dbg_timX_stop:ident,
        $(c: ($cnum:ident, $chnum:literal $(, $aoe:ident)?),)?
        $(m: $timbase:ident,)?
        $(opm: $opm:ident,)?
    ],)+) => {
        $(
            impl Instance for $TIM { }
            /// Alias for Timer peripheral
            pub type $Timer = Timer<$TIM>;

            impl General for $TIM {
                type Width = $bits;

                #[inline(always)]
                fn max_auto_reload() -> u32 {
                    <$bits>::MAX as u32
                }
                #[inline(always)]
                unsafe fn set_auto_reload_unchecked(&mut self, arr: u32) {
                    self.arr.write(|w| w.bits(arr))
                }
                #[inline(always)]
                fn set_auto_reload(&mut self, arr: u32) -> Result<(), Error> {
                    // Note: Make it impossible to set the ARR value to 0, since this
                    // would cause an infinite loop.
                    if arr > 0 && arr <= Self::max_auto_reload() {
                        Ok(unsafe { self.set_auto_reload_unchecked(arr) })
                    } else {
                        Err(Error::WrongAutoReload)
                    }
                }
                #[inline(always)]
                fn read_auto_reload() -> u32 {
                    let tim = unsafe { &*<$TIM>::ptr() };
                    tim.arr.read().bits()
                }
                #[inline(always)]
                fn enable_preload(&mut self, b: bool) {
                    self.cr1.modify(|_, w| w.arpe().bit(b));
                }
                #[inline(always)]
                fn enable_counter(&mut self) {
                    self.cr1.modify(|_, w| w.cen().set_bit());
                }
                #[inline(always)]
                fn disable_counter(&mut self) {
                    self.cr1.modify(|_, w| w.cen().clear_bit());
                }
                #[inline(always)]
                fn is_counter_enabled(&self) -> bool {
                    self.cr1.read().cen().is_enabled()
                }
                #[inline(always)]
                fn reset_counter(&mut self) {
                    self.cnt.reset();
                }
                #[inline(always)]
                fn set_prescaler(&mut self, psc: u16) {
                    self.psc.write(|w| unsafe { w.psc().bits(psc) } );
                }
                #[inline(always)]
                fn read_prescaler(&self) -> u16 {
                    self.psc.read().psc().bits()
                }
                #[inline(always)]
                fn trigger_update(&mut self) {
                    // Sets the URS bit to prevent an interrupt from being triggered by
                    // the UG bit
                    self.cr1.modify(|_, w| w.urs().set_bit());
                    self.egr.write(|w| w.ug().set_bit());
                    self.cr1.modify(|_, w| w.urs().clear_bit());
                }
                #[inline(always)]
                fn clear_interrupt_flag(&mut self, event: Event) {
                    let evt_bits: u32 = event.into();
                    self.sr.write(|w| unsafe { w.bits(!evt_bits) });
                }
                #[inline(always)]
                fn listen_interrupt(&mut self, event: Event, b: bool) {
                    let event_bits: u32 = event.into();
                    self.dier.modify(|r, w| unsafe { w.bits(
                        if b {
                            r.bits() | event_bits
                        } else {
                            r.bits() & !event_bits
                        }
                    ) });
                }
                #[inline(always)]
                fn has_interrupt_flag(&self, event: Event) -> bool {
                    event.contains(self.sr.read().bits())
                }
                #[inline(always)]
                fn read_count(&self) -> Self::Width {
                    self.cnt.read().bits() as Self::Width
                }
                #[inline(always)]
                fn cr1_reset(&mut self) {
                    self.cr1.reset();
                }
                #[inline(always)]
                fn stop_in_debug(&mut self, dbg: &mut DBG, state: bool) {
                    dbg.$dbg_timX_reg.modify(|_, w| w.$dbg_timX_stop().bit(state));
                }
            }
            $(with_pwm!($TIM: $cnum, $chnum $(, $aoe)?);)?

            $(impl MasterTimer for $TIM {
                type Mms = pac::$timbase::cr2::MMS_A;
                fn master_mode(&mut self, mode: Self::Mms) {
                    self.cr2.modify(|_,w| w.mms().variant(mode));
                }
            })?

            $(impl OnePulseMode for $TIM {
                fn start_one_pulse(&mut self) {
                    self.cr1.modify(|_, w| w.$opm().set_bit().cen().set_bit());
                }
            })?
        )+
    }
}

macro_rules! with_pwm {
    // General purpose timer with one output channel (TIM16/17), maximum of 1 complementary output
    ($TIM:ty: CH1, $chnum:literal $(, $aoe:ident)?) => {
        impl WithPwm for $TIM {
            const CH_NUM: u8 = 1;
            const CHN_NUM: u8 = $chnum;

            #[inline(always)]
            fn read_cc_value(channel: u8) -> u32 {
                let tim = unsafe { &*<$TIM>::ptr() };
                if channel < Self::CH_NUM {
                    tim.ccr[channel as usize].read().bits()
                } else {
                    0
                }
            }

            #[inline(always)]
            fn set_cc_value(channel: u8, value: u32) {
                let tim = unsafe { &*<$TIM>::ptr() };
                #[allow(unused_unsafe)]
                if channel < Self::CH_NUM {
                    tim.ccr[channel as usize].write(|w| unsafe { w.bits(value) });
                }
            }

            #[inline(always)]
            fn set_output_compn_polarity(&mut self, channel: Channel, polarity: OcmNPolarity) {
                match channel {
                    Channel::C1 => {
                        self.ccer
                            .modify(|_, w| w.cc1np().bit(polarity.into()) );
                    }
                    _ => {},
                }
            }

            #[inline(always)]
            #[allow(unused_variables)]
            fn set_comp_off_state_run_mode(&mut self, state: bool) {
                $(let $aoe = self.bdtr.modify(|_, w| w.ossr().bit(state));)?
            }

            // enable channel preload and channel mode
            #[inline(always)]
            fn preload_output_channel_in_mode(&mut self, channel: Channel, mode: Ocm, polarity: OcmPolarity) {
                match channel {
                    Channel::C1 => {
                        #[allow(unused_unsafe)]
                        self.ccmr1_output()
                            .modify(|_, w| unsafe { w.oc1pe().set_bit().oc1m().bits(mode as _) });
                        self.ccer.modify(|_,w| w.cc1p().bit(polarity.into()));
                        $(let $aoe = self.cr2.modify(|_,w| w.ois1().set_bit().ois1n().clear_bit() );)?
                    }
                    _ => {},
                }
            }

            #[inline(always)]
            fn start_pwm(&mut self) {
                $(let $aoe = self.bdtr.modify(|_, w| w.aoe().set_bit());)?
                self.cr1.modify(|_, w| w.cen().set_bit());
            }

            #[inline(always)]
            fn stop_pwm(&mut self) {
                self.cr1.modify(|_, w| w.cen().clear_bit());
                $(let $aoe = self.bdtr.modify(|_, w| w.aoe().clear_bit());)?
            }

            #[inline(always)]
            fn enable_channel(c: u8, b: bool) {
                let tim = unsafe { &*<$TIM>::ptr() };
                match c {
                    0 => tim.ccer.modify(|_,w| w.cc1e().bit(b)),
                    _ => {},
                }
            }

            #[inline(always)]
            fn enable_comp(c: u8, b: bool) {
                let tim = unsafe { &*<$TIM>::ptr() };
                if c >= Self::CHN_NUM {
                    panic!("Complementary channel not available");
                }
                // note(SAFETY) the channel number limit is set via
                // hal macros so we should never get an invalid bit
                // set state. If we were to use the named bit methods, then
                // some devices wouldn't compile without featuring
                // them out, so we use bit twiddling instead
                let shift = c * 4 + 2;
                let val = if b { 1 << shift } else { 0 };
                tim.ccer.modify(|r,w| unsafe { w.bits((r.bits() & !(1 << shift)) | val) });
            }
        }
    };
    // General purpose timer with 4 output channels (TIM1, TIM3), maximum of 3 complementary output
    ($TIM:ty: CH4, $chnum:literal $(, $aoe:ident)?) => {
        impl WithPwm for $TIM {
            const CH_NUM: u8 = 4;
            const CHN_NUM: u8 =  $chnum;

            #[inline(always)]
            fn read_cc_value(channel: u8) -> u32 {
                let tim = unsafe { &*<$TIM>::ptr() };
                tim.ccr[channel as usize].read().bits()
            }

            #[inline(always)]
            fn set_cc_value(channel: u8, value: u32) {
                let tim = unsafe { &*<$TIM>::ptr() };
                tim.ccr[channel as usize].write(|w| unsafe { w.bits(value) });
            }

            #[inline(always)]
            #[allow(unused_variables)]
            fn set_comp_off_state_run_mode(&mut self, state: bool) {
                $(let $aoe = self.bdtr.modify(|_, w| w.ossr().bit(state));)?
            }

            #[inline(always)]
            fn set_output_compn_polarity(&mut self, channel: Channel, polarity: OcmNPolarity) {
                match channel {
                    Channel::C1 => {
                        self.ccer
                            .modify(|_, w| w.cc1np().bit(polarity.into()) );
                    }
                    Channel::C2 => {
                        self.ccer
                            .modify(|_, w| w.cc2np().bit(polarity.into()) );
                    }
                    Channel::C3 => {
                        self.ccer
                            .modify(|_, w| w.cc3np().bit(polarity.into()) );
                    }
                    _ => {},
                }
            }

            #[inline(always)]
            fn preload_output_channel_in_mode(&mut self, channel: Channel, mode: Ocm, polarity: OcmPolarity) {
                match channel {
                    Channel::C1 => {
                        self.ccmr1_output()
                            .modify(|_, w| w.oc1pe().set_bit().oc1m().bits(mode as _) );
                        self.ccer.modify(|_,w| w.cc1p().bit(polarity.into()));
                        $(let $aoe = self.cr2.modify(|_,w| w.ois1().set_bit().ois1n().clear_bit() );)?
                    }
                    Channel::C2 => {
                        self.ccmr1_output()
                        .modify(|_, w| w.oc2pe().set_bit().oc2m().bits(mode as _) );
                        $(let $aoe = self.cr2.modify(|_,w| w.ois2().set_bit().ois2n().clear_bit() );)?
                    }
                    Channel::C3 => {
                        self.ccmr2_output()
                        .modify(|_, w| w.oc3pe().set_bit().oc3m().bits(mode as _) );
                        $(let $aoe = self.cr2.modify(|_,w| w.ois3().set_bit().ois3n().clear_bit() );)?
                    }
                    Channel::C4 => {
                        self.ccmr2_output()
                        .modify(|_, w| w.oc4pe().set_bit().oc4m().bits(mode as _) );
                        $(let $aoe = self.cr2.modify(|_,w| w.ois4().set_bit() );)?
                    }
                }
            }

            #[inline(always)]
            fn start_pwm(&mut self) {
                $(let $aoe = self.bdtr.modify(|_, w| w.aoe().set_bit());)?
                self.cr1.modify(|_, w| w.cen().set_bit());
            }

            #[inline(always)]
            fn stop_pwm(&mut self) {
                self.cr1.modify(|_, w| w.cen().clear_bit());
                $(let $aoe = self.bdtr.modify(|_, w| w.aoe().clear_bit());)?
            }

            #[inline(always)]
            fn enable_channel(c: u8, b: bool) {
                let tim = unsafe { &*<$TIM>::ptr() };
                match c {
                    0 => tim.ccer.modify(|_,w| w.cc1e().bit(b)),
                    1 => tim.ccer.modify(|_,w| w.cc2e().bit(b)),
                    2 => tim.ccer.modify(|_,w| w.cc3e().bit(b)),
                    3 => tim.ccer.modify(|_,w| w.cc4e().bit(b)),
                    _ => {},
                }
            }

            #[inline(always)]
            fn enable_comp(c: u8, b: bool) {
                let tim = unsafe { &*<$TIM>::ptr() };
                if c >= Self::CHN_NUM {
                    panic!("Complementary channel not available");
                }
                // note(SAFETY) the channel number limit is set via
                // hal macros so we should never get an invalid bit
                // set state. If we were to use the named bit methods, then
                // some devices wouldn't compile without featuring
                // them out, so we use bit twiddling instead
                let shift = c * 4 + 2;
                let val = if b { 1 << shift } else { 0 };
                tim.ccer.modify(|r,w| unsafe { w.bits((r.bits() & !(1 << shift)) | val) });
            }
        }
    }
}

impl<TIM: Instance> Timer<TIM> {
    /// Initialize timer
    pub fn new(tim: TIM, clocks: &Clocks) -> Self {
        unsafe {
            //NOTE(unsafe) this reference will only be used for atomic writes with no side effects
            let rcc = &(*pac::RCC::ptr());
            // Enable and reset the timer peripheral
            TIM::enable(rcc);
            TIM::reset(rcc);
        }

        Self {
            clk: TIM::timer_clock(clocks),
            tim,
        }
    }

    /// Set the [Timer] bus clock
    pub fn configure(&mut self, clocks: &Clocks) {
        self.clk = TIM::timer_clock(clocks);
    }

    /// Construct a [CounterHz] timer from instance
    pub fn counter_hz(self) -> CounterHz<TIM> {
        CounterHz(self)
    }

    /// Release the timer instance
    pub fn release(self) -> TIM {
        self.tim
    }

    /// Starts listening for an [Event]
    ///
    /// Note, you will also have to enable the TIM2 interrupt in the NVIC to start
    /// receiving events.
    pub fn listen(&mut self, event: Event) {
        self.tim.listen_interrupt(event, true);
    }

    /// Clears interrupt associated with `event`.
    ///
    /// If the interrupt is not cleared, it will immediately retrigger after
    /// the ISR has finished.
    pub fn clear_interrupt(&mut self, event: Event) {
        self.tim.clear_interrupt_flag(event);
    }

    /// [Event] has occurred
    pub fn has_interrupt(&mut self, event: Event) -> bool {
        self.tim.has_interrupt_flag(event)
    }

    /// Stops listening for an [Event]
    pub fn unlisten(&mut self, event: Event) {
        self.tim.listen_interrupt(event, false);
    }

    /// Stopping timer in debug mode can cause troubles when sampling the signal
    pub fn stop_in_debug(&mut self, dbg: &mut DBG, state: bool) {
        self.tim.stop_in_debug(dbg, state);
    }
}

impl<TIM: Instance + MasterTimer> Timer<TIM> {
    /// Set the Timer master mode
    pub fn set_master_mode(&mut self, mode: TIM::Mms) {
        self.tim.master_mode(mode)
    }
}

/// Timer wrapper for fixed precision timers.
///
/// Uses `fugit::TimerDurationU32` for most of operations
pub struct FTimer<TIM, const FREQ: u32> {
    tim: TIM,
}

/// `FTimer` with precision of 1 μs (1 MHz sampling)
pub type FTimerUs<TIM> = FTimer<TIM, 1_000_000>;

/// `FTimer` with precision of 1 ms (1 kHz sampling)
///
/// NOTE: don't use this if your system frequency more than 65 MHz
pub type FTimerMs<TIM> = FTimer<TIM, 1_000>;

impl<TIM: Instance, const FREQ: u32> FTimer<TIM, FREQ> {
    /// Initialize timer
    pub fn new(tim: TIM, clocks: &Clocks) -> Self {
        unsafe {
            //NOTE(unsafe) this reference will only be used for atomic writes with no side effects
            let rcc = &(*pac::RCC::ptr());
            // Enable and reset the timer peripheral
            TIM::enable(rcc);
            TIM::reset(rcc);
        }

        let mut t = Self { tim };
        t.configure(clocks);
        t
    }

    /// Calculate prescaler depending on `Clocks` state
    pub fn configure(&mut self, clocks: &Clocks) {
        let clk = TIM::timer_clock(clocks);
        assert!(clk.raw() % FREQ == 0);
        let psc = clk.raw() / FREQ;
        self.tim.set_prescaler(u16::try_from(psc - 1).unwrap());
    }

    /// Creates `Counter`
    pub fn counter(self) -> Counter<TIM, FREQ> {
        Counter(self)
    }

    /// Creates `Delay`
    pub fn delay(self) -> Delay<TIM, FREQ> {
        Delay(self)
    }

    /// Releases the TIM peripheral
    pub fn release(self) -> TIM {
        self.tim
    }

    /// Starts listening for an `event`
    ///
    /// Note, you will also have to enable the TIMX interrupt in the NVIC to start
    /// receiving events.
    pub fn listen(&mut self, event: Event) {
        self.tim.listen_interrupt(event, true);
    }

    /// Clears interrupt associated with `event`.
    ///
    /// If the interrupt is not cleared, it will immediately retrigger after
    /// the ISR has finished.
    pub fn clear_interrupt(&mut self, event: Event) {
        self.tim.clear_interrupt_flag(event);
    }

    /// Get interrupt [Event]
    pub fn has_interrupt(&self, event: Event) -> bool {
        self.tim.has_interrupt_flag(event)
    }

    /// Stops listening for an `event`
    pub fn unlisten(&mut self, event: Event) {
        self.tim.listen_interrupt(event, false);
    }

    /// Stopping timer in debug mode can cause troubles when sampling the signal
    pub fn stop_in_debug(&mut self, dbg: &mut DBG, state: bool) {
        self.tim.stop_in_debug(dbg, state);
    }
}

impl<TIM: Instance + MasterTimer, const FREQ: u32> FTimer<TIM, FREQ> {
    /// Set Master mode on [FTimer] instance
    pub fn set_master_mode(&mut self, mode: TIM::Mms) {
        self.tim.master_mode(mode)
    }
}

impl<TIM: Instance + OnePulseMode, const FREQ: u32> FTimer<TIM, FREQ> {
    /// Creates `OpmDelay`
    pub fn onepulsemode_delay(self) -> OpmDelay<TIM, FREQ> {
        OpmDelay(self)
    }
}

#[inline(always)]
const fn compute_arr_presc(freq: u32, clock: u32) -> (u16, u32) {
    let ticks = clock / freq;
    let psc = (ticks - 1) / (1 << 16);
    let arr = ticks / (psc + 1) - 1;
    (psc as u16, arr)
}

hal!(
    pac::TIM1: [Timer1, u16, apb_fz2, dbg_timer1_stop, c: (CH4, 3, _aoe), m: tim1, opm: opm,],
);

#[cfg(any(feature = "py32f030", feature = "py32f003"))]
hal!(
    pac::TIM3: [Timer3, u16, apb_fz1, dbg_timer3_stop, c: (CH4, 0), m: tim3, opm: opm,],
    pac::TIM17: [Timer17, u16, apb_fz2, dbg_timer17_stop, c: (CH1, 1, _aoe), opm: opm,],
);

#[cfg(any(feature = "py32f030", feature = "py32f003"))]
hal!(
    pac::TIM16: [Timer16, u16, apb_fz2, dbg_timer16_stop, c: (CH1, 1, _aoe), opm: opm,],
);

#[cfg(any(feature = "py32f002a"))]
hal!(
    pac::TIM16: [Timer16, u16, apb_fz2, dbg_timer16_stop, opm: opm,],
);

#[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002b"))]
hal!(
    pac::TIM14: [Timer14, u16, apb_fz2, dbg_timer14_stop, c: (CH1, 0),],
);
