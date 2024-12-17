//! # General Purpose I/Os
//!
//! The GPIO pins are organised into groups of 16 pins which can be accessed through the
//! `gpioa`, `gpiob`... modules. To get access to the pins, you first need to convert them into a
//! HAL designed struct from the `pac` struct using the [split](trait.GpioExt.html#tymethod.split) function.
//! ```rust
//! // Acquire the GPIOA peripheral
//! // NOTE: `dp` is the device peripherals from the `PAC` crate
//! let mut gpioa = dp.GPIOA.split();
//! ```
//!
//! This gives you a struct containing all the pins `px0..px15`. These
//! structs are what you use to interract with the pins to change
//! their modes, or their inputs or outputs. For example, to set `pa5`
//! high, you would call
//!
//! ```rust
//! let output = gpioa.pa5.into_push_pull_output();
//! output.set_high();
//! ```
//!
//! ## Modes
//!
//! Each GPIO pin can be set to various modes:
//!
//! - **Alternate**: Pin mode required when the pin is driven by other peripherals
//! - **Dynamic**: Pin mode is selected at runtime. See changing configurations for more details
//! - Input
//!     - **PullUp**: Input connected to high with a weak pull-up resistor. Will be high when nothing
//!       is connected
//!     - **PullDown**: Input connected to ground with a weak pull-down resistor. Will be low when nothing
//!       is connected
//!     - **Floating**: Input not pulled to high or low. Will be undefined when nothing is connected
//! - Output
//!     - **PushPull**: Output which either drives the pin high or low
//!     - **OpenDrain**: Output which leaves the gate floating, or pulls it to ground in drain
//!       mode. Can be used as an input in the `open` configuration
//! - **Debugger**: Some pins start out being used by the debugger. A pin in this mode can only be
//!   used if the [JTAG peripheral has been turned off](#accessing-pa13-and-pa14).
//!
//! ## Changing modes
//! The simplest way to change the pin mode is to use the `into_<mode>` functions. These return a
//! new struct with the correct mode that you can use the input or output functions on.
//!
//! If you need a more temporary mode change, and can not use the `into_<mode>` functions for
//! ownership reasons, you can use the `as_<mode>` functions to temporarily change the pin type, do
//! some output or input, and then have it change back once done.
//!
//! ### Dynamic Mode Change
//! The above mode change methods guarantee that you can only call input functions when the pin is
//! in input mode, and output when in output modes, but can lead to some issues. Therefore, there
//! is also a mode where the state is kept track of at runtime, allowing you to change the mode
//! often, and without problems with ownership, or references, at the cost of some performance and
//! the risk of runtime errors.
//!
//! To make a pin dynamic, use the `into_dynamic` function, and then use the `make_<mode>` functions to
//! change the mode
//!
//! ## Accessing PA13, and PA14
//!
//! These pins are used by the DBG peripheral by default. To use them in your program, you need to
//! disable that peripheral. This is done using the [rcc::APB::disable_dbg](../rcc/struct.APB.html#method.disable_dbg) function

use crate::pac;
use core::marker::PhantomData;

mod erased;
pub use erased::{AnyPin, ErasedPin};
mod partially_erased;
pub use partially_erased::{PEPin, PartiallyErasedPin};

mod hal_02;
mod hal_1;

pub trait PinExt {
    type Mode;

    /// Return pin number
    fn pin_id(&self) -> u8;

    /// Return port id
    ///
    /// id is ascii byte, 'A' for porta, 'B' for portb
    fn port_id(&self) -> u8;
}

/// Extension trait to split a GPIO peripheral in independent pins and registers
pub trait GpioExt {
    /// The parts to split the GPIO into
    type Parts;

    /// Splits the GPIO block into independent pins and registers
    ///
    /// This resets the state of the GPIO block
    fn split(self) -> Self::Parts;

    /// Splits the GPIO block into independent pins and registers without resetting its state
    ///
    /// # Safety
    /// Make sure that all pins modes are set in reset state.
    unsafe fn split_without_reset(self) -> Self::Parts;
}

/// Marker trait for active states.
pub trait Active {}

/// Input mode (type state)
#[derive(Default)]
pub struct Input<PULL = Floating>(PhantomData<PULL>);

impl<PULL> Active for Input<PULL> {}

/// Used by the debugger (type state)
#[derive(Default)]
pub struct Debugger;

/// Floating input (type state)
#[derive(Default)]
pub struct Floating;

/// Pulled down input (type state)
#[derive(Default)]
pub struct PullDown;

/// Pulled up input (type state)
#[derive(Default)]
pub struct PullUp;

/// Output mode (type state)
#[derive(Default)]
pub struct Output<Otype = PushPull>(PhantomData<Otype>);

impl<MODE> Active for Output<MODE> {}

/// Open drain input or output (type state)
#[derive(Default)]
pub struct OpenDrain;

/// Push pull output (type state)
#[derive(Default)]
pub struct PushPull;

/// Analog mode (type state)
#[derive(Default)]
pub struct Analog;

impl Active for Analog {}

/// Alternate function mode (type state)
#[derive(Default)]
pub struct Alternate<AF>(PhantomData<AF>);

impl<AF> Active for Alternate<AF> {}

/// Alternate function 0
#[derive(Default)]
pub struct AF0;
/// Alternate function 1
#[derive(Default)]
pub struct AF1;
/// Alternate function 2
#[derive(Default)]
pub struct AF2;
/// Alternate function 3
#[derive(Default)]
pub struct AF3;
/// Alternate function 4
#[derive(Default)]
pub struct AF4;
/// Alternate function 5
#[derive(Default)]
pub struct AF5;
/// Alternate function 6
#[derive(Default)]
pub struct AF6;
/// Alternate function 7
#[derive(Default)]
pub struct AF7;
/// Alternate function 8
#[derive(Default)]
pub struct AF8;
/// Alternate function 9
#[derive(Default)]
pub struct AF9;
/// Alternate function 10
#[derive(Default)]
pub struct AF10;
/// Alternate function 11
#[derive(Default)]
pub struct AF11;
/// Alternate function 12
#[derive(Default)]
pub struct AF12;
/// Alternate function 13
#[derive(Default)]
pub struct AF13;
/// Alternate function 14
#[derive(Default)]
pub struct AF14;
/// Alternate function 15
#[derive(Default)]
pub struct AF15;

/// Digital output pin state
pub use embedded_hal_02::digital::v2::PinState;

// Using SCREAMING_SNAKE_CASE to be consistent with other HALs
// see 59b2740 and #125 for motivation
#[allow(non_camel_case_types)]
#[derive(Debug, PartialEq, Eq, Clone, Copy)]
pub enum Edge {
    Rising,
    Falling,
    RisingFalling,
}

mod sealed {
    /// Marker trait that show if `ExtiPin` can be implemented
    pub trait Interruptable {}

    pub trait PinMode: Default {
        const CNF: super::Cnf;
        const MODE: super::Mode;
        const PULL: Option<bool> = None;
        const AF: Option<super::Af> = None;
    }
}

use crate::pac::gpioa::{afrl::AFSEL0_A as Af, moder::MODE0_A as Mode, otyper::OT0_A as Cnf};

use sealed::Interruptable;
pub(crate) use sealed::PinMode;

impl<MODE> Interruptable for Input<MODE> {}
impl Interruptable for Dynamic {}

/// External Interrupt Pin
pub trait ExtiPin {
    fn make_interrupt_source(&mut self, exti: &mut pac::EXTI);
    fn trigger_on_edge(&mut self, exti: &mut pac::EXTI, level: Edge);
    fn enable_interrupt(&mut self, exti: &mut pac::EXTI);
    fn disable_interrupt(&mut self, exti: &mut pac::EXTI);
    fn clear_interrupt_pending_bit(&mut self);
    fn check_interrupt(&self) -> bool;
}

impl<PIN> ExtiPin for PIN
where
    PIN: PinExt,
    PIN::Mode: Interruptable,
{
    /// Make corresponding EXTI line sensitive to this pin
    fn make_interrupt_source(&mut self, exti: &mut pac::EXTI) {
        let pin_number = self.pin_id();
        let offset = 8 * (pin_number % 4);
        // for pins 0-3, mux selects ports a=0, b=1, or f=2
        // for py32f002b, a=0, b=1, c=2
        let port = match self.port_id() {
            b'A' => 0,
            b'B' => 1,
            b'C' | b'F' => 2,
            _ => unreachable!(),
        };
        // note(Safety) While the match below has invalid combinations for some parts, it cannot
        // actually hit those, because the match works off the pin and port number which is embedded
        // in the pin as states, so they can't have an invalid combination
        // For example there only 2 pins on port C of the py32f002b
        match pin_number {
            // pins 0-3, mux selects port as above
            0..=3 => {
                exti.exticr1.modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            // pin 4 mux selects port
            4 => {
                exti.exticr2.modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            // pins 5-7, mux selects ports a=0, b=1
            5..=7 => {
                exti.exticr2
                    .modify(|r, w| unsafe { w.bits(r.bits() | (port << offset)) });
            }
            // BUGBUG: py32f002a only has 15 port A pins? pin 8, mux selects ports a=0, b=1
            #[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002a"))]
            8 => {
                exti.exticr3
                    .modify(|r, w| unsafe { w.bits(r.bits() | (port << offset)) });
            }
            // BUGBUG: py32f002a only has 15 port A pins? pin 9-15, no mux
            #[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002a"))]
            9..=15 => {}
            _ => unreachable!(),
        }
    }

    /// Generate interrupt on rising edge, falling edge or both
    fn trigger_on_edge(&mut self, exti: &mut pac::EXTI, edge: Edge) {
        let pin_number = self.pin_id();
        match edge {
            Edge::Rising => {
                exti.rtsr
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << pin_number)) });
                exti.ftsr
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << pin_number)) });
            }
            Edge::Falling => {
                exti.rtsr
                    .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << pin_number)) });
                exti.ftsr
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << pin_number)) });
            }
            Edge::RisingFalling => {
                exti.rtsr
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << pin_number)) });
                exti.ftsr
                    .modify(|r, w| unsafe { w.bits(r.bits() | (1 << pin_number)) });
            }
        }
    }

    /// Enable external interrupts from this pin.
    fn enable_interrupt(&mut self, exti: &mut pac::EXTI) {
        exti.imr
            .modify(|r, w| unsafe { w.bits(r.bits() | (1 << self.pin_id())) });
    }

    /// Disable external interrupts from this pin
    fn disable_interrupt(&mut self, exti: &mut pac::EXTI) {
        exti.imr
            .modify(|r, w| unsafe { w.bits(r.bits() & !(1 << self.pin_id())) });
    }

    /// Clear the interrupt pending bit for this pin
    fn clear_interrupt_pending_bit(&mut self) {
        unsafe { (*pac::EXTI::ptr()).pr.write(|w| w.bits(1 << self.pin_id())) };
    }

    /// Reads the interrupt pending bit for this pin
    fn check_interrupt(&self) -> bool {
        unsafe { ((*pac::EXTI::ptr()).pr.read().bits() & (1 << self.pin_id())) != 0 }
    }
}

/// Tracks the current pin state for dynamic pins
pub enum Dynamic {
    InputFloating,
    InputPullUp,
    InputPullDown,
    OutputPushPull,
    OutputOpenDrain,
}

impl Default for Dynamic {
    fn default() -> Self {
        Dynamic::InputFloating
    }
}

impl Active for Dynamic {}

/// `Dynamic` pin error
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PinModeError {
    IncorrectMode,
}

impl Dynamic {
    fn is_input(&self) -> bool {
        use Dynamic::*;
        match self {
            InputFloating | InputPullUp | InputPullDown | OutputOpenDrain => true,
            OutputPushPull => false,
        }
    }

    fn is_output(&self) -> bool {
        use Dynamic::*;
        match self {
            InputFloating | InputPullUp | InputPullDown => false,
            OutputPushPull | OutputOpenDrain => true,
        }
    }
}

macro_rules! gpio {
    ($GPIOX:ident, $gpiox:ident, $PXx:ident, $port_id:expr, [
        $($PXi:ident: ($pxi:ident, $pin_number:expr $(, $MODE:ty)?),)+
    ]) => {
        /// GPIO
        pub mod $gpiox {

            use crate::{
                rcc::{Enable, Reset},
                pac::RCC,
                pac::$GPIOX
            };

            use super::{
                Active, PartiallyErasedPin, ErasedPin, Floating, GpioExt, Input,
                Pin,
            };
            #[allow(unused)]
            use super::Debugger;

            /// GPIO parts
            pub struct Parts {
                $(
                    /// Pin
                    pub $pxi: $PXi $(<$MODE>)?,
                )+
            }

            $(
                pub type $PXi<MODE = Input<Floating>> = Pin<$port_id, $pin_number, MODE>;
            )+

            impl GpioExt for $GPIOX {
                type Parts = Parts;

                fn split(self) -> Parts {
                    let rcc = unsafe { &(*RCC::ptr()) };
                    $GPIOX::enable(rcc);
                    $GPIOX::reset(rcc);

                    Parts {
                        $(
                            $pxi: $PXi::new(),
                        )+
                    }
                }

                unsafe fn split_without_reset(self) -> Parts {
                    let rcc = unsafe { &(*RCC::ptr()) };
                    $GPIOX::enable(rcc);

                    Parts {
                        $(
                            $pxi: $PXi::new(),
                        )+
                    }
                }
            }

            impl<MODE> PartiallyErasedPin<$port_id, MODE> {
                pub fn erase(self) -> ErasedPin<MODE> {
                    ErasedPin::$PXx(self)
                }
            }

            impl<const N: u8, MODE> Pin<$port_id, N, MODE>
            where
                MODE: Active,
            {
                /// Erases the pin number and port from the type
                ///
                /// This is useful when you want to collect the pins into an array where you
                /// need all the elements to have the same type
                pub fn erase(self) -> ErasedPin<MODE> {
                    self.erase_number().erase()
                }
            }
        }
        pub use $gpiox::{ $($PXi,)+ };
    }
}

/// Generic pin type
///
/// - `P` is port name: `A` for GPIOA, `B` for GPIOB, etc.
/// - `N` is pin number: from `0` to `15`.
/// - `MODE` is one of the pin modes (see [Modes](crate::gpio#modes) section).
pub struct Pin<const P: char, const N: u8, MODE = Input<Floating>> {
    mode: MODE,
}

impl<const P: char, const N: u8, MODE: Default> Pin<P, N, MODE> {
    fn new() -> Self {
        Self {
            mode: Default::default(),
        }
    }
}

impl<const P: char, const N: u8, MODE: Default> PinExt for Pin<P, N, MODE> {
    type Mode = MODE;

    fn pin_id(&self) -> u8 {
        N
    }

    fn port_id(&self) -> u8 {
        P as u8
    }
}

impl<const P: char, const N: u8> Pin<P, N, Debugger> {
    /// Put the pin in an active state. The caller
    /// must enforce that the pin is really in this
    /// state in the hardware.
    #[allow(dead_code)]
    pub(crate) unsafe fn activate(self) -> Pin<P, N, Input<Floating>> {
        Pin::new()
    }
}

// Internal helper functions

// NOTE: The functions in this impl block are "safe", but they
// are callable when the pin is in modes where they don't make
// sense.
impl<const P: char, const N: u8, MODE> Pin<P, N, MODE> {
    /**
      Set the output of the pin regardless of its mode.
      Primarily used to set the output value of the pin
      before changing its mode to an output to avoid
      a short spike of an incorrect value
    */

    #[inline(always)]
    fn _set_state(&mut self, state: PinState) {
        match state {
            PinState::High => self._set_high(),
            PinState::Low => self._set_low(),
        }
    }

    #[inline(always)]
    fn _set_high(&mut self) {
        // NOTE(unsafe) atomic write to a stateless register
        let gpio = unsafe { &(*gpiox::<P>()) };
        unsafe { gpio.bsrr.write(|w| w.bits(1 << N)) }
    }

    #[inline(always)]
    fn _set_low(&mut self) {
        // NOTE(unsafe) atomic write to a stateless register
        let gpio = unsafe { &(*gpiox::<P>()) };
        unsafe { gpio.bsrr.write(|w| w.bits(1 << (N + 16))) }
    }

    #[inline(always)]
    fn _is_set_low(&self) -> bool {
        // NOTE(unsafe) atomic read with no side effects
        let gpio = unsafe { &(*gpiox::<P>()) };
        gpio.odr.read().bits() & (1 << N) == 0
    }

    #[inline(always)]
    fn _is_low(&self) -> bool {
        // NOTE(unsafe) atomic read with no side effects
        let gpio = unsafe { &(*gpiox::<P>()) };
        gpio.idr.read().bits() & (1 << N) != 0
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE>
where
    MODE: Active,
{
    /// Erases the pin number from the type
    #[inline]
    pub fn erase_number(self) -> PartiallyErasedPin<P, MODE> {
        PartiallyErasedPin::new(N)
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, Output<MODE>> {
    #[inline]
    pub fn set_high(&mut self) {
        self._set_high()
    }

    #[inline]
    pub fn set_low(&mut self) {
        self._set_low()
    }

    #[inline(always)]
    pub fn get_state(&self) -> PinState {
        if self._is_set_low() {
            PinState::Low
        } else {
            PinState::High
        }
    }

    #[inline(always)]
    pub fn set_state(&mut self, state: PinState) {
        self._set_state(state)
    }

    #[inline]
    pub fn is_set_high(&self) -> bool {
        !self._is_set_low()
    }

    #[inline]
    pub fn is_set_low(&self) -> bool {
        self._is_set_low()
    }

    #[inline]
    pub fn toggle(&mut self) {
        if self._is_set_low() {
            self._set_high()
        } else {
            self._set_low()
        }
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, Input<MODE>> {
    #[inline]
    pub fn is_high(&self) -> bool {
        !self._is_low()
    }

    #[inline]
    pub fn is_low(&self) -> bool {
        self._is_low()
    }
}

impl<const P: char, const N: u8> Pin<P, N, Output<OpenDrain>> {
    #[inline]
    pub fn is_high(&self) -> bool {
        !self._is_low()
    }

    #[inline]
    pub fn is_low(&self) -> bool {
        self._is_low()
    }
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE>
where
    MODE: Active,
{
    /// Configures the pin to operate as a floating input pin
    #[inline]
    pub fn into_floating_input(self) -> Pin<P, N, Input<Floating>> {
        self.into_mode()
    }

    /// Configures the pin to operate as a pulled down input pin
    #[inline]
    pub fn into_pull_down_input(self) -> Pin<P, N, Input<PullDown>> {
        self.into_mode()
    }

    /// Configures the pin to operate as a pulled up input pin
    #[inline]
    pub fn into_pull_up_input(self) -> Pin<P, N, Input<PullUp>> {
        self.into_mode()
    }

    /// Configures the pin to operate as an open-drain output pin.
    /// Initial state will be low.
    #[inline]
    pub fn into_open_drain_output(self) -> Pin<P, N, Output<OpenDrain>> {
        self.into_open_drain_output_with_state(PinState::Low)
    }

    /// Configures the pin to operate as an open-drain output pin.
    /// `initial_state` specifies whether the pin should be initially high or low.
    #[inline]
    pub fn into_open_drain_output_with_state(
        mut self,
        initial_state: PinState,
    ) -> Pin<P, N, Output<OpenDrain>> {
        self._set_state(initial_state);
        self.into_mode()
    }

    /// Configures the pin to operate as an push-pull output pin.
    /// Initial state will be low.
    #[inline]
    pub fn into_push_pull_output(self) -> Pin<P, N, Output<PushPull>> {
        self.into_push_pull_output_with_state(PinState::Low)
    }

    /// Configures the pin to operate as an push-pull output pin.
    /// `initial_state` specifies whether the pin should be initially high or low.
    #[inline]
    pub fn into_push_pull_output_with_state(
        mut self,
        initial_state: PinState,
    ) -> Pin<P, N, Output<PushPull>> {
        self._set_state(initial_state);
        self.into_mode()
    }

    /// Configures the pin to operate as an push-pull output pin.
    /// The state will not be changed.
    #[inline]
    pub fn into_push_pull_output_with_current_state(self) -> Pin<P, N, Output<PushPull>> {
        self.into_mode()
    }

    /// Configures the pin to operate as an analog input pin
    #[inline]
    pub fn into_analog(self) -> Pin<P, N, Analog> {
        self.into_mode()
    }

    /// Configures the pin to operate in AF0 mode
    #[inline]
    pub fn into_alternate_af0(self) -> Pin<P, N, Alternate<AF0>> {
        self.into_mode()
    }

    /// Configures the pin to operate in AF1 mode
    #[inline]
    pub fn into_alternate_af1(self) -> Pin<P, N, Alternate<AF1>> {
        self.into_mode()
    }

    /// Configures the pin to operate in AF2 mode
    #[inline]
    pub fn into_alternate_af2(self) -> Pin<P, N, Alternate<AF2>> {
        self.into_mode()
    }

    /// Configures the pin to operate in AF3 mode
    #[inline]
    pub fn into_alternate_af3(self) -> Pin<P, N, Alternate<AF3>> {
        self.into_mode()
    }

    /// Configures the pin to operate in AF4 mode
    #[inline]
    pub fn into_alternate_af4(self) -> Pin<P, N, Alternate<AF4>> {
        self.into_mode()
    }

    /// Configures the pin to operate in AF5 mode
    #[inline]
    pub fn into_alternate_af5(self) -> Pin<P, N, Alternate<AF5>> {
        self.into_mode()
    }

    /// Configures the pin to operate in AF6 mode
    #[inline]
    pub fn into_alternate_af6(self) -> Pin<P, N, Alternate<AF6>> {
        self.into_mode()
    }

    /// Configures the pin to operate in AF7 mode
    #[inline]
    pub fn into_alternate_af7(self) -> Pin<P, N, Alternate<AF7>> {
        self.into_mode()
    }

    /// Configures the pin to operate in AF8 mode
    #[inline]
    pub fn into_alternate_af8(self) -> Pin<P, N, Alternate<AF8>> {
        self.into_mode()
    }

    /// Configures the pin to operate in AF9 mode
    #[inline]
    pub fn into_alternate_af9(self) -> Pin<P, N, Alternate<AF9>> {
        self.into_mode()
    }

    /// Configures the pin to operate in AF10 mode
    #[inline]
    pub fn into_alternate_af10(self) -> Pin<P, N, Alternate<AF10>> {
        self.into_mode()
    }

    /// Configures the pin to operate in AF11 mode
    #[inline]
    pub fn into_alternate_af11(self) -> Pin<P, N, Alternate<AF11>> {
        self.into_mode()
    }

    /// Configures the pin to operate in AF12 mode
    #[inline]
    pub fn into_alternate_af12(self) -> Pin<P, N, Alternate<AF12>> {
        self.into_mode()
    }

    /// Configures the pin to operate in AF13 mode
    #[inline]
    pub fn into_alternate_af13(self) -> Pin<P, N, Alternate<AF13>> {
        self.into_mode()
    }

    /// Configures the pin to operate in AF14 mode
    #[inline]
    pub fn into_alternate_af14(self) -> Pin<P, N, Alternate<AF14>> {
        self.into_mode()
    }

    /// Configures the pin to operate in AF15 mode
    #[inline]
    pub fn into_alternate_af15(self) -> Pin<P, N, Alternate<AF15>> {
        self.into_mode()
    }

    /// Configures the pin as a pin that can change between input
    /// and output without changing the type. It starts out
    /// as a floating input
    #[inline]
    pub fn into_dynamic(mut self) -> Pin<P, N, Dynamic> {
        self.mode::<Input<Floating>>();
        Pin::new()
    }
}

// These macros are defined here instead of at the top level in order
// to be able to refer to macro variables from the outer layers.
macro_rules! impl_temp_output {
    ($fn_name:ident, $stateful_fn_name:ident, $mode:ty) => {
        /// Temporarily change the mode of the pin.
        ///
        /// The value of the pin after conversion is undefined. If you
        /// want to control it, use `$stateful_fn_name`
        #[inline]
        pub fn $fn_name(&mut self, mut f: impl FnMut(&mut Pin<P, N, $mode>)) {
            self.mode::<$mode>();
            let mut temp = Pin::<P, N, $mode>::new();
            f(&mut temp);
            self.mode::<$mode>();
            Self::new();
        }

        /// Temporarily change the mode of the pin.
        ///
        /// Note that the new state is set slightly before conversion
        /// happens. This can cause a short output glitch if switching
        /// between output modes
        #[inline]
        pub fn $stateful_fn_name(
            &mut self,
            state: PinState,
            mut f: impl FnMut(&mut Pin<P, N, $mode>),
        ) {
            self._set_state(state);
            self.mode::<$mode>();
            let mut temp = Pin::<P, N, $mode>::new();
            f(&mut temp);
            self.mode::<$mode>();
            Self::new();
        }
    };
}
macro_rules! impl_temp_input {
    ($fn_name:ident, $mode:ty) => {
        /// Temporarily change the mode of the pin.
        #[inline]
        pub fn $fn_name(&mut self, mut f: impl FnMut(&mut Pin<P, N, $mode>)) {
            self.mode::<$mode>();
            let mut temp = Pin::<P, N, $mode>::new();
            f(&mut temp);
            self.mode::<$mode>();
            Self::new();
        }
    };
}

impl<const P: char, const N: u8, MODE> Pin<P, N, MODE>
where
    MODE: Active + PinMode,
{
    impl_temp_output!(
        as_push_pull_output,
        as_push_pull_output_with_state,
        Output<PushPull>
    );
    impl_temp_output!(
        as_open_drain_output,
        as_open_drain_output_with_state,
        Output<OpenDrain>
    );
    impl_temp_input!(as_floating_input, Input<Floating>);
    impl_temp_input!(as_pull_up_input, Input<PullUp>);
    impl_temp_input!(as_pull_down_input, Input<PullDown>);
}

// Dynamic pin

impl<const P: char, const N: u8> Pin<P, N, Dynamic> {
    #[inline]
    pub fn make_pull_up_input(&mut self) {
        // NOTE(unsafe), we have a mutable reference to the current pin
        self.mode::<Input<PullUp>>();
        self.mode = Dynamic::InputPullUp;
    }

    #[inline]
    pub fn make_pull_down_input(&mut self) {
        // NOTE(unsafe), we have a mutable reference to the current pin
        self.mode::<Input<PullDown>>();
        self.mode = Dynamic::InputPullDown;
    }

    #[inline]
    pub fn make_floating_input(&mut self) {
        // NOTE(unsafe), we have a mutable reference to the current pin
        self.mode::<Input<Floating>>();
        self.mode = Dynamic::InputFloating;
    }

    #[inline]
    pub fn make_push_pull_output(&mut self) {
        // NOTE(unsafe), we have a mutable reference to the current pin
        self.mode::<Output<PushPull>>();
        self.mode = Dynamic::OutputPushPull;
    }

    #[inline]
    pub fn make_open_drain_output(&mut self) {
        // NOTE(unsafe), we have a mutable reference to the current pin
        self.mode::<Output<OpenDrain>>();
        self.mode = Dynamic::OutputOpenDrain;
    }
}

impl PinMode for Analog {
    const MODE: Mode = Mode::Analog;
    const CNF: Cnf = Cnf::PushPull;
}

impl PinMode for Input<Floating> {
    const MODE: Mode = Mode::Input;
    const CNF: Cnf = Cnf::PushPull;
}

impl PinMode for Input<PullDown> {
    const MODE: Mode = Mode::Input;
    const CNF: Cnf = Cnf::PushPull;
    const PULL: Option<bool> = Some(false);
}

impl PinMode for Input<PullUp> {
    const MODE: Mode = Mode::Input;
    const CNF: Cnf = Cnf::PushPull;
    const PULL: Option<bool> = Some(true);
}

impl PinMode for Output<PushPull> {
    const MODE: Mode = Mode::Output;
    const CNF: Cnf = Cnf::PushPull;
}

impl PinMode for Output<OpenDrain> {
    const MODE: Mode = Mode::Output;
    const CNF: Cnf = Cnf::OpenDrain;
}

impl PinMode for Alternate<AF0> {
    const MODE: Mode = Mode::Alternate;
    const CNF: Cnf = Cnf::PushPull;
    const AF: Option<Af> = Some(Af::Af0);
}

impl PinMode for Alternate<AF1> {
    const MODE: Mode = Mode::Alternate;
    const CNF: Cnf = Cnf::PushPull;
    const AF: Option<Af> = Some(Af::Af1);
}

impl PinMode for Alternate<AF2> {
    const MODE: Mode = Mode::Alternate;
    const CNF: Cnf = Cnf::PushPull;
    const AF: Option<Af> = Some(Af::Af2);
}

impl PinMode for Alternate<AF3> {
    const MODE: Mode = Mode::Alternate;
    const CNF: Cnf = Cnf::PushPull;
    const AF: Option<Af> = Some(Af::Af3);
}

impl PinMode for Alternate<AF4> {
    const MODE: Mode = Mode::Alternate;
    const CNF: Cnf = Cnf::PushPull;
    const AF: Option<Af> = Some(Af::Af4);
}

impl PinMode for Alternate<AF5> {
    const MODE: Mode = Mode::Alternate;
    const CNF: Cnf = Cnf::PushPull;
    const AF: Option<Af> = Some(Af::Af5);
}

impl PinMode for Alternate<AF6> {
    const MODE: Mode = Mode::Alternate;
    const CNF: Cnf = Cnf::PushPull;
    const AF: Option<Af> = Some(Af::Af6);
}

impl PinMode for Alternate<AF7> {
    const MODE: Mode = Mode::Alternate;
    const CNF: Cnf = Cnf::PushPull;
    const AF: Option<Af> = Some(Af::Af7);
}

impl PinMode for Alternate<AF8> {
    const MODE: Mode = Mode::Alternate;
    const CNF: Cnf = Cnf::PushPull;
    const AF: Option<Af> = Some(Af::Af8);
}

impl PinMode for Alternate<AF9> {
    const MODE: Mode = Mode::Alternate;
    const CNF: Cnf = Cnf::PushPull;
    const AF: Option<Af> = Some(Af::Af9);
}

impl PinMode for Alternate<AF10> {
    const MODE: Mode = Mode::Alternate;
    const CNF: Cnf = Cnf::PushPull;
    const AF: Option<Af> = Some(Af::Af10);
}

impl PinMode for Alternate<AF11> {
    const MODE: Mode = Mode::Alternate;
    const CNF: Cnf = Cnf::PushPull;
    const AF: Option<Af> = Some(Af::Af11);
}

impl PinMode for Alternate<AF12> {
    const MODE: Mode = Mode::Alternate;
    const CNF: Cnf = Cnf::PushPull;
    const AF: Option<Af> = Some(Af::Af12);
}

impl PinMode for Alternate<AF13> {
    const MODE: Mode = Mode::Alternate;
    const CNF: Cnf = Cnf::PushPull;
    const AF: Option<Af> = Some(Af::Af13);
}

impl PinMode for Alternate<AF14> {
    const MODE: Mode = Mode::Alternate;
    const CNF: Cnf = Cnf::PushPull;
    const AF: Option<Af> = Some(Af::Af14);
}

impl PinMode for Alternate<AF15> {
    const MODE: Mode = Mode::Alternate;
    const CNF: Cnf = Cnf::PushPull;
    const AF: Option<Af> = Some(Af::Af15);
}

impl<const P: char, const N: u8, M> Pin<P, N, M> {
    fn mode<MODE: PinMode>(&mut self) {
        let gpio = unsafe { &(*gpiox::<P>()) };

        let offset2 = 2 * N;
        // set pull up/down if necessary
        if let Some(pull) = MODE::PULL {
            let pupdv = if pull { 0b01 } else { 0b11 };
            unsafe {
                gpio.pupdr
                    .modify(|r, w| w.bits((r.bits() & !(0b11 << offset2)) | (pupdv << offset2)))
            };
        }

        // set the mode
        let mv: u8 = MODE::MODE.into();
        unsafe {
            gpio.moder
                .modify(|r, w| w.bits((r.bits() & !(0b11 << offset2)) | ((mv as u32) << offset2)))
        };
        // if an output, set output type
        if MODE::MODE == Mode::Output {
            let otyperv = if MODE::CNF == Cnf::OpenDrain {
                0b1 << N
            } else {
                !(0b1 << N)
            };
            unsafe { gpio.otyper.modify(|r, w| w.bits(r.bits() | otyperv)) };
        }
        // if an alternate function pin, set that
        if let Some(af) = MODE::AF {
            let afv: u8 = af.into();
            unsafe {
                if N < 8 {
                    let offset4 = N * 4;
                    gpio.afrl.modify(|r, w| {
                        w.bits((r.bits() & !(0b1111 << offset4)) | ((afv as u32) << offset4))
                    });
                } else {
                    #[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002a"))]
                    {
                        let offset4 = (N - 8) * 4;
                        gpio.afrh.modify(|r, w| {
                            w.bits((r.bits() & !(0b1111 << offset4)) | ((afv as u32) << offset4))
                        });
                    }
                    #[cfg(feature = "py32f002b")]
                    {
                        // py32f002b does only have a maximum of 8 pins per port so it does not have AFRH
                        unreachable!();
                    }
                }
            }
        }
    }

    #[inline]
    pub(crate) fn into_mode<MODE: PinMode>(mut self) -> Pin<P, N, MODE> {
        self.mode::<MODE>();
        Pin::new()
    }
}

impl Analog {
    pub fn new<const P: char, const N: u8, MODE>(pin: Pin<P, N, MODE>) -> Pin<P, N, Self>
    where
        Self: PinMode,
    {
        pin.into_mode()
    }
}

impl<PULL> Input<PULL> {
    pub fn new<const P: char, const N: u8, MODE>(
        pin: Pin<P, N, MODE>,
        _pull: PULL,
    ) -> Pin<P, N, Self>
    where
        Self: PinMode,
    {
        pin.into_mode()
    }
}

impl<Otype> Output<Otype> {
    pub fn new<const P: char, const N: u8, MODE>(
        mut pin: Pin<P, N, MODE>,
        state: PinState,
    ) -> Pin<P, N, Self>
    where
        Self: PinMode,
    {
        pin._set_state(state);
        pin.into_mode()
    }
}

impl<AF> Alternate<AF> {
    pub fn new<const P: char, const N: u8, MODE>(pin: Pin<P, N, MODE>) -> Pin<P, N, Self>
    where
        Self: PinMode,
    {
        pin.into_mode()
    }
}

#[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002a"))]
gpio!(GPIOA, gpioa, PAx, 'A', [
    PA0: (pa0, 0),
    PA1: (pa1, 1),
    PA2: (pa2, 2),
    PA3: (pa3, 3),
    PA4: (pa4, 4),
    PA5: (pa5, 5),
    PA6: (pa6, 6),
    PA7: (pa7, 7),
    PA8: (pa8, 8),
    PA9: (pa9, 9),
    PA10: (pa10, 10),
    PA11: (pa11, 11),
    PA12: (pa12, 12),
    PA13: (pa13, 13, Debugger),
    PA14: (pa14, 14, Debugger),
    PA15: (pa15, 15),
]);

#[cfg(feature = "py32f002b")]
gpio!(GPIOA, gpioa, PAx, 'A', [
    PA0: (pa0, 0),
    PA1: (pa1, 1),
    PA2: (pa2, 2, Debugger),
    PA3: (pa3, 3),
    PA4: (pa4, 4),
    PA5: (pa5, 5),
    PA6: (pa6, 6),
    PA7: (pa7, 7),
]);

#[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002a"))]
gpio!(GPIOB, gpiob, PBx, 'B', [
    PB0: (pb0, 0),
    PB1: (pb1, 1),
    PB2: (pb2, 2),
    PB3: (pb3, 3),
    PB4: (pb4, 4),
    PB5: (pb5, 5),
    PB6: (pb6, 6),
    PB7: (pb7, 7),
    PB8: (pb8, 8),
]);

#[cfg(feature = "py32f002b")]
gpio!(GPIOB, gpiob, PBx, 'B', [
    PB0: (pb0, 0),
    PB1: (pb1, 1),
    PB2: (pb2, 2),
    PB3: (pb3, 3),
    PB4: (pb4, 4),
    PB5: (pb5, 5),
    PB6: (pb6, 6, Debugger),
    PB7: (pb7, 7),
    PB8: (pb8, 8),
]);

#[cfg(feature = "py32f002b")]
gpio!(GPIOC, gpioc, PCx, 'C', [
    PC0: (pc0, 0),
    PC1: (pc1, 1),
]);

#[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002a"))]
gpio!(GPIOF, gpiof, PFx, 'F', [
    PF0: (pf0, 0),
    PF1: (pf1, 1),
    PF2: (pf2, 2),
    PF3: (pf3, 3),
    PF4: (pf4, 4),
]);

const fn gpiox<const P: char>() -> *const crate::pac::gpioa::RegisterBlock {
    match P {
        'A' => crate::pac::GPIOA::ptr(),
        'B' => crate::pac::GPIOB::ptr() as _,
        #[cfg(feature = "py32f002b")]
        'C' => crate::pac::GPIOC::ptr() as _,
        #[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002a"))]
        'F' => crate::pac::GPIOF::ptr() as _,
        _ => unreachable!(),
    }
}
