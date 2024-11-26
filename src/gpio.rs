//! General Purpose Input / Output

use core::marker::PhantomData;

use crate::pac;

mod hal_02;
mod hal_1;

pub trait PinExt {
    type Mode;

    /// Return pin number
    fn pin_id(&self) -> u8;

    /// Return port
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

trait GpioRegExt {
    fn is_low(&self, pos: u8) -> bool;
    fn is_set_low(&self, pos: u8) -> bool;
    fn set_high(&self, pos: u8);
    fn set_low(&self, pos: u8);
}

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
}

use sealed::Interruptable;

impl<MODE> Interruptable for Input<MODE> {}

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
        let port = match self.port_id() {
            b'A' => 0,
            b'B' => 1,
            b'C' | b'F' => 2,
            _ => unreachable!(),
        };
        match pin_number {
            // for pins 0-3, mux selects ports a=0, b=1, or f=2
            // for py32f002b, a=0, b=1, c=2
            0..=3 => {
                exti.exticr1.modify(|r, w| unsafe {
                    w.bits((r.bits() & !(0xf << offset)) | (port << offset))
                });
            }
            // pin 4 is same as pins 0-3
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
            // BUGBUG: py32f002a only has 8 pins? pin 8, mux selects ports a=0, b=1
            #[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002a"))]
            8 => {
                exti.exticr3
                    .modify(|r, w| unsafe { w.bits(r.bits() | (port << offset)) });
            }
            // BUGBUG: py32f002a only has 8 pins? pin 9-15, no mux
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

/// Alternate function 0
pub struct AF0;
/// Alternate function 1
pub struct AF1;
/// Alternate function 2
pub struct AF2;
/// Alternate function 3
pub struct AF3;
/// Alternate function 4
pub struct AF4;
/// Alternate function 5
pub struct AF5;
/// Alternate function 6
pub struct AF6;
/// Alternate function 7
pub struct AF7;
/// Alternate function 8
pub struct AF8;
/// Alternate function 9
pub struct AF9;
/// Alternate function 10
pub struct AF10;
/// Alternate function 11
pub struct AF11;
/// Alternate function 12
pub struct AF12;
/// Alternate function 13
pub struct AF13;
/// Alternate function 14
pub struct AF14;
/// Alternate function 15
pub struct AF15;

/// Alternate function mode (type state)
#[derive(Default)]
pub struct Alternate<AF> {
    _mode: PhantomData<AF>,
}

/// Input mode (type state)
#[derive(Default)]
pub struct Input<MODE> {
    _mode: PhantomData<MODE>,
}

/// Floating input (type state)
#[derive(Default)]
pub struct Floating;

/// Pulled down input (type state)
#[derive(Default)]
pub struct PullDown;

/// Pulled up input (type state)
#[derive(Default)]
pub struct PullUp;

/// Open drain input or output (type state)
#[derive(Default)]
pub struct OpenDrain;

/// Analog mode (type state)
#[derive(Default)]
pub struct Analog;

/// Output mode (type state)
#[derive(Default)]
pub struct Output<MODE> {
    _mode: PhantomData<MODE>,
}

/// Used by the debugger (type state)
#[derive(Default)]
pub struct Debugger;

/// Push pull output (type state)
#[derive(Default)]
pub struct PushPull;

/// Fully erased pin
pub struct Pin<MODE> {
    i: u8,
    p: u8,
    port: *const dyn GpioRegExt,
    _mode: PhantomData<MODE>,
}

// NOTE(unsafe) The only write access is to BSRR, which is thread safe
unsafe impl<MODE> Sync for Pin<MODE> {}
// NOTE(unsafe) this only enables read access to the same pin from multiple
// threads
unsafe impl<MODE> Send for Pin<MODE> {}

impl<MODE> PinExt for Pin<MODE> {
    type Mode = MODE;

    fn pin_id(&self) -> u8 {
        self.i
    }

    fn port_id(&self) -> u8 {
        self.p
    }
}

macro_rules! gpio_trait {
    ($gpiox:ident) => {
        impl GpioRegExt for crate::pac::$gpiox::RegisterBlock {
            fn is_low(&self, pos: u8) -> bool {
                // NOTE(unsafe) atomic read with no side effects
                self.idr.read().bits() & (1 << pos) == 0
            }

            fn is_set_low(&self, pos: u8) -> bool {
                // NOTE(unsafe) atomic read with no side effects
                self.odr.read().bits() & (1 << pos) == 0
            }

            fn set_high(&self, pos: u8) {
                // NOTE(unsafe) atomic write to a stateless register
                unsafe { self.bsrr.write(|w| w.bits(1 << pos)) }
            }

            fn set_low(&self, pos: u8) {
                // NOTE(unsafe) atomic write to a stateless register
                unsafe { self.bsrr.write(|w| w.bits(1 << (pos + 16))) }
            }
        }
    };
}

gpio_trait!(gpioa);

#[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002a"))]
gpio_trait!(gpiof);
#[cfg(feature = "py32f002b")]
gpio_trait!(gpioc);

macro_rules! gpio {
    ([$($GPIOX:ident, $gpiox:ident, $PXx:ident, $PCH:literal, $gate:meta => [
        $($PXi:ident: ($pxi:ident, $i:expr, $MODE:ty),)+
    ]),+]) => {
        $(
            /// GPIO
             #[cfg($gate)]
            pub mod $gpiox {
                use core::marker::PhantomData;

                use crate::{
                    rcc::{Enable, Reset},
                    pac::RCC,
                    pac::$GPIOX
                };

                use super::{
                    Alternate, Analog, Floating, GpioExt, Input, OpenDrain, Output,
                    PullDown, PullUp, PushPull, AF0, AF1, AF2, AF3, AF4, AF5, AF6, AF7,
                    AF8, AF9, AF10, AF11, AF12, AF13, AF14, AF15, Pin, GpioRegExt,
                };

                /// GPIO parts
                pub struct Parts {
                    $(
                        /// Pin
                        pub $pxi: $PXi<$MODE>,
                    )+
                }

                impl GpioExt for $GPIOX {
                    type Parts = Parts;

                    fn split(self) -> Parts {
                        let rcc = unsafe { &(*RCC::ptr()) };
                        $GPIOX::enable(rcc);
                        $GPIOX::reset(rcc);

                        Parts {
                            $(
                                $pxi: $PXi { _mode: PhantomData },
                            )+
                        }
                    }

                    unsafe fn split_without_reset(self) -> Parts {
                        let rcc = unsafe { &(*RCC::ptr()) };
                        $GPIOX::enable(rcc);

                        Parts {
                            $(
                                $pxi: $PXi { _mode: PhantomData },
                            )+
                        }
                    }
                }

                fn _set_alternate_mode (index:usize, mode: u32)
                {
                    let offset = 2 * index;
                    let offset2 = 4 * index;
                    unsafe {
                        let reg = &(*$GPIOX::ptr());
                        if offset2 < 32 {
                            reg.afrl.modify(|r, w| {
                                w.bits((r.bits() & !(0b1111 << offset2)) | (mode << offset2))
                            });
                        } else {
                            #[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002a"))] {
                                let offset2 = offset2 - 32;
                                reg.afrh.modify(|r, w| {
                                    w.bits((r.bits() & !(0b1111 << offset2)) | (mode << offset2))
                                });
                            }

                            #[cfg(not(any(feature = "py32f030", feature = "py32f003", feature = "py32f002a")))] {
                                // py32f002b does only have a maximum of 8 pins per port so it does not have AFRH
                                unreachable!();
                            }
                        }
                        reg.moder.modify(|r, w| {
                            w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset))
                        });
                    }
                }

                $(
                    /// Pin
                    pub struct $PXi<MODE> {
                        _mode: PhantomData<MODE>,
                    }

                    impl<MODE> $PXi<MODE> {
                        /// Configures the pin to operate in AF0 mode
                        pub fn into_alternate_af0(
                            self,
                        ) -> $PXi<Alternate<AF0>> {
                            _set_alternate_mode($i, 0);
                            $PXi { _mode: PhantomData }
                        }

                        /// Configures the pin to operate in AF1 mode
                        pub fn into_alternate_af1(
                            self,
                        ) -> $PXi<Alternate<AF1>> {
                            _set_alternate_mode($i, 1);
                            $PXi { _mode: PhantomData }
                        }

                        /// Configures the pin to operate in AF2 mode
                        pub fn into_alternate_af2(
                            self,
                        ) -> $PXi<Alternate<AF2>> {
                            _set_alternate_mode($i, 2);
                            $PXi { _mode: PhantomData }
                        }

                        /// Configures the pin to operate in AF3 mode
                        pub fn into_alternate_af3(
                            self,
                        ) -> $PXi<Alternate<AF3>> {
                            _set_alternate_mode($i, 3);
                            $PXi { _mode: PhantomData }
                        }

                        /// Configures the pin to operate in AF4 mode
                        pub fn into_alternate_af4(
                            self,
                        ) -> $PXi<Alternate<AF4>> {
                            _set_alternate_mode($i, 4);
                            $PXi { _mode: PhantomData }
                        }

                        /// Configures the pin to operate in AF5 mode
                        pub fn into_alternate_af5(
                            self,
                        ) -> $PXi<Alternate<AF5>> {
                            _set_alternate_mode($i, 5);
                            $PXi { _mode: PhantomData }
                        }

                        /// Configures the pin to operate in AF6 mode
                        pub fn into_alternate_af6(
                            self,
                        ) -> $PXi<Alternate<AF6>> {
                            _set_alternate_mode($i, 6);
                            $PXi { _mode: PhantomData }
                        }

                        /// Configures the pin to operate in AF7 mode
                        pub fn into_alternate_af7(
                            self,
                        ) -> $PXi<Alternate<AF7>> {
                            _set_alternate_mode($i, 7);
                            $PXi { _mode: PhantomData }
                        }

                        /// Configures the pin to operate in AF8 mode
                        pub fn into_alternate_af8(
                            self,
                        ) -> $PXi<Alternate<AF8>> {
                            _set_alternate_mode($i, 8);
                            $PXi { _mode: PhantomData }
                        }

                        /// Configures the pin to operate in AF9 mode
                        pub fn into_alternate_af9(
                            self,
                        ) -> $PXi<Alternate<AF9>> {
                            _set_alternate_mode($i, 9);
                            $PXi { _mode: PhantomData }
                        }

                        /// Configures the pin to operate in AF10 mode
                        pub fn into_alternate_af10(
                            self,
                        ) -> $PXi<Alternate<AF10>> {
                            _set_alternate_mode($i, 10);
                            $PXi { _mode: PhantomData }
                        }

                        /// Configures the pin to operate in AF11 mode
                        pub fn into_alternate_af11(
                            self,
                        ) -> $PXi<Alternate<AF11>> {
                            _set_alternate_mode($i, 11);
                            $PXi { _mode: PhantomData }
                        }

                        /// Configures the pin to operate in AF12 mode
                        pub fn into_alternate_af12(
                            self,
                        ) -> $PXi<Alternate<AF12>> {
                            _set_alternate_mode($i, 12);
                            $PXi { _mode: PhantomData }
                        }

                        /// Configures the pin to operate in AF13 mode
                        pub fn into_alternate_af13(
                            self,
                        ) -> $PXi<Alternate<AF13>> {
                            _set_alternate_mode($i, 13);
                            $PXi { _mode: PhantomData }
                        }

                        /// Configures the pin to operate in AF14 mode
                        pub fn into_alternate_af14(
                            self,
                        ) -> $PXi<Alternate<AF14>> {
                            _set_alternate_mode($i, 14);
                            $PXi { _mode: PhantomData }
                        }

                        /// Configures the pin to operate in AF15 mode
                        pub fn into_alternate_af15(
                            self,
                        ) -> $PXi<Alternate<AF15>> {
                            _set_alternate_mode($i, 15);
                            $PXi { _mode: PhantomData }
                        }

                        /// Configures the pin to operate as a floating input pin
                        pub fn into_floating_input(
                            self,
                        ) -> $PXi<Input<Floating>> {
                            let offset = 2 * $i;
                            unsafe {
                                let reg = &(*$GPIOX::ptr());
                                reg.pupdr.modify(|r, w| {
                                    w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                                });
                                reg.moder.modify(|r, w| {
                                    w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                                });
                            }
                            $PXi { _mode: PhantomData }
                        }

                        /// Configures the pin to operate as a pulled down input pin
                        pub fn into_pull_down_input(
                            self,
                            ) -> $PXi<Input<PullDown>> {
                            let offset = 2 * $i;
                            unsafe {
                                let reg = &(*$GPIOX::ptr());
                                reg.pupdr.modify(|r, w| {
                                    w.bits((r.bits() & !(0b11 << offset)) | (0b10 << offset))
                                });
                                reg.moder.modify(|r, w| {
                                    w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                                });
                            }
                            $PXi { _mode: PhantomData }
                        }

                        /// Configures the pin to operate as a pulled up input pin
                        pub fn into_pull_up_input(
                            self,
                        ) -> $PXi<Input<PullUp>> {
                            let offset = 2 * $i;
                            unsafe {
                                let reg = &(*$GPIOX::ptr());
                                reg.pupdr.modify(|r, w| {
                                    w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset))
                                });
                                reg.moder.modify(|r, w| {
                                    w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                                });
                            }
                            $PXi { _mode: PhantomData }
                        }

                        /// Configures the pin to operate as an analog pin
                        pub fn into_analog(
                            self,
                        ) -> $PXi<Analog> {
                            let offset = 2 * $i;
                            unsafe {
                                let reg = &(*$GPIOX::ptr());
                                reg.pupdr.modify(|r, w| {
                                    w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                                });
                                reg.moder.modify(|r, w| {
                                    w.bits((r.bits() & !(0b11 << offset)) | (0b11 << offset))
                                });
                            }
                            $PXi { _mode: PhantomData }
                        }

                        /// Configures the pin to operate as an open drain output pin
                        pub fn into_open_drain_output(
                            self,
                        ) -> $PXi<Output<OpenDrain>> {
                            let offset = 2 * $i;
                            unsafe {
                                let reg = &(*$GPIOX::ptr());
                                reg.pupdr.modify(|r, w| {
                                    w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                                });
                                reg.otyper.modify(|r, w| {
                                    w.bits(r.bits() | (0b1 << $i))
                                });
                                reg.moder.modify(|r, w| {
                                    w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset))
                                });
                            }
                            $PXi { _mode: PhantomData }
                        }

                        /// Configures the pin to operate as an push pull output pin
                        pub fn into_push_pull_output(
                            self,
                        ) -> $PXi<Output<PushPull>> {
                            let offset = 2 * $i;
                            unsafe {
                                let reg = &(*$GPIOX::ptr());
                                reg.pupdr.modify(|r, w| {
                                    w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                                });
                                reg.otyper.modify(|r, w| {
                                    w.bits(r.bits() & !(0b1 << $i))
                                });
                                reg.moder.modify(|r, w| {
                                    w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset))
                                });
                            }
                            $PXi { _mode: PhantomData }
                        }

                        /// Configures the pin to operate as an push pull output pin with quick fall
                        /// and rise times
                        pub fn into_push_pull_output_hs(
                            self,
                        ) -> $PXi<Output<PushPull>> {
                            let offset = 2 * $i;
                            unsafe {
                                let reg = &(*$GPIOX::ptr());
                                reg.pupdr.modify(|r, w| {
                                    w.bits((r.bits() & !(0b11 << offset)) | (0b00 << offset))
                                });
                                reg.otyper.modify(|r, w| {
                                    w.bits(r.bits() & !(0b1 << $i))
                                });
                                reg.ospeedr.modify(|r, w| {
                                    w.bits(r.bits() & !(0b1 << $i))
                                });
                                reg.moder.modify(|r, w| {
                                    w.bits((r.bits() & !(0b11 << offset)) | (0b01 << offset))
                                });
                            }
                            $PXi { _mode: PhantomData }
                        }
                    }

                    impl $PXi<Output<OpenDrain>> {
                        /// Enables / disables the internal pull up
                        pub fn internal_pull_up(&mut self, on: bool) {
                            let offset = 2 * $i;
                            let value = if on { 0b01 } else { 0b00 };
                            unsafe {
                                let reg = &(*$GPIOX::ptr());
                                reg.pupdr.modify(|r, w| {
                                    w.bits((r.bits() & !(0b11 << offset)) | (value << offset))
                                });
                            }
                        }
                    }

                    impl<AF> $PXi<Alternate<AF>> {
                        /// Enables / disables the internal pull up
                        pub fn internal_pull_up(self, on: bool) -> Self {
                            let offset = 2 * $i;
                            let value = if on { 0b01 } else { 0b00 };
                            unsafe {
                                let reg = &(*$GPIOX::ptr());
                                reg.pupdr.modify(|r, w| {
                                    w.bits((r.bits() & !(0b11 << offset)) | (value << offset))
                                });
                            }
                            self
                        }
                    }

                    impl<AF> $PXi<Alternate<AF>> {
                        /// Turns pin alternate configuration pin into open drain
                        pub fn set_open_drain(self) -> Self {
                            let offset = $i;
                            unsafe {
                                let reg = &(*$GPIOX::ptr());
                                reg.otyper.modify(|r, w| {
                                    w.bits(r.bits() | (1 << offset))
                                });
                            }
                            self
                        }
                    }

                    impl<MODE> $PXi<Output<MODE>> {
                        pub fn is_set_high(&mut self) -> bool {
                            !self.is_set_low()
                        }

                        pub fn is_set_low(&mut self) -> bool {
                            unsafe { (*$GPIOX::ptr()).is_set_low($i) }
                        }

                        pub fn set_high(&mut self) {
                            unsafe { (*$GPIOX::ptr()).set_high($i) };
                        }

                        #[allow(dead_code)]
                        pub fn set_low(&mut self) {
                            unsafe { (*$GPIOX::ptr()).set_low($i) };
                        }

                        #[allow(dead_code)]
                        pub fn is_high(&mut self) -> bool {
                            !self.is_low()
                        }

                        #[allow(dead_code)]
                        pub fn is_low(&mut self) -> bool {
                            unsafe { (*$GPIOX::ptr()).is_low($i) }
                        }

                        pub fn toggle(&mut self) {
                            if self.is_low() {
                                self.set_high();
                            } else {
                                self.set_low();
                            }
                        }

                        /// Erases the pin number from the type
                        ///
                        /// This is useful when you want to collect the pins into an array where you
                        /// need all the elements to have the same type
                        pub fn downgrade(self) -> Pin<Output<MODE>> {
                            Pin {
                                i: $i,
                                p: $PCH,
                                port: $GPIOX::ptr() as *const dyn GpioRegExt,
                                _mode: self._mode,
                            }
                        }
                    }

                    impl<MODE> $PXi<Input<MODE>> {
                        pub fn is_high(&mut self) -> bool {
                            !self.is_low()
                        }

                        pub fn is_low(&mut self) -> bool {
                            unsafe { (*$GPIOX::ptr()).is_low($i) }
                        }

                        /// Erases the pin number from the type
                        ///
                        /// This is useful when you want to collect the pins into an array where you
                        /// need all the elements to have the same type
                        pub fn downgrade(self) -> Pin<Input<MODE>> {
                            Pin {
                                i: $i,
                                p: $PCH,
                                port: $GPIOX::ptr() as *const dyn GpioRegExt,
                                _mode: self._mode,
                            }
                        }
                    }

                )+
            }
        )+
    }
}

gpio!([
    // BUGBUG: py32f002b only has 8 pins?
    GPIOA, gpioa, PA, b'A', any(
        feature = "device-selected"
    ) => [
        PA0: (pa0, 0, Input<Floating>),
        PA1: (pa1, 1, Input<Floating>),
        PA2: (pa2, 2, Input<Floating>),
        PA3: (pa3, 3, Input<Floating>),
        PA4: (pa4, 4, Input<Floating>),
        PA5: (pa5, 5, Input<Floating>),
        PA6: (pa6, 6, Input<Floating>),
        PA7: (pa7, 7, Input<Floating>),
        PA8: (pa8, 8, Input<Floating>),
        PA9: (pa9, 9, Input<Floating>),
        PA10: (pa10, 10, Input<Floating>),
        PA11: (pa11, 11, Input<Floating>),
        PA12: (pa12, 12, Input<Floating>),
        PA13: (pa13, 13, Input<Floating>),
        PA14: (pa14, 14, Input<Floating>),
        PA15: (pa15, 15, Input<Floating>),
    ],
    GPIOB, gpiob, PB, b'B', any(
        feature = "device-selected"
    ) => [
        PB0: (pb0, 0, Input<Floating>),
        PB1: (pb1, 1, Input<Floating>),
        PB2: (pb2, 2, Input<Floating>),
        PB3: (pb3, 3, Input<Floating>),
        PB4: (pb4, 4, Input<Floating>),
        PB5: (pb5, 5, Input<Floating>),
        PB6: (pb6, 6, Input<Floating>),
        PB7: (pb7, 7, Input<Floating>),
        PB8: (pb8, 8, Input<Floating>),
    ],
    GPIOC, gpioc, PC, b'C', any(
        feature = "py32f002b"
    ) => [
        PC0: (pf0, 0, Input<Floating>),
        PC1: (pf1, 1, Input<Floating>),
    ],
    GPIOF, gpiof, PF, b'F', any(
        feature = "py32f030",
        feature = "py32f003",
        feature = "py32f002a"
    ) => [
        PF0: (pf0, 0, Input<Floating>),
        PF1: (pf1, 1, Input<Floating>),
        PF2: (pf2, 2, Input<Floating>),
        PF3: (pf3, 3, Input<Floating>),
        PF4: (pf4, 4, Input<Floating>),
    ]
]);
