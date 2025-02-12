use super::*;

/// Type alias for an [ErasedPin]
pub type AnyPin<MODE> = ErasedPin<MODE>;

macro_rules! impl_pxx {
    ($(($port_id:literal :: $pin:ident)),*) => {
        /// Erased pin
        ///
        /// `MODE` is one of the pin modes (see [Modes](crate::gpio#modes) section).
        pub enum ErasedPin<MODE> {
            $(
                /// Partially erased pin
                $pin(PartiallyErasedPin<$port_id, MODE>)
            ),*
        }

        impl<MODE> PinExt for ErasedPin<MODE> {
            type Mode = MODE;

            #[inline(always)]
            fn pin_id(&self) -> u8 {
                match self {
                    $(Self::$pin(pin) => pin.pin_id()),*
                }
            }

            #[inline(always)]
            fn port_id(&self) -> u8 {
                match self {
                    $(Self::$pin(pin) => pin.port_id()),*
                }
            }
        }

        impl<MODE> ErasedPin<Output<MODE>> {
            /// Set a pin to high level
            pub fn set_high(&mut self) {
                match self {
                    $(Self::$pin(pin) => pin.set_high()),*
                }
            }
            /// Set a pin to low level
            pub fn set_low(&mut self) {
                match self {
                    $(Self::$pin(pin) => pin.set_low()),*
                }
            }
            /// returns true if pin is set to high level
            pub fn is_set_high(&self) -> bool {
                match self {
                    $(Self::$pin(pin) => pin.is_set_high()),*
                }
            }
            /// returns true if pin is set to low level
            pub fn is_set_low(&self) -> bool {
                match self {
                    $(Self::$pin(pin) => pin.is_set_low()),*
                }
            }
        }

        impl<MODE> ErasedPin<Input<MODE>> {
            /// returns true if pin is at high level
            pub fn is_high(&self) -> bool {
                match self {
                    $(Self::$pin(pin) => pin.is_high()),*
                }
            }
            /// returns true if pin is at low level
            pub fn is_low(&self) -> bool {
                match self {
                    $(Self::$pin(pin) => pin.is_low()),*
                }
            }
        }

        impl ErasedPin<Output<OpenDrain>> {
            /// returns true if pin is at high level
            pub fn is_high(&self) -> bool {
                match self {
                    $(Self::$pin(pin) => pin.is_high()),*
                }
            }

            /// returns true if pin is at low level
            pub fn is_low(&self) -> bool {
                match self {
                    $(Self::$pin(pin) => pin.is_low()),*
                }
            }
        }
    }
}

impl<MODE> ErasedPin<Output<MODE>> {
    /// Get the [PinState] of a pin
    #[inline(always)]
    pub fn get_state(&self) -> PinState {
        if self.is_set_low() {
            PinState::Low
        } else {
            PinState::High
        }
    }

    /// Set the [PinState] of a pin
    #[inline(always)]
    pub fn set_state(&mut self, state: PinState) {
        match state {
            PinState::Low => self.set_low(),
            PinState::High => self.set_high(),
        }
    }

    /// Toggle the pin from a high level to low level, or the reverse
    #[inline(always)]
    pub fn toggle(&mut self) {
        if self.is_set_low() {
            self.set_high()
        } else {
            self.set_low()
        }
    }
}

#[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002a"))]
impl_pxx! {
    ('A'::PAx),
    ('B'::PBx),
    ('F'::PFx)
}

#[cfg(feature = "py32f002b")]
impl_pxx! {
    ('A'::PAx),
    ('B'::PBx),
    ('C'::PCx)
}
