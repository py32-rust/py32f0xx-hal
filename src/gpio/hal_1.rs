use core::convert::Infallible;

use super::{Input, OpenDrain, Output, Pin};

//pub use embedded_hal::digital::PinState;
use embedded_hal::digital::{ErrorType, InputPin, OutputPin, StatefulOutputPin};

// fn into_state(state: PinState) -> super::PinState {
//     match state {
//         PinState::Low => super::PinState::Low,
//         PinState::High => super::PinState::High,
//     }
// }

// Implementations for `Pin`
impl<MODE> ErrorType for Pin<Output<MODE>> {
    type Error = Infallible;
}
impl<MODE> ErrorType for Pin<Input<MODE>> {
    type Error = Infallible;
}

impl<MODE> StatefulOutputPin for Pin<Output<MODE>> {
    #[inline(always)]
    fn is_set_high(&mut self) -> Result<bool, Self::Error> {
        self.is_set_low().map(|v| !v)
    }

    #[inline(always)]
    fn is_set_low(&mut self) -> Result<bool, Self::Error> {
        Ok(unsafe { (*self.port).is_set_low(self.i) })
    }
}

impl<MODE> OutputPin for Pin<Output<MODE>> {
    #[inline(always)]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        unsafe { (*self.port).set_high(self.i) };
        Ok(())
    }

    #[inline(always)]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        unsafe { (*self.port).set_low(self.i) }
        Ok(())
    }
}

impl InputPin for Pin<Output<OpenDrain>> {
    #[inline(always)]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        self.is_low().map(|v| !v)
    }

    #[inline(always)]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(unsafe { (*self.port).is_low(self.i) })
    }
}

impl<MODE> InputPin for Pin<Input<MODE>> {
    #[inline(always)]
    fn is_high(&mut self) -> Result<bool, Self::Error> {
        self.is_low().map(|v| !v)
    }

    #[inline(always)]
    fn is_low(&mut self) -> Result<bool, Self::Error> {
        Ok(unsafe { (*self.port).is_low(self.i) })
    }
}

// // PartiallyErasedPin

// impl<const P: char, MODE> ErrorType for PartiallyErasedPin<P, MODE> {
//     type Error = Infallible;
// }

// impl<const P: char, MODE> OutputPin for PartiallyErasedPin<P, Output<MODE>> {
//     #[inline(always)]
//     fn set_high(&mut self) -> Result<(), Self::Error> {
//         self.set_high();
//         Ok(())
//     }

//     #[inline(always)]
//     fn set_low(&mut self) -> Result<(), Self::Error> {
//         self.set_low();
//         Ok(())
//     }
// }

// impl<const P: char, MODE> StatefulOutputPin for PartiallyErasedPin<P, Output<MODE>> {
//     #[inline(always)]
//     fn is_set_high(&mut self) -> Result<bool, Self::Error> {
//         Ok((*self).is_set_high())
//     }

//     #[inline(always)]
//     fn is_set_low(&mut self) -> Result<bool, Self::Error> {
//         Ok((*self).is_set_low())
//     }
// }

// impl<const P: char> InputPin for PartiallyErasedPin<P, Output<OpenDrain>> {
//     #[inline(always)]
//     fn is_high(&mut self) -> Result<bool, Self::Error> {
//         Ok((*self).is_high())
//     }

//     #[inline(always)]
//     fn is_low(&mut self) -> Result<bool, Self::Error> {
//         Ok((*self).is_low())
//     }
// }

// impl<const P: char, MODE> InputPin for PartiallyErasedPin<P, Input<MODE>> {
//     #[inline(always)]
//     fn is_high(&mut self) -> Result<bool, Self::Error> {
//         Ok((*self).is_high())
//     }

//     #[inline(always)]
//     fn is_low(&mut self) -> Result<bool, Self::Error> {
//         Ok((*self).is_low())
//     }
// }

// // ErasedPin

// impl<MODE> ErrorType for ErasedPin<MODE> {
//     type Error = core::convert::Infallible;
// }

// impl<MODE> OutputPin for ErasedPin<Output<MODE>> {
//     fn set_high(&mut self) -> Result<(), Infallible> {
//         self.set_high();
//         Ok(())
//     }

//     fn set_low(&mut self) -> Result<(), Infallible> {
//         self.set_low();
//         Ok(())
//     }
// }

// impl<MODE> StatefulOutputPin for ErasedPin<Output<MODE>> {
//     fn is_set_high(&mut self) -> Result<bool, Self::Error> {
//         Ok((*self).is_set_high())
//     }

//     fn is_set_low(&mut self) -> Result<bool, Self::Error> {
//         Ok((*self).is_set_low())
//     }
// }

// impl<MODE> InputPin for ErasedPin<Input<MODE>> {
//     fn is_high(&mut self) -> Result<bool, Infallible> {
//         Ok((*self).is_high())
//     }

//     fn is_low(&mut self) -> Result<bool, Infallible> {
//         Ok((*self).is_low())
//     }
// }

// impl InputPin for ErasedPin<Output<OpenDrain>> {
//     fn is_high(&mut self) -> Result<bool, Infallible> {
//         Ok((*self).is_high())
//     }

//     fn is_low(&mut self) -> Result<bool, Infallible> {
//         Ok((*self).is_low())
//     }
// }
