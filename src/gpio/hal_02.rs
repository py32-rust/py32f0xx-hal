use super::*;
use core::convert::Infallible;
use embedded_hal_02::digital::v2::{toggleable, InputPin, OutputPin, StatefulOutputPin};

// Pin

impl<MODE> OutputPin for Pin<Output<MODE>> {
    type Error = Infallible;
    #[inline]
    fn set_high(&mut self) -> Result<(), Self::Error> {
        unsafe { (*self.port).set_high(self.i) };
        Ok(())
    }
    #[inline]
    fn set_low(&mut self) -> Result<(), Self::Error> {
        unsafe { (*self.port).set_low(self.i) };
        Ok(())
    }
}

impl<MODE> StatefulOutputPin for Pin<Output<MODE>> {
    #[inline]
    fn is_set_high(&self) -> Result<bool, Self::Error> {
        self.is_set_low().map(|v| !v)
    }
    #[inline]
    fn is_set_low(&self) -> Result<bool, Self::Error> {
        Ok(unsafe { (*self.port).is_set_low(self.i) })
    }
}

impl<MODE> InputPin for Pin<Input<MODE>> {
    type Error = Infallible;
    #[inline]
    fn is_high(&self) -> Result<bool, Self::Error> {
        self.is_low().map(|v| !v)
    }

    #[inline]
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(unsafe { (*self.port).is_low(self.i) })
    }
}

impl InputPin for Pin<Output<OpenDrain>> {
    type Error = Infallible;
    #[inline]
    fn is_high(&self) -> Result<bool, Self::Error> {
        self.is_low().map(|v| !v)
    }

    #[inline]
    fn is_low(&self) -> Result<bool, Self::Error> {
        Ok(unsafe { (*self.port).is_low(self.i) })
    }
}

impl<MODE> toggleable::Default for Pin<Output<MODE>> {}

// PartiallyErasedPin

// impl<const P: char, MODE> OutputPin for PartiallyErasedPin<P, Output<MODE>> {
//     type Error = Infallible;

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
//     fn is_set_high(&self) -> Result<bool, Self::Error> {
//         Ok(self.is_set_high())
//     }

//     #[inline(always)]
//     fn is_set_low(&self) -> Result<bool, Self::Error> {
//         Ok(self.is_set_low())
//     }
// }

// impl<const P: char, MODE> ToggleableOutputPin for PartiallyErasedPin<P, Output<MODE>> {
//     type Error = Infallible;

//     #[inline(always)]
//     fn toggle(&mut self) -> Result<(), Self::Error> {
//         self.toggle();
//         Ok(())
//     }
// }

// impl<const P: char> InputPin for PartiallyErasedPin<P, Output<OpenDrain>> {
//     type Error = Infallible;

//     #[inline(always)]
//     fn is_high(&self) -> Result<bool, Self::Error> {
//         Ok(self.is_high())
//     }

//     #[inline(always)]
//     fn is_low(&self) -> Result<bool, Self::Error> {
//         Ok(self.is_low())
//     }
// }

// impl<const P: char, MODE> InputPin for PartiallyErasedPin<P, Input<MODE>> {
//     type Error = Infallible;

//     #[inline(always)]
//     fn is_high(&self) -> Result<bool, Self::Error> {
//         Ok(self.is_high())
//     }

//     #[inline(always)]
//     fn is_low(&self) -> Result<bool, Self::Error> {
//         Ok(self.is_low())
//     }
// }

// ErasedPin

// impl<MODE> OutputPin for ErasedPin<Output<MODE>> {
//     type Error = Infallible;
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
//     fn is_set_high(&self) -> Result<bool, Self::Error> {
//         Ok(self.is_set_high())
//     }

//     fn is_set_low(&self) -> Result<bool, Self::Error> {
//         Ok(self.is_set_low())
//     }
// }

// impl<MODE> InputPin for ErasedPin<Input<MODE>> {
//     type Error = Infallible;
//     fn is_high(&self) -> Result<bool, Infallible> {
//         Ok(self.is_high())
//     }

//     fn is_low(&self) -> Result<bool, Infallible> {
//         Ok(self.is_low())
//     }
// }

// impl InputPin for ErasedPin<Output<OpenDrain>> {
//     type Error = Infallible;
//     fn is_high(&self) -> Result<bool, Infallible> {
//         Ok(self.is_high())
//     }

//     fn is_low(&self) -> Result<bool, Infallible> {
//         Ok(self.is_low())
//     }
// }
