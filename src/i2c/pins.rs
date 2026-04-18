//! Pins for I2c peripheral

use crate::{
    gpio::*,
};

/// Trait for identifying SCL pins
pub trait SclPin<I2C> {}
/// Trait for identifying SDA pins
pub trait SdaPin<I2C> {}

macro_rules! i2c_pins {
    ($($I2C:ident => {
        scl => [$($scl:ty),+ $(,)*],
        sda => [$($sda:ty),+ $(,)*],
    })+) => {
        $(
            $(
                impl SclPin<crate::pac::$I2C> for $scl {}
            )+
            $(
                impl SdaPin<crate::pac::$I2C> for $sda {}
            )+
        )+
    }
}

#[cfg(feature = "py32f030")]
i2c_pins! {
    I2C => {
        scl => [
            gpioa::PA3<Alternate<AF12>>,
            gpioa::PA8<Alternate<AF12>>,
            gpioa::PA9<Alternate<AF6>>,
            gpioa::PA10<Alternate<AF12>>,
            gpioa::PA11<Alternate<AF6>>,
            gpiob::PB6<Alternate<AF6>>,
            gpiob::PB8<Alternate<AF6>>,
            gpiof::PF1<Alternate<AF12>>
        ],
        sda => [
            gpioa::PA2<Alternate<AF12>>,
            gpioa::PA7<Alternate<AF12>>,
            gpioa::PA9<Alternate<AF12>>,
            gpioa::PA10<Alternate<AF6>>,
            gpioa::PA12<Alternate<AF6>>,
            gpiob::PB7<Alternate<AF6>>,
            gpiof::PF0<Alternate<AF12>>
        ],
    }
}

#[cfg(feature = "py32f002a")]
i2c_pins! {
    I2C => {
        scl => [
            gpioa::PA3<Alternate<AF12>>,
            gpioa::PA8<Alternate<AF12>>,
            gpioa::PA9<Alternate<AF6>>,
            gpioa::PA10<Alternate<AF12>>,
            gpiob::PB6<Alternate<AF6>>,
            gpiob::PB8<Alternate<AF6>>,
            gpiof::PF1<Alternate<AF12>>
        ],
        sda => [
            gpioa::PA2<Alternate<AF12>>,
            gpioa::PA7<Alternate<AF12>>,
            gpioa::PA9<Alternate<AF12>>,
            gpioa::PA10<Alternate<AF6>>,
            gpioa::PA12<Alternate<AF6>>,
            gpiob::PB7<Alternate<AF6>>,
            gpiof::PF0<Alternate<AF12>>
        ],
    }
}

#[cfg(feature = "py32f003")]
i2c_pins! {
    I2C => {
        scl => [
            gpioa::PA3<Alternate<AF12>>,
            gpiob::PB6<Alternate<AF6>>,
            gpiof::PF1<Alternate<AF12>>
        ],
        sda => [
            gpioa::PA2<Alternate<AF12>>,
            gpioa::PA7<Alternate<AF12>>,
            gpioa::PA12<Alternate<AF6>>,
            gpiob::PB7<Alternate<AF6>>,
            gpiof::PF0<Alternate<AF12>>
        ],
    }
}

#[cfg(feature = "py32f002b")]
i2c_pins! {
    I2C => {
        scl => [
            gpioa::PA2<Alternate<AF6>>,
            gpiob::PB3<Alternate<AF6>>,
        ],
        sda => [
            gpiob::PB4<Alternate<AF6>>,
            gpiob::PB6<Alternate<AF6>>,
        ],
    }
}
