use crate::gpio::AF2;
use crate::gpio::{gpioa::*, gpiob::*, Alternate};
use crate::pac;

#[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002a"))]
use crate::gpio::{AF1, AF13, AF14};

#[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002b"))]
use crate::gpio::AF5;

#[cfg(any(feature = "py32f030", feature = "py32f003"))]
use crate::gpio::{gpiof::*, AF0, AF4};

#[cfg(feature = "py32f002b")]
use crate::gpio::{gpioc::*, AF3};

// Output channels marker traits
/// Timer channel 1 Output Pin
pub trait PinC1<TIM> {}
/// Timer channel 1 Complementary Pin
pub trait PinC1N<TIM> {}
/// Timer channel 2 Output Pin
pub trait PinC2<TIM> {}
/// Timer channel 2 Complementary Pin
pub trait PinC2N<TIM> {}
/// Timer channel 3 Output Pin
pub trait PinC3<TIM> {}
/// Timer channel 3 Complementary Pin
pub trait PinC3N<TIM> {}
/// Timer channel 4 Output Pin
pub trait PinC4<TIM> {}

macro_rules! channel_impl {
    ( $( $TIM:ty, $PINC:ident, $PINX:ident, $MODE:ident<$AF:ident>; )+ ) => {
        $(
            impl $PINC<$TIM> for $PINX<$MODE<$AF>> {}
        )+
    };
}

#[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002a"))]
channel_impl!(
    pac::TIM1, PinC3, PA0, Alternate<AF13>;
    pac::TIM1, PinC1N, PA0, Alternate<AF14>;
    pac::TIM1, PinC4, PA1, Alternate<AF13>;
    pac::TIM1, PinC2N, PA1, Alternate<AF14>;
    pac::TIM1, PinC1, PA3, Alternate<AF13>;
    pac::TIM1, PinC1N, PA7, Alternate<AF2>;
    pac::TIM1, PinC2, PA13, Alternate<AF13>;
    pac::TIM1, PinC2N, PB0, Alternate<AF2>;
    pac::TIM1, PinC3N, PB1, Alternate<AF2>;
    pac::TIM1, PinC3, PB6, Alternate<AF1>;
);

#[cfg(any(feature = "py32f030", feature = "py32f002a"))]
channel_impl!(
    pac::TIM1, PinC1, PA8, Alternate<AF2>;
    pac::TIM1, PinC2, PA9, Alternate<AF2>;
    pac::TIM1, PinC3, PA10, Alternate<AF2>;
    pac::TIM1, PinC4, PA11, Alternate<AF2>;
    pac::TIM1, PinC2, PB3, Alternate<AF1>;
);

#[cfg(any(feature = "py32f030", feature = "py32f003"))]
channel_impl!(
    pac::TIM3, PinC1, PA2, Alternate<AF13>;
    pac::TIM3, PinC3, PA4, Alternate<AF13>;
    pac::TIM3, PinC2, PA5, Alternate<AF13>;
    pac::TIM3, PinC1, PA6, Alternate<AF1>;
    pac::TIM3, PinC2, PA7, Alternate<AF1>;
    pac::TIM3, PinC3, PB0, Alternate<AF1>;
    pac::TIM3, PinC4, PB1, Alternate<AF1>;
    pac::TIM3, PinC2, PB5, Alternate<AF1>;

    pac::TIM14, PinC1, PA4, Alternate<AF4>;
    pac::TIM14, PinC1, PA7, Alternate<AF4>;
    pac::TIM14, PinC1, PB1, Alternate<AF0>;
    pac::TIM14, PinC1, PF0, Alternate<AF2>;
    pac::TIM14, PinC1, PF1, Alternate<AF13>;

    pac::TIM16, PinC1, PA6, Alternate<AF5>;
    pac::TIM16, PinC1N, PB6, Alternate<AF2>;

    pac::TIM17, PinC1, PA7, Alternate<AF5>;
    pac::TIM17, PinC1N, PB7, Alternate<AF2>;
);

#[cfg(feature = "py32f030")]
channel_impl!(
    pac::TIM3, PinC1, PB4, Alternate<AF1>;

    pac::TIM16, PinC1, PB8, Alternate<AF2>;

    pac::TIM17, PinC1, PB8, Alternate<AF13>;
);

#[cfg(feature = "py32f002b")]
channel_impl!(
    pac::TIM1, PinC1, PA0, Alternate<AF2>;
    pac::TIM1, PinC2, PA1, Alternate<AF2>;
    pac::TIM1, PinC4, PA2, Alternate<AF2>;
    pac::TIM1, PinC2, PA3, Alternate<AF2>;
    pac::TIM1, PinC3, PA4, Alternate<AF2>;
    pac::TIM1, PinC1, PA5, Alternate<AF2>;
    pac::TIM1, PinC4, PA7, Alternate<AF2>;
    pac::TIM1, PinC2, PB0, Alternate<AF2>;
    pac::TIM1, PinC3N, PB0, Alternate<AF3>;
    pac::TIM1, PinC2N, PB1, Alternate<AF2>;
    pac::TIM1, PinC4, PB1, Alternate<AF3>;
    pac::TIM1, PinC1N, PB2, Alternate<AF2>;
    pac::TIM1, PinC3, PB2, Alternate<AF3>;
    pac::TIM1, PinC3, PB5, Alternate<AF2>;
    pac::TIM1, PinC1N, PC0, Alternate<AF2>;

    pac::TIM14, PinC1, PA4, Alternate<AF5>;
    pac::TIM14, PinC1, PA5, Alternate<AF5>;
    pac::TIM14, PinC1, PB5, Alternate<AF5>;
    pac::TIM14, PinC1, PB7, Alternate<AF5>;
);
