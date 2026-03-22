use super::*;

macro_rules! bus {
    ($($PER:ident => ($apbX:ty, $enr:tt, $bit:literal),)+) => {
        $(
            impl crate::Sealed for crate::pac::$PER {}

            impl RccBus for crate::pac::$PER {
                type Bus = $apbX;
            }
            impl Enable for crate::pac::$PER {
                #[inline(always)]
                fn enable(rcc: &rcc::RegisterBlock) {
                    unsafe {
                        rcc.$enr.modify(|r,w| w.bits(r.bits() | (1 << $bit)));
                    }
                }
                #[inline(always)]
                fn disable(rcc: &rcc::RegisterBlock) {
                    unsafe {
                        rcc.$enr.modify(|r,w| w.bits(r.bits() & !(1 << $bit)));
                    }
                }
            }
        )+
    };
    ($($PER:ident => ($apbX:ty, $enr:tt, $bit:literal, $rstr:tt),)+) => {
        $(
            bus!($PER => ($apbX, $enr, $bit),);
            impl Reset for crate::pac::$PER {
                #[inline(always)]
                fn reset(rcc: &rcc::RegisterBlock) {
                    unsafe {
                        rcc.$rstr.modify(|r,w| w.bits(r.bits() | (1 << $bit)));
                        rcc.$rstr.modify(|r,w| w.bits(r.bits() & !(1 << $bit)));
                    }
                }
            }
        )+
    };
}

bus! {
    RTC => (APB, apbenr1, 10),
}

bus! {
    ADC => (APB, apbenr2, 20, apbrstr2),
    CRC => (AHB, ahbenr, 12, ahbrstr),
    DBG => (APB, apbenr1, 27, apbrstr1),
    GPIOA => (APB, iopenr, 0, ioprstr),
    GPIOB => (APB, iopenr, 1, ioprstr),
    I2C => (APB, apbenr1, 21, apbrstr1),
    PWR => (APB, apbenr1, 28, apbrstr1),
    SPI1 => (APB, apbenr2, 12, apbrstr2),
    SYSCFG => (APB, apbenr2, 0, apbrstr2),
    TIM1 => (APB, apbenr2, 11, apbrstr2),
    USART1 => (APB, apbenr2, 14, apbrstr2),
}

#[cfg(any(feature = "py32f003", feature = "py32f030"))]
bus! {
    USART2 => (APB, apbenr1, 17, apbrstr1),
    DMA => (AHB, ahbenr, 0, ahbrstr),
    WWDG => (APB, apbenr1, 11, apbrstr1),
}

#[cfg(any(feature = "py32f030"))]
bus! {
    SPI2 => (APB, apbenr1, 14, apbrstr1),
}

#[cfg(any(feature = "py32f002a", feature = "py32f003", feature = "py32f030"))]
bus! {
    GPIOF => (APB, iopenr, 5, ioprstr),
    LPTIM => (APB, apbenr1, 31, apbrstr1),
}

#[cfg(any(feature = "py32f002b"))]
bus! {
    GPIOC => (APB, iopenr, 2, ioprstr),
    LPTIM1 => (APB, apbenr1, 31, apbrstr1),
}

#[cfg(any(feature = "py32f030"))]
bus! {
    LED => (APB, apbenr2, 23, apbrstr2),
}

#[cfg(any(feature = "py32f003", feature = "py32f030"))]
bus! {
    TIM3 => (APB, apbenr1, 1, apbrstr1),
    TIM17 => (APB, apbenr2, 18, apbrstr2),
}

#[cfg(any(feature = "py32f002b", feature = "py32f003", feature = "py32f030"))]
bus! {
    TIM14 => (APB, apbenr2, 15, apbrstr2),
}

#[cfg(any(feature = "py32f002a", feature = "py32f003", feature = "py32f030"))]
bus! {
    TIM16 => (APB, apbenr2, 17, apbrstr2),
}
