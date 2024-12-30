use super::*;

macro_rules! bus {
    ($($PER:ident => ($apbX:ty, $enr:tt, $rstr:tt, $bit:literal),)+) => {
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
    }
}

bus! {
    ADC => (APB, apbenr2, apbrstr2, 20),
    CRC => (AHB, ahbenr, ahbrstr, 12),
    DBG => (APB, apbenr1, apbrstr1, 27),
    GPIOA => (APB, iopenr, ioprstr, 0),
    GPIOB => (APB, iopenr, ioprstr, 1),
    I2C => (APB, apbenr1, apbrstr1, 21),
    PWR => (APB, apbenr1, apbrstr1, 28),
    SPI1 => (APB, apbenr2, apbrstr2, 12),
    SYSCFG => (APB, apbenr2, apbrstr2, 0),
    TIM1 => (APB, apbenr2, apbrstr2, 11),
    USART1 => (APB, apbenr2, apbrstr2, 14),
}

#[cfg(any(feature = "py32f003", feature = "py32f030"))]
bus! {
    USART2 => (APB, apbenr1, apbrstr1, 17),
    DMA => (AHB, ahbenr, ahbrstr, 0),
    WWDG => (APB, apbenr1, apbrstr1, 11),
}

#[cfg(any(feature = "py32f030"))]
bus! {
    SPI2 => (APB, apbenr1, apbrstr1, 14),
}

#[cfg(any(feature = "py32f002a", feature = "py32f003", feature = "py32f030"))]
bus! {
    GPIOF => (APB, iopenr, ioprstr, 5),
    LPTIM => (APB, apbenr1, apbrstr1, 31),
}

#[cfg(any(feature = "py32f002b"))]
bus! {
    GPIOC => (APB, iopenr, ioprstr, 2),
    LPTIM1 => (APB, apbenr1, apbrstr1, 31),
}

#[cfg(any(feature = "py32f030"))]
bus! {
    LED => (APB, apbenr2, apbrstr2, 23),
}

#[cfg(any(feature = "py32f003", feature = "py32f030"))]
bus! {
    TIM3 => (APB, apbenr1, apbrstr1, 1),
    TIM17 => (APB, apbenr2, apbrstr2, 18),
}

#[cfg(any(feature = "py32f002b", feature = "py32f003", feature = "py32f030"))]
bus! {
    TIM14 => (APB, apbenr2, apbrstr2, 15),
}

#[cfg(any(feature = "py32f002a", feature = "py32f003", feature = "py32f030"))]
bus! {
    TIM16 => (APB, apbenr2, apbrstr2, 17),
}
