#[cfg(feature = "with-dma")]
pub use crate::dma::CircReadDma as _py32f0xx_hal_dma_CircReadDma;
#[cfg(feature = "with-dma")]
pub use crate::dma::DmaExt as _py32f0xx_hal_dma_DmaExt;
#[cfg(feature = "with-dma")]
pub use crate::dma::ReadDma as _py32f0xx_hal_dma_ReadDma;
#[cfg(feature = "with-dma")]
pub use crate::dma::ReadWriteDma as _py32f0xx_hal_dma_ReadWriteDma;
#[cfg(feature = "with-dma")]
pub use crate::dma::WriteDma as _py32f0xx_hal_dma_WriteDma;
pub use crate::gpio::GpioExt as _py32f0xx_hal_gpio_GpioExt;
pub use crate::rcc::RccExt as _py32f0xx_hal_rcc_RccExt;
pub use crate::serial::SerialExt as _py32f0xx_hal_serial_SerialExt;
pub use crate::spi::SpiExt as _py32f0xx_hal_spi_SpiExt;
pub use crate::time::U32Ext as _py32f0xx_hal_time_U32Ext;
pub use crate::timer::pwm::PwmExt as _py32f0xx_hal_timer_pwm_PwmExt;
pub use crate::timer::SysTimerExt as _py32f0xx_hal_timer_SysTimerExt;
pub use crate::timer::TimerExt as _py32f0xx_hal_timer_TimerExt;
pub use fugit::ExtU32 as _fugit_ExtU32;
pub use fugit::RateExtU32 as _fugit_RateExtU32;
