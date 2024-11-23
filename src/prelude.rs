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
pub use crate::serial::SerialExt as _;
pub use crate::spi::SpiExt as _;
pub use crate::time::U32Ext as _py32f0xx_hal_time_U32Ext;
