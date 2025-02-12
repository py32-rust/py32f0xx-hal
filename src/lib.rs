//! # HAL for the py32f0xx family of microcontrollers
//!
//! This is an implementation of the [`embedded-hal`] traits for the py32f0xx family of
//! microcontrollers.
//!
//! [`embedded-hal`]: https://crates.io/crates/embedded-hal
//!
//! ## Usage
//!
//! This crate supports multiple microcontrollers in the
//! py32f0xx family. Which specific microcontroller you want to build for has to be
//! specified with a feature, for example `py32f030`.
//!
//! If no microcontroller is specified, the crate will not compile.
//!
//! The currently supported variants are
//!
//! - `py32f002a`
//! - `py32f002b`
//! - `py32f003`
//! - `py32f030`
//!
//! ## Commonly used setup
//! Almost all peripherals require references to some registers in `RCC`. The following
//! code shows how to set up those registers
//!
//! ```rust
//! // Get access to the device specific peripherals from the peripheral access crate
//! let mut dp = pac::Peripherals::take().unwrap();
//!
//! // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
//! // `clocks`
//! let clocks = dp.RCC.configure.freeze(&mut dp.FLASH);
//! ```
//!
//! ## Usage examples
//!
//! See the [examples] folder.
//!
//! Most of the examples require the following additional dependencies
//! ```toml
//! [dependencies]
//! cortex-m = { version = "0.7.7", features = ["critical-section-single-core"] }
//! cortex-m-rt = "0.7.3"
//! # Panic behaviour, see https://crates.io/keywords/panic-impl for alternatives
//! panic-halt = "0.2.0"
//! ```
//!
//! [examples]: https://github.com/py32-rust/py32f0xx-hal/tree/main/examples
//! [README]: https://github.com/py32-rust/py32f0xx-hal/tree/main
#![no_std]
#![allow(non_camel_case_types)]
#![allow(clippy::uninit_assumed_init)]
#![deny(missing_docs)]

#[cfg(feature = "py32f002a")]
pub use py32f0::py32f002a as pac;
#[cfg(feature = "py32f002b")]
pub use py32f0::py32f002b as pac;
#[cfg(feature = "py32f003")]
pub use py32f0::py32f003 as pac;
#[cfg(feature = "py32f030")]
pub use py32f0::py32f030 as pac;

#[cfg(feature = "device-selected")]
pub mod adc;
#[cfg(all(feature = "device-selected", feature = "with-dma"))]
pub mod dma;
#[cfg(feature = "device-selected")]
pub mod gpio;
#[cfg(feature = "device-selected")]
pub mod i2c;
#[cfg(feature = "device-selected")]
pub mod prelude;
#[cfg(feature = "device-selected")]
pub mod rcc;
#[cfg(any(feature = "py32f003", feature = "py32f030"))]
pub mod rtc;
#[cfg(feature = "device-selected")]
pub mod serial;
#[cfg(feature = "device-selected")]
pub mod spi;
#[cfg(feature = "device-selected")]
pub mod time;
#[cfg(feature = "device-selected")]
pub mod timer;
#[cfg(feature = "device-selected")]
pub mod watchdog;

mod sealed {
    pub trait Sealed {}
}
use sealed::Sealed;
