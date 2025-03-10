# Change Log

All notable changes to this project will be documented in this file.

## v0.3.1 - 2025-03-08

### Fixed

- Bug in gpio, _is_low(), was returning false if low, true if high, now correct

## v0.3.0 - 2025-02-14

### Added

- `timer` module trait:`WithPwm` and enum:`Channel`
- implemented `WithPwm` trait on hardware timers
- timer module `pwm` that implements pwm using new code
- implemented embedded-hal pwm traits
- added `crate::timer::pwm::PwmExt` trait to prelude

### Changed

- This version requires v0.2.1 of the [py32-rs](https://github.com/py32-rust/py32-rs.git) pac crate.
- Updated pwm examples to use new pwm code

### Removed

- Removed BUGBUG comment in gpio.rs
- Removed allow unsafe directive in rtc.rs
- Removed `pwm` separate module, reimplemented in `timer`
- Removed use of `bitflags` crate in timer.rs
- Removed `embedded-time` from Cargo.toml

### Fixed

- Checked against datasheets and corrected I2C pins in `i2c_pins` macro
- Fixed warning in rcc.rs
- Fixed array register call in monotonic.rs

## v0.2.2 - 2025-02-07

### Changed

- Modified license from BSD to dual MIT/Apache
- Added peripheral coverage in README.md

## v0.2.1 - 2025-02-04

### Fixed

- Fixed bug with gpio mode setting for output push pull vs open drain, causing hardware resets.

## v0.2.0 - 2025-01-21

This version depends on py32-rs v0.2.0 or later

### Added

- added optional feature "rtic" to implement `rtic-monotonic::Monotonic` trait on timers.
- added optional feature "defmt" to implement `defmt::Format` trait on Errors in various modules
- added feature "with-dma" for internal use to simplify dma related code
- added module `dma`
- module `gpio` has the following new types and traits:
 * Struct `PartiallyErasedPin` and `ErasedPin` replaces previous `Pin`
 * Struct `DynamicPin` allows pin to swap between an `Input` and `Output` pin during runtime at the cost of possible runtime errors
 * Trait `ExtiPin` allows connecting a pin to EXTI lines for interrupt and event handling
 * Struct `Debugger` marks pins that upon bootup are dedicated to the DBG peripheral, must be `activate` before use as a gpio
 * Trait `PinExt` adds functions to retrieve pin number and port id ('A', 'B', etc)
- gpio Pins can be temporarily mode changed to do an action, using these methods:
 * `as_push_pull_output`
 * `as_push_pull_output_with_state`
 * `as_open_drain_output`
 * `as_open_drain_output_with_state`
 * `as_floating_input`
 * `as_pull_up_input`
 * `as_pull_down_input`
- module `rcc`
 * new traits and implementations `Enable`, `Reset`, `BusClock`, `BusTimerClock`, `RccBus`
 * added function `debug_stop_mode` for control of clock function in sleep and stop conditions
- added module `rtc` and examples thereof
- module `serial`
 * Implemented embedded-hal v1.0 nb::Read/Write traits
 * Implemented embedded-hal v1.0 io::Write trait
 * Implemented `with_dma` methods, so Rx/Tx can use DMA
 * Added `SerialExt` trait and implementations for construction methods
 * Can configure stop bits, parity, word length
 * added `reconfigure`
- module `spi`
 * Implemented embedded-hal v1.0 SpiBus, nb, and blocking, traits
 * Implemented `with_dma` methods, both Rx/Tx can use DMA, separately and together
 * Added `SpiExt` trait and implementations for construction methods
 * Works with both 8 and 16 bit words, though 16bit not tested
 * Added `SpiSlave` for slave functionality, though it is not tested
 * Added frame size conversion methods, [ex: `frame_size_8_bit`]
- module `timer`
 * Old module `timers` was removed. Now follows both hal 1.0 and 0.2.7 traits

### Changed

- Fixed repo url's
- Changed all examples to use new api's where necessary, and to remove code structure that was causing much of the example to be optimized away
- module `adc` changed to use new rcc enable, reset, and bus frequencies
- module `gpio`
 * pin `into_<mode>` fns have removed the atomic context parameter which is not needed depending on what OS is used
 * implemented `split_without_reset`
 * Pin type doesn't represent an erased pin, but the most specific type of pin, see Added above
 * embedded-hal v0.2.7 traits moved to gpio/hal_02 module
- module `i2c` changed to use rcc enable, reset, and bus frequencies
- module `prelude` removed all embedded-hal traits
- module `pwm` changed to use rcc enable, reset, and bus frequencies
- module `serial`
 * embedded-hal v0.2.7 trait implementations moved to serial/hal_02 module
 * changed to use rcc enable, reset, and bus frequencies
- module `spi`
 * embedded-hal v0.2.7 trait implementations moved to spi/hal_02 module
 * changed to use rcc enable, reset, and bus frequencies
- module `time` changed to use fugit crate, this alters how you specify frequency, ie, hz -> Hz, etc

### Removed

- `delay` and `timers` modules removed, the functionality is in the `timer` module now
- `spi::Event::Crc` removed, as that doesn't exist on this micro
- `timers` module removed, replaced by `timer`

### Fixed

- Fixed github action `changelog.yml`
- Fixed github action `ci.yml`
- Fixed `tool/check.py`

## v0.1.1 - 2024-10-10

## V0.1.0 - 2024-09-07

## v0.0.1 - 2023-06-10

 - Original Release

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).
