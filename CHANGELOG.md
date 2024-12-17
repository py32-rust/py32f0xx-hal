# Change Log

All notable changes to this project will be documented in this file.

## Unreleased

This version depends on py32-rs v1.1.2 or later

### Added

- added module `dma`
- added feature gate to select "with-dma"
- module `gpio` has the following new types and traits:
 * Struct `PartiallyErasedPin` and `ErasedPin` replaces previous `Pin`
 * Struct `DynamicPin` allows pin to swap between an `Input` and `Output` pin during runtime at the cost of possible runtime errors
 * Trait `ExtiPin` allows connecting a pin to EXTI lines for interrupt and event handling
 * Trait `Debugger` marks pins that upon bootup are dedicated to the DBG peripheral, must be `activate` before use as a gpio
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
 *  added function `debug_stop_mode` for control of clock function in sleep and stop conditions
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

### Changed

- Changed all examples to use new api's where necessary
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
- module `time` changed to use fugit crate, this alters how you specify frequency, ie, hz -> Hz, etc.
- module `timers`
 * changed to use rcc enable, reset, and bus frequencies

### Deprecated

### Removed

- `delay` and `timers` modules removed, the functionality is in the `timer` module now
- `spi::Event::Crc` removed, as that doesn't exist on this micro

### Fixed

### Security

## v0.1.1

## V0.1.0

## v0.0.1

 - Original Release

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).
