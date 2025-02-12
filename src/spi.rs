#![allow(dead_code)]

//! API for the integrated SPI peripherals
//!
//! The spi bus acts as the master (generating the clock) and you need to handle the CS separately.
//!
//! The most significant bit is transmitted first & only 8-bit transfers are supported
//!
//! # Example
//! Echo incoming data in the next transfer
//! ``` no_run
//! use py32f0xx_hal as hal;
//!
//! use crate::hal::pac;
//! use crate::hal::prelude::*;
//! use crate::hal::spi::{Spi, Mode, Phase, Polarity};
//! use embedded_hal::spi::SpiBus;
//!
//! let mut p = pac::Peripherals::take().unwrap();
//! let rcc = p.RCC.constrain().freeze(&mut p.FLASH);
//!
//! let gpioa = p.GPIOA.split(&mut rcc);
//!
//! // Configure pins for SPI
//! let sck = gpioa.pa5.into_alternate_af0();
//! let miso = gpioa.pa6.into_alternate_af0();
//! let mosi = gpioa.pa7.into_alternate_af0();
//!
//! // Configure SPI with 1MHz rate
//! let mut spi = p.SPI1.spi((Some(sck), Some(miso), Some(mosi)), Mode {
//!     polarity: Polarity::IdleHigh,
//!     phase: Phase::CaptureOnSecondTransition,
//! }, 1.mhz(), &rcc.clocks);
//!
//! let mut data = [0];
//! let result = [0];
//! loop {
//!     spi.transfer(&mut data, &result).unwrap();
//!     data[0] = result[0];
//! }
//! ```

use core::marker::PhantomData;
use core::ops::{Deref, DerefMut};
use core::ptr;
#[cfg(feature = "with-dma")]
use core::sync::atomic::{self, Ordering};
#[cfg(feature = "with-dma")]
use embedded_dma::{ReadBuffer, WriteBuffer};

use crate::pac::{self, RCC};

use crate::gpio::*;

use crate::rcc::{BusClock, Clocks, Enable, Reset};

use crate::time::Hertz;

#[cfg(feature = "with-dma")]
pub mod dma;
mod hal_02;
mod hal_1;

/// Clock polarity
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Polarity {
    /// Clock signal low when idle
    IdleLow,
    /// Clock signal high when idle
    IdleHigh,
}

/// Clock phase
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    /// Data in "captured" on the first clock transition
    CaptureOnFirstTransition,
    /// Data in "captured" on the second clock transition
    CaptureOnSecondTransition,
}

/// SPI mode
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Mode {
    /// Clock polarity
    pub polarity: Polarity,
    /// Clock phase
    pub phase: Phase,
}

/// Interrupt event
pub enum Event {
    /// New data has been received
    Rxne,
    /// New data can be sent
    Txe,
    /// an error condition occurs(Overrun, ModeFault)
    Error,
}

/// The bit format to send the data in
#[derive(Debug, Clone, Copy)]
pub enum SpiBitFormat {
    /// Least significant bit first
    LsbFirst,
    /// Most significant bit first
    MsbFirst,
}

/// SPI error
#[non_exhaustive]
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// Overrun occurred
    Overrun,
    /// Mode fault occurred
    ModeFault,
}

/// Extension trait for SPI devices
pub trait SpiExt: Sized + Instance {
    /// Use SPI in master mode with 8 bit words
    fn spi<SCKPIN, MISOPIN, MOSIPIN>(
        self,
        pins: (Option<SCKPIN>, Option<MISOPIN>, Option<MOSIPIN>),
        mode: Mode,
        freq: Hertz,
        clocks: &Clocks,
    ) -> Spi<Self, SCKPIN, MISOPIN, MOSIPIN, u8>
    where
        SCKPIN: SckPin<Self>,
        MISOPIN: MisoPin<Self>,
        MOSIPIN: MosiPin<Self>;
    /// Use SPI in master mode with 16 bit words
    fn spi_u16<SCKPIN, MISOPIN, MOSIPIN>(
        self,
        pins: (Option<SCKPIN>, Option<MISOPIN>, Option<MOSIPIN>),
        mode: Mode,
        freq: Hertz,
        clocks: &Clocks,
    ) -> Spi<Self, SCKPIN, MISOPIN, MOSIPIN, u16>
    where
        SCKPIN: SckPin<Self>,
        MISOPIN: MisoPin<Self>,
        MOSIPIN: MosiPin<Self>,
    {
        Self::spi(self, pins, mode, freq, clocks).frame_size_16bit()
    }
    /// Use SPI in slave mode with 8 bit words
    fn spi_slave<SCKPIN, MISOPIN, MOSIPIN>(
        self,
        pins: (Option<SCKPIN>, Option<MISOPIN>, Option<MOSIPIN>),
        mode: Mode,
    ) -> SpiSlave<Self, SCKPIN, MISOPIN, MOSIPIN, u8>
    where
        SCKPIN: SckPin<Self>,
        MISOPIN: MisoPin<Self>,
        MOSIPIN: MosiPin<Self>;
    /// Use SPI in slave mode with 16 bit words
    fn spi_slave_u16<SCKPIN, MISOPIN, MOSIPIN>(
        self,
        pins: (Option<SCKPIN>, Option<MISOPIN>, Option<MOSIPIN>),
        mode: Mode,
    ) -> SpiSlave<Self, SCKPIN, MISOPIN, MOSIPIN, u16>
    where
        SCKPIN: SckPin<Self>,
        MISOPIN: MisoPin<Self>,
        MOSIPIN: MosiPin<Self>,
    {
        Self::spi_slave(self, pins, mode).frame_size_16bit()
    }
}

impl<SPI: Instance> SpiExt for SPI {
    fn spi<SCKPIN, MISOPIN, MOSIPIN>(
        self,
        pins: (Option<SCKPIN>, Option<MISOPIN>, Option<MOSIPIN>),
        mode: Mode,
        freq: Hertz,
        clocks: &Clocks,
    ) -> Spi<Self, SCKPIN, MISOPIN, MOSIPIN, u8>
    where
        SCKPIN: SckPin<SPI>,
        MISOPIN: MisoPin<SPI>,
        MOSIPIN: MosiPin<SPI>,
    {
        Spi::new(self, pins, mode, freq, clocks)
    }
    fn spi_slave<SCKPIN, MISOPIN, MOSIPIN>(
        self,
        pins: (Option<SCKPIN>, Option<MISOPIN>, Option<MOSIPIN>),
        mode: Mode,
    ) -> SpiSlave<Self, SCKPIN, MISOPIN, MOSIPIN, u8>
    where
        SCKPIN: SckPin<SPI>,
        MISOPIN: MisoPin<SPI>,
        MOSIPIN: MosiPin<SPI>,
    {
        SpiSlave::new(self, pins, mode)
    }
}

/// SPI with a frame size, common to master and slave modes
pub struct SpiInner<SPI, W> {
    spi: SPI,
    _framesize: PhantomData<W>,
}

impl<SPI, W> SpiInner<SPI, W> {
    fn new(spi: SPI) -> Self {
        Self {
            spi,
            _framesize: PhantomData,
        }
    }
}

/// Spi in Master mode
pub struct Spi<SPI: Instance, SCKPIN, MISOPIN, MOSIPIN, W> {
    inner: SpiInner<SPI, W>,
    #[allow(clippy::type_complexity)]
    pins: (Option<SCKPIN>, Option<MISOPIN>, Option<MOSIPIN>),
}

/// Spi in Slave mode
pub struct SpiSlave<SPI: Instance, SCKPIN, MISOPIN, MOSIPIN, W> {
    inner: SpiInner<SPI, W>,
    #[allow(clippy::type_complexity)]
    pins: (Option<SCKPIN>, Option<MISOPIN>, Option<MOSIPIN>),
}

impl<SPI: Instance, SCKPIN, MISOPIN, MOSIPIN, W> Deref for Spi<SPI, SCKPIN, MISOPIN, MOSIPIN, W> {
    type Target = SpiInner<SPI, W>;
    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<SPI: Instance, SCKPIN, MISOPIN, MOSIPIN, W> DerefMut
    for Spi<SPI, SCKPIN, MISOPIN, MOSIPIN, W>
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

impl<SPI: Instance, SCKPIN, MISOPIN, MOSIPIN, W> Deref
    for SpiSlave<SPI, SCKPIN, MISOPIN, MOSIPIN, W>
{
    /// The SPI hardware with frame size
    type Target = SpiInner<SPI, W>;
    fn deref(&self) -> &Self::Target {
        &self.inner
    }
}

impl<SPI: Instance, SCKPIN, MISOPIN, MOSIPIN, W> DerefMut
    for SpiSlave<SPI, SCKPIN, MISOPIN, MOSIPIN, W>
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.inner
    }
}

/// Instance of a specific SPI peripheral
pub trait Instance:
    crate::Sealed + Deref<Target = crate::pac::spi1::RegisterBlock> + Enable + Reset + BusClock
{
}

impl Instance for pac::SPI1 {}
#[cfg(feature = "py32f030")]
impl Instance for pac::SPI2 {}

/// trait for SPI Sck pins
pub trait SckPin<SPI> {}
/// trait for SPI MISO pins
pub trait MisoPin<SPI> {}
/// trait for SPI MOSI pins
pub trait MosiPin<SPI> {}

macro_rules! spi_pins {
    ($($SPI:ident => {
        sck => [$($sck:ty),+ $(,)*],
        miso => [$($miso:ty),+ $(,)*],
        mosi => [$($mosi:ty),+ $(,)*],
    })+) => {
        $(
            $(
                impl SckPin<crate::pac::$SPI> for $sck {}
            )+
            $(
                impl MisoPin<crate::pac::$SPI> for $miso {}
            )+
            $(
                impl MosiPin<crate::pac::$SPI> for $mosi {}
            )+
        )+
    }
}

#[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002a"))]
spi_pins! {
    SPI1 => {
        sck =>  [
            gpioa::PA1<Alternate<AF0>>,
            gpioa::PA2<Alternate<AF10>>,
            gpioa::PA5<Alternate<AF0>>,
            gpiob::PB3<Alternate<AF0>>,
        ],
        miso => [
            gpioa::PA0<Alternate<AF10>>,
            gpioa::PA3<Alternate<AF0>>,
            gpioa::PA6<Alternate<AF0>>,
            gpioa::PA7<Alternate<AF10>>,
            gpioa::PA13<Alternate<AF10>>,
            gpiob::PB4<Alternate<AF0>>,
        ],
        mosi => [
            gpioa::PA1<Alternate<AF10>>,
            gpioa::PA2<Alternate<AF0>>,
            gpioa::PA3<Alternate<AF10>>,
            gpioa::PA7<Alternate<AF0>>,
            gpioa::PA12<Alternate<AF0>>,
            gpiob::PB5<Alternate<AF0>>,
        ],
    }
}

#[cfg(feature = "py32f030")]
spi_pins! {
    SPI1 => {
        sck =>  [
            gpioa::PA9<Alternate<AF10>>,
        ],
        miso => [
            gpioa::PA11<Alternate<AF0>>,
        ],
        mosi => [
            gpioa::PA8<Alternate<AF10>>,
        ],
    }
    SPI2 => {
        sck =>  [
            gpioa::PA1<Alternate<AF0>>,
            gpiob::PB2<Alternate<AF1>>,
            gpiob::PB8<Alternate<AF1>>,
            gpiof::PF0<Alternate<AF3>>,
        ],
        miso => [
            gpioa::PA3<Alternate<AF0>>,
            gpioa::PA9<Alternate<AF0>>,
            gpiob::PB6<Alternate<AF3>>,
            gpiof::PF1<Alternate<AF3>>,
            gpiof::PF3<Alternate<AF3>>,
        ],
        mosi => [
            gpioa::PA4<Alternate<AF2>>,
            gpioa::PA10<Alternate<AF0>>,
            gpiob::PB7<Alternate<AF1>>,
            gpiof::PF2<Alternate<AF3>>,
        ],
    }
}

#[cfg(feature = "py32f002b")]
spi_pins! {
    SPI1 => {
        sck =>  [
            gpiob::PB0<Alternate<AF0>>,
            gpiob::PB2<Alternate<AF0>>,
        ],
        miso => [
            gpioa::PA1<Alternate<AF0>>,
            gpiob::PB6<Alternate<AF2>>,
            gpioc::PC1<Alternate<AF0>>,
        ],
        mosi => [
            gpioa::PA0<Alternate<AF0>>,
            gpioa::PA7<Alternate<AF0>>,
            gpiob::PB7<Alternate<AF0>>,
        ],
    }
}

impl<SPI: Instance, SCKPIN, MISOPIN, MOSIPIN, WIDTH: Copy>
    Spi<SPI, SCKPIN, MISOPIN, MOSIPIN, WIDTH>
{
    /// Creates a new spi instance
    pub fn new<F>(
        spi: SPI,
        pins: (Option<SCKPIN>, Option<MISOPIN>, Option<MOSIPIN>),
        mode: Mode,
        speed: F,
        clocks: &Clocks,
    ) -> Self
    where
        SCKPIN: SckPin<SPI>,
        MISOPIN: MisoPin<SPI>,
        MOSIPIN: MosiPin<SPI>,
        F: Into<Hertz>,
    {
        Self::_new(spi, pins, mode, speed, clocks)
    }

    fn _new<F: Into<Hertz>>(
        spi: SPI,
        pins: (Option<SCKPIN>, Option<MISOPIN>, Option<MOSIPIN>),
        mode: Mode,
        speed: F,
        clocks: &Clocks,
    ) -> Self {
        /* Enable clock for SPI */
        let rcc = unsafe { &(*RCC::ptr()) };
        SPI::enable(rcc);
        /* Reset SPI */
        SPI::reset(rcc);

        /* Make sure the SPI unit is disabled so we can configure it */
        spi.cr1.modify(|_, w| w.spe().clear_bit());

        let br = match clocks.pclk().raw() / speed.into().raw() {
            0 => unreachable!(),
            1..=2 => 0b000,
            3..=5 => 0b001,
            6..=11 => 0b010,
            12..=23 => 0b011,
            24..=47 => 0b100,
            48..=95 => 0b101,
            96..=191 => 0b110,
            _ => 0b111,
        };

        // mstr: master configuration
        // lsbfirst: MSB first
        // ssm: enable software slave management (NSS pin free for other uses)
        // ssi: set nss high = master mode
        // bidimode: 2-line unidirectional
        // spe: enable the SPI bus
        spi.cr1.write(|w| {
            w.cpha().bit(mode.phase == Phase::CaptureOnSecondTransition);
            w.cpol().bit(mode.polarity == Polarity::IdleHigh);
            w.mstr().set_bit();
            w.br().bits(br);
            w.lsbfirst().clear_bit();
            w.ssm().set_bit();
            w.ssi().set_bit();
            w.rxonly().clear_bit();
            w.bidimode().clear_bit();
            w.spe().set_bit()
        });
        // fifo reception threshold set to 1/4
        #[cfg(not(feature = "py32f002b"))]
        spi.cr2.modify(|_, w| w.frxth().quarter());

        Spi::<SPI, SCKPIN, MISOPIN, MOSIPIN, WIDTH> {
            inner: SpiInner::new(spi),
            pins,
        }
    }
    /// Release the SPI instance and any pins used
    pub fn release(self) -> (SPI, (Option<SCKPIN>, Option<MISOPIN>, Option<MOSIPIN>)) {
        (self.inner.spi, self.pins)
    }
}

impl<SPI: Instance, SCKPIN, MISOPIN, MOSIPIN, WIDTH: Copy>
    SpiSlave<SPI, SCKPIN, MISOPIN, MOSIPIN, WIDTH>
{
    /// Creates a new spi slave instance
    pub fn new(
        spi: SPI,
        pins: (Option<SCKPIN>, Option<MISOPIN>, Option<MOSIPIN>),
        mode: Mode,
    ) -> Self
    where
        SCKPIN: SckPin<SPI>,
        MISOPIN: MisoPin<SPI>,
        MOSIPIN: MosiPin<SPI>,
    {
        Self::_new(spi, pins, mode)
    }

    fn _new(
        spi: SPI,
        pins: (Option<SCKPIN>, Option<MISOPIN>, Option<MOSIPIN>),
        mode: Mode,
    ) -> Self {
        // enable and reset SPI
        let rcc = unsafe { &(*RCC::ptr()) };
        SPI::enable(rcc);
        SPI::reset(rcc);

        // disable SS output
        spi.cr2.write(|w| w.ssoe().clear_bit());

        spi.cr1.write(|w| {
            // clock phase from config
            w.cpha().bit(mode.phase == Phase::CaptureOnSecondTransition);
            // clock polarity from config
            w.cpol().bit(mode.polarity == Polarity::IdleHigh);
            // mstr: slave configuration
            w.mstr().clear_bit();
            // lsbfirst: MSB first
            w.lsbfirst().clear_bit();
            // ssm: enable software slave management (NSS pin free for other uses)
            w.ssm().set_bit();
            // ssi: set nss low = slave mode
            w.ssi().clear_bit();
            // bidimode: 2-line unidirectional
            w.bidimode().clear_bit();
            // both TX and RX are used
            w.rxonly().clear_bit();
            // spe: enable the SPI bus
            w.spe().set_bit()
        });

        SpiSlave::<SPI, SCKPIN, MISOPIN, MOSIPIN, WIDTH> {
            inner: SpiInner::new(spi),
            pins,
        }
    }
    /// Release the SPI instance with any pins
    pub fn release(self) -> (SPI, (Option<SCKPIN>, Option<MISOPIN>, Option<MOSIPIN>)) {
        (self.inner.spi, self.pins)
    }
}

impl<SPI: Instance, W: Copy> SpiInner<SPI, W> {
    /// Select which frame format is used for data transfers
    pub fn bit_format(&mut self, format: SpiBitFormat) {
        self.spi
            .cr1
            .modify(|_, w| w.lsbfirst().bit(matches!(format, SpiBitFormat::LsbFirst)));
    }

    /// Starts listening to the SPI by enabling the _Received data
    /// ready to be read (RXNE)_ interrupt and _Transmit data
    /// register empty (TXE)_ interrupt
    pub fn listen(&mut self, event: Event) {
        self.spi.cr2.modify(|_, w| match event {
            Event::Rxne => w.rxneie().set_bit(),
            Event::Txe => w.txeie().set_bit(),
            Event::Error => w.errie().set_bit(),
        });
    }

    /// Stops listening to the SPI by disabling the _Received data
    /// ready to be read (RXNE)_ interrupt and _Transmit data
    /// register empty (TXE)_ interrupt
    pub fn unlisten(&mut self, event: Event) {
        self.spi.cr2.modify(|_, w| match event {
            Event::Rxne => w.rxneie().clear_bit(),
            Event::Txe => w.txeie().clear_bit(),
            Event::Error => w.errie().clear_bit(),
        });
    }

    /// Returns true if the tx register is empty (and can accept data)
    #[inline]
    pub fn is_tx_empty(&self) -> bool {
        self.spi.sr.read().txe().bit_is_set()
    }

    /// Returns true if the rx register is not empty (and can be read)
    #[inline]
    pub fn is_rx_not_empty(&self) -> bool {
        self.spi.sr.read().rxne().bit_is_set()
    }

    /// Returns true if the transfer is in progress
    #[inline]
    pub fn is_busy(&self) -> bool {
        self.spi.sr.read().bsy().bit_is_set()
    }

    /// Returns true if data are received and the previous data have not yet been read from SPI_DR.
    #[inline]
    pub fn is_overrun(&self) -> bool {
        self.spi.sr.read().ovr().bit_is_set()
    }
}

impl<SPI: Instance, SCKPIN, MISOPIN, MOSIPIN> Spi<SPI, SCKPIN, MISOPIN, MOSIPIN, u8> {
    /// Converts from 8bit dataframe to 16bit.
    pub fn frame_size_16bit(self) -> Spi<SPI, SCKPIN, MISOPIN, MOSIPIN, u16> {
        self.spi.cr1.modify(|_, w| w.spe().clear_bit());
        self.spi.cr2.modify(|_, w| {
            w.ds().set_bit();
            #[cfg(not(feature = "py32f002b"))]
            w.frxth().half();
            w
        });
        self.spi.cr1.modify(|_, w| w.spe().set_bit());
        Spi {
            inner: SpiInner::new(self.inner.spi),
            pins: self.pins,
        }
    }
}

impl<SPI: Instance, SCKPIN, MISOPIN, MOSIPIN> SpiSlave<SPI, SCKPIN, MISOPIN, MOSIPIN, u8> {
    /// Converts from 8bit dataframe to 16bit.
    pub fn frame_size_16bit(self) -> SpiSlave<SPI, SCKPIN, MISOPIN, MOSIPIN, u16> {
        self.spi.cr1.modify(|_, w| w.spe().clear_bit());
        self.spi.cr2.modify(|_, w| {
            w.ds().set_bit();
            #[cfg(not(feature = "py32f002b"))]
            w.frxth().half();
            w
        });
        self.spi.cr1.modify(|_, w| w.spe().set_bit());
        SpiSlave {
            inner: SpiInner::new(self.inner.spi),
            pins: self.pins,
        }
    }
}

impl<SPI: Instance, SCKPIN, MISOPIN, MOSIPIN> Spi<SPI, SCKPIN, MISOPIN, MOSIPIN, u16> {
    /// Converts from 16bit dataframe to 8bit.
    pub fn frame_size_8bit(self) -> Spi<SPI, SCKPIN, MISOPIN, MOSIPIN, u16> {
        self.spi.cr1.modify(|_, w| w.spe().clear_bit());
        self.spi.cr2.modify(|_, w| {
            w.ds().clear_bit();
            #[cfg(not(feature = "py32f002b"))]
            w.frxth().quarter();
            w
        });
        self.spi.cr1.modify(|_, w| w.spe().set_bit());
        Spi {
            inner: SpiInner::new(self.inner.spi),
            pins: self.pins,
        }
    }
}

impl<SPI: Instance, SCKPIN, MISOPIN, MOSIPIN> SpiSlave<SPI, SCKPIN, MISOPIN, MOSIPIN, u16> {
    /// Converts from 16bit dataframe to 8bit.
    pub fn frame_size_8bit(self) -> SpiSlave<SPI, SCKPIN, MISOPIN, MOSIPIN, u8> {
        self.spi.cr1.modify(|_, w| w.spe().clear_bit());
        self.spi.cr2.modify(|_, w| {
            w.ds().clear_bit();
            #[cfg(not(feature = "py32f002b"))]
            w.frxth().quarter();
            w
        });
        self.spi.cr1.modify(|_, w| w.spe().set_bit());
        SpiSlave {
            inner: SpiInner::new(self.inner.spi),
            pins: self.pins,
        }
    }
}

/// Trait for SPI instance that can read and write
pub trait SpiReadWrite<T> {
    /// Read the SPI data register
    fn read_data_reg(&mut self) -> T;
    /// Write the SPI data register
    fn write_data_reg(&mut self, data: T);
    /// Write a slice to the SPI data register, blocking
    fn spi_write(&mut self, words: &[T]) -> Result<(), Error>;
}

impl<SPI: Instance, W: Copy> SpiReadWrite<W> for SpiInner<SPI, W> {
    fn read_data_reg(&mut self) -> W {
        // NOTE(read_volatile) read only 1 byte (the svd2rust API only allows
        // reading a half-word)
        unsafe { ptr::read_volatile(self.spi.dr().as_ptr() as *const W) }
    }

    fn write_data_reg(&mut self, data: W) {
        // NOTE(write_volatile) see note above
        unsafe { ptr::write_volatile(self.spi.dr().as_ptr() as *mut W, data) }
    }

    // Implement write as per the "Transmit only procedure" page 712
    // of RM0008 Rev 20. This is more than twice as fast as the
    // default Write<> implementation (which reads and drops each
    // received value)
    fn spi_write(&mut self, words: &[W]) -> Result<(), Error> {
        // Write each word when the tx buffer is empty
        for word in words {
            loop {
                let sr = self.spi.sr.read();
                if sr.txe().bit_is_set() {
                    self.write_data_reg(*word);
                    if sr.modf().bit_is_set() {
                        return Err(Error::ModeFault);
                    }
                    break;
                }
            }
        }
        // Wait for final TXE
        while !self.is_tx_empty() {}
        // Wait for final !BSY
        while self.is_busy() {}
        // Clear OVR set due to dropped received values
        let _ = self.read_data_reg();
        let _ = self.spi.sr.read();
        Ok(())
    }
}

impl<SPI, W> SpiInner<SPI, W>
where
    SPI: Instance,
    W: Copy,
{
    /// Read a word from the SPI device, non-blocking
    pub fn read_nonblocking(&mut self) -> nb::Result<W, Error> {
        let sr = self.spi.sr.read();

        Err(if sr.ovr().bit_is_set() {
            Error::Overrun.into()
        } else if sr.modf().bit_is_set() {
            Error::ModeFault.into()
        } else if sr.rxne().bit_is_set() {
            // NOTE(read_volatile) read only 1 byte (the svd2rust API only allows
            // reading a half-word)
            return Ok(self.read_data_reg());
        } else {
            nb::Error::WouldBlock
        })
    }
    /// Write a word to the SPI device, non-blocking
    pub fn write_nonblocking(&mut self, data: W) -> nb::Result<(), Error> {
        let sr = self.spi.sr.read();

        // NOTE: Error::Overrun was deleted in #408. Need check
        Err(if sr.modf().bit_is_set() {
            Error::ModeFault.into()
        } else if sr.txe().bit_is_set() {
            // NOTE(write_volatile) see note above
            self.write_data_reg(data);
            return Ok(());
        } else {
            nb::Error::WouldBlock
        })
    }
    /// Write a slice of words to the SPI device, blocking
    pub fn write(&mut self, words: &[W]) -> Result<(), Error> {
        self.spi_write(words)
    }
}
