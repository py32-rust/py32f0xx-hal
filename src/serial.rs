//! API for the integrated USART ports
//!
//! This only implements the usual asynchronous bidirectional 8-bit transfers.
//!
//! It's possible to use a read-only/write-only serial implementation with
//! `rx`/`tx`.
//!
//! # Examples
//! Echo
//! ``` no_run
//! use py32f0xx_hal as hal;
//!
//! use crate::hal::prelude::*;
//! use crate::hal::serial::Serial;
//! use crate::hal::pac;
//!
//! use nb::block;
//!
//! let rcc = p.RCC.configure().sysclk(24.mhz()).freeze();
//!
//! let gpiob = p.GPIOB.split(&mut rcc);
//!
//! // USART1 on Pins B6 and B7
//! let tx = gpiob.pb6.into_alternate_af0();
//! let rx = gpiob.pb7.into_alternate_af0();
//!
//! // Create an interface struct for USART1 with 115200 Baud
//! let mut serial = p.USART1.serial((tx, rx), 115_200.bps(), &rcc.clocks);
//!
//! loop {
//!     let received = block!(serial.read()).unwrap();
//!     block!(serial.write_u8(received)).ok();
//! }
//! ```
//!
//! Hello World
//! ``` no_run
//! use py32f0xx_hal as hal;
//!
//! use crate::hal::prelude::*;
//! use crate::hal::serial::Serial;
//! use crate::hal::pac;
//!
//! use nb::block;
//!
//! let rcc = p.RCC.configure().sysclk(24.mhz()).freeze();
//!
//! let gpiob = p.GPIOB.split(&mut rcc);
//!
//! let tx = gpiob.pb6.into_alternate_af0();
//!
//! let mut serial = p.USART1.tx(tx, 115_200.bps(), &rcc.clocks);
//!
//! loop {
//!     serial.write_str("Hello World!\r\n");
//! }
//! ```

use core::marker::PhantomData;
use core::ops::Deref;

#[cfg(feature = "with-dma")]
use crate::dma::{
    self, Ch, CircBuffer, DmaExt, PeriphMap, Receive, RxDma, Transfer, TransferPayload, Transmit,
    TxDma,
};
use crate::gpio::{gpioa::*, gpiob::*};
use crate::gpio::{Alternate, AF1};
use crate::pac::{self, RCC};
use crate::rcc::{BusClock, Clocks, Enable, Reset};
use crate::time::{Bps, U32Ext};
#[cfg(feature = "with-dma")]
use core::sync::atomic::{self, Ordering};
#[cfg(feature = "with-dma")]
use embedded_dma::{ReadBuffer, WriteBuffer};

#[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002a"))]
use crate::gpio::{gpiof::*, AF0, AF8};

#[cfg(any(feature = "py32f030", feature = "py32f003"))]
use crate::gpio::{AF3, AF4, AF9};

#[cfg(feature = "py32f002b")]
use crate::gpio::AF3;

mod hal_02;
mod hal_1;

pub trait TxPin<USART> {}
pub trait RxPin<USART> {}

/// Macro to implement `TxPin` / `RxPin` for a certain pin, using a certain
/// alternative function and for a certain serial peripheral.
macro_rules! impl_pins {
    ($($pin:ident, $af:ident, $instance:ident, $trait:ident;)*) => {
        $(
            impl $trait<crate::pac::$instance> for $pin<Alternate<$af>> {}
        )*
    }
}

#[cfg(any(feature = "py32f030", feature = "py32f003"))]
impl_pins!(
    PA0, AF9, USART2, TxPin;
    PA1, AF9, USART2, RxPin;
    PA2, AF1, USART1, TxPin;
    PA2, AF4, USART2, TxPin;
    PA3, AF1, USART1, RxPin;
    PA3, AF4, USART2, RxPin;
    PA4, AF9, USART2, TxPin;
    PA5, AF9, USART2, RxPin;
    PA7, AF8, USART1, TxPin;
    PA7, AF9, USART2, TxPin;
    PA13, AF8, USART1, RxPin;
    PA14, AF1, USART1, TxPin;
    PA14, AF4, USART2, TxPin;

    PB2, AF0, USART1, RxPin;
    PB2, AF3, USART2, RxPin;
    PB6, AF0, USART1, TxPin;
    PB6, AF4, USART2, TxPin;
    PB7, AF0, USART1, RxPin;
    PB7, AF4, USART2, RxPin;

    PF0, AF4, USART2, RxPin;
    PF0, AF8, USART1, RxPin;
    PF0, AF9, USART2, TxPin;
    PF1, AF4, USART2, TxPin;
    PF1, AF8, USART1, TxPin;
    PF1, AF9, USART2, RxPin;
    PF2, AF4, USART2, RxPin;
    PF3, AF0, USART1, TxPin;
    PF3, AF4, USART2, TxPin;
);

#[cfg(feature = "py32f030")]
impl_pins!(
    PA8, AF8, USART1, RxPin;
    PA8, AF9, USART2, RxPin;
    PA9, AF1, USART1, TxPin;
    PA9, AF4, USART2, TxPin;
    PA9, AF8, USART1, RxPin;
    PA10, AF1, USART1, RxPin;
    PA10, AF4, USART2, RxPin;
    PA10, AF8, USART1, TxPin;
    PA15, AF1, USART1, RxPin;
    PA15, AF4, USART2, RxPin;

    PB8, AF4, USART2, TxPin;
    PB8, AF8, USART1, TxPin;
);

#[cfg(feature = "py32f002a")]
impl_pins!(
    PA2, AF1, USART1, TxPin;
    PA3, AF1, USART1, RxPin;

    PA7, AF8, USART1, TxPin;
    PA8, AF8, USART1, RxPin;

    PA9, AF1, USART1, TxPin;
    PA9, AF8, USART1, RxPin;

    PA10, AF1, USART1, RxPin;
    PA10, AF8, USART1, TxPin;

    PA13, AF8, USART1, RxPin;
    PA14, AF1, USART1, TxPin;

    PB2, AF0, USART1, RxPin;
    PB6, AF0, USART1, TxPin;

    PF0, AF8, USART1, RxPin;
    PF1, AF8, USART1, TxPin;
);

#[cfg(feature = "py32f002b")]
impl_pins!(
    PA2, AF1, USART1, RxPin;
    PA3, AF1, USART1, TxPin;
    PA4, AF1, USART1, RxPin;
    PA6, AF1, USART1, TxPin;
    PA7, AF1, USART1, TxPin;
    PA7, AF3, USART1, RxPin;

    PB4, AF1, USART1, TxPin;
    PB5, AF1, USART1, RxPin;
    PB6, AF1, USART1, TxPin;
);

pub trait SerialExt: Sized + Instance {
    fn serial<TXPIN, RXPIN>(
        self,
        pins: (TXPIN, RXPIN),
        config: impl Into<Config>,
        clocks: &Clocks,
    ) -> Serial<Self, TXPIN, RXPIN>;
    fn tx<TXPIN>(self, tx_pin: TXPIN, config: impl Into<Config>, clocks: &Clocks) -> Tx<Self>;
    fn rx<RXPIN>(self, rx_pin: RXPIN, config: impl Into<Config>, clocks: &Clocks) -> Rx<Self>;
}

impl<USART: Instance> SerialExt for USART {
    fn serial<TXPIN, RXPIN>(
        self,
        pins: (TXPIN, RXPIN),
        config: impl Into<Config>,
        clocks: &Clocks,
    ) -> Serial<Self, TXPIN, RXPIN> {
        Serial::new(self, pins, config, clocks)
    }
    fn tx<TXPIN>(self, tx_pin: TXPIN, config: impl Into<Config>, clocks: &Clocks) -> Tx<Self> {
        Serial::_new(self, (tx_pin, ()), config, clocks).split().0
    }
    fn rx<RXPIN>(self, rx_pin: RXPIN, config: impl Into<Config>, clocks: &Clocks) -> Rx<Self> {
        Serial::_new(self, ((), rx_pin), config, clocks).split().1
    }
}

use crate::pac::usart1 as uart_base;

pub trait Instance:
    crate::Sealed + Deref<Target = uart_base::RegisterBlock> + Enable + Reset + BusClock
{
    #[doc(hidden)]
    fn ptr() -> *const uart_base::RegisterBlock;
}

macro_rules! inst {
    ($($USARTX:ty;)+) => {
        $(
            impl Instance for $USARTX {
                fn ptr() -> *const uart_base::RegisterBlock {
                    <$USARTX>::ptr()
                }
            }
        )+
    };
}

#[cfg(any(
    feature = "py32f002a",
    feature = "py32f002b",
    feature = "py32f003",
    feature = "py32f030",
))]
inst! {
    pac::USART1;
}

#[cfg(any(feature = "py32f003", feature = "py32f030",))]
inst! {
    pac::USART2;
}

/// Serial error
#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    /// The peripheral receive buffer was overrun.
    Overrun,
    /// Received data does not conform to the peripheral configuration.
    /// Can be caused by a misconfigured device on either end of the serial line.
    FrameFormat,
    /// Parity check failed.
    Parity,
    /// Serial line is too noisy to read valid data.
    Noise,
    /// A different error occurred. The original error may contain more information.
    Other,
}

pub enum WordLength {
    /// When parity is enabled, a word has 7 data bits + 1 parity bit,
    /// otherwise 8 data bits.
    Bits8,
    /// When parity is enabled, a word has 8 data bits + 1 parity bit,
    /// otherwise 9 data bits.
    Bits9,
}

pub enum Parity {
    ParityNone,
    ParityEven,
    ParityOdd,
}

pub enum StopBits {
    /// 1 stop bit
    STOP1,
    /// 2 stop bits
    STOP2,
}

pub struct Config {
    pub baudrate: Bps,
    pub wordlength: WordLength,
    pub parity: Parity,
    pub stopbits: StopBits,
}

impl Config {
    pub fn baudrate(mut self, baudrate: Bps) -> Self {
        self.baudrate = baudrate;
        self
    }

    pub fn wordlength(mut self, wordlength: WordLength) -> Self {
        self.wordlength = wordlength;
        self
    }

    pub fn wordlength_8bits(mut self) -> Self {
        self.wordlength = WordLength::Bits8;
        self
    }

    pub fn wordlength_9bits(mut self) -> Self {
        self.wordlength = WordLength::Bits9;
        self
    }

    pub fn parity(mut self, parity: Parity) -> Self {
        self.parity = parity;
        self
    }

    pub fn parity_none(mut self) -> Self {
        self.parity = Parity::ParityNone;
        self
    }

    pub fn parity_even(mut self) -> Self {
        self.parity = Parity::ParityEven;
        self
    }

    pub fn parity_odd(mut self) -> Self {
        self.parity = Parity::ParityOdd;
        self
    }

    pub fn stopbits(mut self, stopbits: StopBits) -> Self {
        self.stopbits = stopbits;
        self
    }
}

impl Default for Config {
    fn default() -> Config {
        Config {
            baudrate: 115_200_u32.bps(),
            wordlength: WordLength::Bits8,
            parity: Parity::ParityNone,
            stopbits: StopBits::STOP1,
        }
    }
}

impl From<Bps> for Config {
    fn from(baud: Bps) -> Self {
        Config::default().baudrate(baud)
    }
}

/// Serial abstraction
pub struct Serial<USART: Instance, TXPIN, RXPIN> {
    pub tx: Tx<USART>,
    pub rx: Rx<USART>,
    #[allow(clippy::type_complexity)]
    pub token: ReleaseToken<USART, (TXPIN, RXPIN)>,
}

/// Serial transmitter
pub struct Tx<USART> {
    _usart: PhantomData<USART>,
}

/// Serial receiver
pub struct Rx<USART> {
    _usart: PhantomData<USART>,
}

/// Stores data for release
pub struct ReleaseToken<USART, PINS> {
    usart: USART,
    pins: PINS,
}

impl<USART: Instance, TXPIN> Serial<USART, TXPIN, ()> {
    pub fn tx(
        usart: USART,
        tx_pin: TXPIN,
        config: impl Into<Config>,
        clocks: &Clocks,
    ) -> Tx<USART> {
        Self::_new(usart, (tx_pin, ()), config, clocks).split().0
    }
}

impl<USART: Instance, RXPIN> Serial<USART, (), RXPIN> {
    pub fn rx(
        usart: USART,
        rx_pin: RXPIN,
        config: impl Into<Config>,
        clocks: &Clocks,
    ) -> Rx<USART> {
        Self::_new(usart, ((), rx_pin), config, clocks).split().1
    }
}

impl<USART: Instance, TXPIN, RXPIN> Serial<USART, TXPIN, RXPIN> {
    /// Configures the serial interface and creates the interface
    /// struct.
    ///
    /// `Bps` is the baud rate of the interface.
    ///
    /// `Clocks` passes information about the current frequencies of
    /// the clocks.  The existence of the struct ensures that the
    /// clock settings are fixed.
    ///
    /// The `serial` struct takes ownership over the `USARTX` device
    /// registers and the specified `PINS`
    pub fn new(
        usart: USART,
        pins: (TXPIN, RXPIN),
        config: impl Into<Config>,
        clocks: &Clocks,
    ) -> Self {
        Self::_new(usart, (pins.0, pins.1), config, clocks)
    }

    fn _new(
        usart: USART,
        pins: (TXPIN, RXPIN),
        config: impl Into<Config>,
        clocks: &Clocks,
    ) -> Self {
        // Enable and reset USART
        let rcc = unsafe { &(*RCC::ptr()) };
        USART::enable(rcc);
        USART::reset(rcc);

        apply_config::<USART>(config.into(), clocks);

        let pins = (pins.0, pins.1);

        // UE: enable USART
        // TE: enable transceiver
        // RE: enable receiver
        usart.cr1.modify(|_r, w| {
            w.ue().enabled();
            w.te().enabled();
            w.re().enabled()
        });

        Serial {
            tx: Tx {
                _usart: PhantomData,
            },
            rx: Rx {
                _usart: PhantomData,
            },
            token: ReleaseToken { usart, pins },
        }
    }
}

impl<USART: Instance, TXPIN, RXPIN> Serial<USART, TXPIN, RXPIN> {
    /// Reconfigure the USART instance.
    ///
    /// If a transmission is currently in progress, this returns
    /// [`nb::Error::WouldBlock`].
    pub fn reconfigure(
        &mut self,
        config: impl Into<Config>,
        clocks: &Clocks,
    ) -> nb::Result<(), Error> {
        reconfigure(&mut self.tx, &mut self.rx, config, clocks)
    }

    /// Returns ownership of the borrowed register handles
    ///
    /// # Examples
    ///
    /// Basic usage:
    ///
    /// ```
    /// let mut serial = Serial::new(usart, (tx_pin, rx_pin), 9600.bps(), &clocks);
    ///
    /// // You can split the `Serial`
    /// let Serial { tx, rx, token } = serial;
    ///
    /// // You can reunite the `Serial` back
    /// let serial = Serial { tx, rx, token };
    ///
    /// // Release `Serial`
    /// let (usart, (tx_pin, rx_pin)) = serial.release();
    /// ```
    #[allow(clippy::type_complexity)]
    pub fn release(self) -> (USART, (TXPIN, RXPIN)) {
        (self.token.usart, self.token.pins)
    }

    /// Separates the serial struct into separate channel objects for sending (Tx) and
    /// receiving (Rx)
    ///
    /// If in the future it will be necessary to free up resources,
    /// then you need to use a different method of separation:
    ///
    /// ```
    /// let Serial { tx, rx, token } = serial;
    /// ```
    pub fn split(self) -> (Tx<USART>, Rx<USART>) {
        (self.tx, self.rx)
    }
}

fn apply_config<USART: Instance>(config: Config, clocks: &Clocks) {
    let usart = unsafe { &*USART::ptr() };

    // Configure baud rate
    let brr = USART::clock(clocks).0 / config.baudrate.0;
    assert!(brr >= 16, "impossible baud rate");
    usart.brr.write(|w| unsafe { w.bits(brr) });

    // Configure word
    usart.cr1.modify(|_r, w| {
        use crate::pac::usart1::cr1::M_A;
        w.m().variant(match config.wordlength {
            WordLength::Bits8 => M_A::M8,
            WordLength::Bits9 => M_A::M9,
        });
        use crate::pac::usart1::cr1::PS_A;
        w.ps().variant(match config.parity {
            Parity::ParityOdd => PS_A::Odd,
            _ => PS_A::Even,
        });
        w.pce().bit(!matches!(config.parity, Parity::ParityNone));
        w
    });

    // Configure stop bits
    use crate::pac::usart1::cr2::STOP_A;
    let stop_bits = match config.stopbits {
        StopBits::STOP1 => STOP_A::Stop1,
        StopBits::STOP2 => STOP_A::Stop2,
    };
    usart.cr2.modify(|_r, w| w.stop().variant(stop_bits));
}

/// Reconfigure the USART instance.
///
/// If a transmission is currently in progress, this returns
/// [`nb::Error::WouldBlock`].
pub fn reconfigure<USART: Instance>(
    tx: &mut Tx<USART>,
    #[allow(unused_variables)] rx: &mut Rx<USART>,
    config: impl Into<Config>,
    clocks: &Clocks,
) -> nb::Result<(), Error> {
    // if we're currently busy transmitting, we have to wait until that is
    // over -- regarding reception, we assume that the caller -- with
    // exclusive access to the Serial instance due to &mut self -- knows
    // what they're doing.
    tx.flush()?;
    apply_config::<USART>(config.into(), clocks);
    Ok(())
}

impl<USART: Instance> Tx<USART> {
    /// Writes 9-bit words to the UART/USART
    ///
    /// If the UART/USART was configured with `WordLength::Bits9`, the 9 least significant bits will
    /// be transmitted and the other 7 bits will be ignored. Otherwise, the 8 least significant bits
    /// will be transmitted and the other 8 bits will be ignored.
    pub fn write_u16(&mut self, word: u16) -> nb::Result<(), Error> {
        let usart = unsafe { &*USART::ptr() };

        if usart.sr.read().txe().bit_is_set() {
            usart.dr().write(|w| w.dr().bits(word));
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    pub fn write_u8(&mut self, word: u8) -> nb::Result<(), Error> {
        self.write_u16(word as u16)
    }

    pub fn bwrite_all_u16(&mut self, buffer: &[u16]) -> Result<(), Error> {
        for &w in buffer {
            nb::block!(self.write_u16(w))?;
        }
        Ok(())
    }

    pub fn bwrite_all_u8(&mut self, buffer: &[u8]) -> Result<(), Error> {
        for &w in buffer {
            nb::block!(self.write_u8(w))?;
        }
        Ok(())
    }

    pub fn flush(&mut self) -> nb::Result<(), Error> {
        let usart = unsafe { &*USART::ptr() };

        if usart.sr.read().tc().bit_is_set() {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
    }

    pub fn bflush(&mut self) -> Result<(), Error> {
        nb::block!(self.flush())
    }

    /// Start listening for transmit interrupt event
    pub fn listen(&mut self) {
        unsafe { (*USART::ptr()).cr1.modify(|_, w| w.txeie().set_bit()) };
    }

    /// Stop listening for transmit interrupt event
    pub fn unlisten(&mut self) {
        unsafe { (*USART::ptr()).cr1.modify(|_, w| w.txeie().clear_bit()) };
    }

    /// Returns true if the tx register is empty (and can accept data)
    pub fn is_tx_empty(&self) -> bool {
        unsafe { (*USART::ptr()).sr.read().txe().bit_is_set() }
    }

    /// Returns true if the transmission is complete, transmitter can be turned off safely
    pub fn is_tx_complete(&self) -> bool {
        unsafe { (*USART::ptr()).sr.read().tc().bit_is_set() }
    }
}

impl<USART: Instance> core::fmt::Write for Tx<USART> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        s.bytes()
            .try_for_each(|c| nb::block!(self.write_u8(c)))
            .map_err(|_| core::fmt::Error)
    }
}

impl<USART: Instance> Rx<USART> {
    /// Reads 9-bit words from the UART/USART
    ///
    /// If the UART/USART was configured with `WordLength::Bits9`, the returned value will contain
    /// 9 received data bits and all other bits set to zero. Otherwise, the returned value will contain
    /// 8 received data bits and all other bits set to zero.
    pub fn read_u16(&mut self) -> nb::Result<u16, Error> {
        let usart = unsafe { &*USART::ptr() };
        let sr = usart.sr.read();

        // Check for any errors
        let err = if sr.pe().bit_is_set() {
            Some(Error::Parity)
        } else if sr.fe().bit_is_set() {
            Some(Error::FrameFormat)
        } else if sr.ne().bit_is_set() {
            Some(Error::Noise)
        } else if sr.ore().bit_is_set() {
            Some(Error::Overrun)
        } else {
            None
        };

        if let Some(err) = err {
            // Some error occurred. In order to clear that error flag, you have to
            // do a read from the sr register followed by a read from the dr register.
            let _ = usart.sr.read();
            let _ = usart.dr().read();
            Err(nb::Error::Other(err))
        } else {
            // Check if a byte is available
            if sr.rxne().bit_is_set() {
                // Read the received byte
                Ok(usart.dr().read().dr().bits())
            } else {
                Err(nb::Error::WouldBlock)
            }
        }
    }

    pub fn read(&mut self) -> nb::Result<u8, Error> {
        self.read_u16().map(|word16| word16 as u8)
    }

    /// Start listening for receive interrupt event
    pub fn listen(&mut self) {
        unsafe { (*USART::ptr()).cr1.modify(|_, w| w.rxneie().set_bit()) };
    }

    /// Stop listening for receive interrupt event
    pub fn unlisten(&mut self) {
        unsafe { (*USART::ptr()).cr1.modify(|_, w| w.rxneie().clear_bit()) };
    }

    /// Start listening for idle interrupt event
    pub fn listen_idle(&mut self) {
        unsafe { (*USART::ptr()).cr1.modify(|_, w| w.idleie().set_bit()) };
    }

    /// Stop listening for idle interrupt event
    pub fn unlisten_idle(&mut self) {
        unsafe { (*USART::ptr()).cr1.modify(|_, w| w.idleie().clear_bit()) };
    }

    /// Returns true if the line idle status is set
    pub fn is_idle(&self) -> bool {
        unsafe { (*USART::ptr()).sr.read().idle().bit_is_set() }
    }

    /// Returns true if the rx register is not empty (and can be read)
    pub fn is_rx_not_empty(&self) -> bool {
        unsafe { (*USART::ptr()).sr.read().rxne().bit_is_set() }
    }

    /// Clear idle line interrupt flag
    pub fn clear_idle_interrupt(&self) {
        unsafe {
            let _ = (*USART::ptr()).sr.read();
            let _ = (*USART::ptr()).dr().read();
        }
    }
}

/// Interrupt event
pub enum Event {
    /// New data can be sent
    Txe,
    /// New data has been received
    Rxne,
    /// Idle line state detected
    Idle,
}

impl<USART: Instance, TXPIN, RXPIN> Serial<USART, TXPIN, RXPIN> {
    /// Starts listening to the USART by enabling the _Received data
    /// ready to be read (RXNE)_ interrupt and _Transmit data
    /// register empty (TXE)_ interrupt
    pub fn listen(&mut self, event: Event) {
        match event {
            Event::Rxne => self.rx.listen(),
            Event::Txe => self.tx.listen(),
            Event::Idle => self.rx.listen_idle(),
        }
    }

    /// Stops listening to the USART by disabling the _Received data
    /// ready to be read (RXNE)_ interrupt and _Transmit data
    /// register empty (TXE)_ interrupt
    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::Rxne => self.rx.unlisten(),
            Event::Txe => self.tx.unlisten(),
            Event::Idle => self.rx.unlisten_idle(),
        }
    }

    /// Returns true if the line idle status is set
    pub fn is_idle(&self) -> bool {
        self.rx.is_idle()
    }

    /// Returns true if the tx register is empty (and can accept data)
    pub fn is_tx_empty(&self) -> bool {
        self.tx.is_tx_empty()
    }

    /// Returns true if the rx register is not empty (and can be read)
    pub fn is_rx_not_empty(&self) -> bool {
        self.rx.is_rx_not_empty()
    }

    /// Clear idle line interrupt flag
    pub fn clear_idle_interrupt(&self) {
        self.rx.clear_idle_interrupt();
    }
}

impl<USART: Instance, TXPIN, RXPIN> core::fmt::Write for Serial<USART, TXPIN, RXPIN> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.tx.write_str(s)
    }
}

pub type Rx1 = Rx<pac::USART1>;
pub type Tx1 = Tx<pac::USART1>;
#[cfg(any(feature = "py32f003", feature = "py32f030",))]
pub type Rx2 = Rx<pac::USART2>;
#[cfg(any(feature = "py32f003", feature = "py32f030",))]
pub type Tx2 = Tx<pac::USART2>;

#[cfg(feature = "with-dma")]
macro_rules! serialdmarx {
    ($USARTX:ty: ($dmaX:ty, $dmamux:expr, {
            $($rxdma:ident: $ch:literal,)+
    }),) => {

        impl Rx<$USARTX> {
            pub fn with_dma<DMA: DmaExt, const C: u8>(
                self,
                mut channel: Ch<DMA, C>,
            ) -> RxDma<Rx<$USARTX>, Ch<DMA, C>> {
                unsafe {
                    // turn on syscfg clock and set mux
                    pac::SYSCFG::enable(&*pac::RCC::ptr());
                    channel.set_map($dmamux);
                    (*<$USARTX>::ptr()).cr3.modify(|_, w| w.dmar().set_bit());
                }
                RxDma {
                    payload: self,
                    channel,
                }
            }
        }

        $(
            pub type $rxdma = RxDma<Rx<$USARTX>, Ch<$dmaX, $ch>>;

            impl Receive for $rxdma {
                type RxChannel = Ch<$dmaX, $ch>;
                type TransmittedWord = u8;
            }

            impl TransferPayload for $rxdma {
                fn start(&mut self) {
                    self.channel.start();
                }
                fn stop(&mut self) {
                    self.channel.stop();
                }
            }

            impl $rxdma {
                pub fn release(mut self) -> (Rx<$USARTX>, Ch<$dmaX, $ch>) {
                    self.stop();
                    unsafe {
                        (*<$USARTX>::ptr()).cr3.modify(|_, w| w.dmar().clear_bit());
                    }
                    // reset the mux to default
                    self.channel.set_map(PeriphMap::Adc);
                    let RxDma { payload, channel } = self;
                    (payload, channel)
                }
            }

            impl<B> crate::dma::CircReadDma<B, u8> for $rxdma
            where
                &'static mut [B; 2]: WriteBuffer<Word = u8>,
                B: 'static,
            {
                fn circ_read(mut self, mut buffer: &'static mut [B; 2]) -> CircBuffer<B, Self> {
                    // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                    // until the end of the transfer.
                    let (ptr, len) = unsafe { buffer.write_buffer() };
                    self.channel.set_peripheral_address(
                        unsafe { (*<$USARTX>::ptr()).dr().as_ptr() as u32 },
                        false,
                    );
                    self.channel.set_memory_address(ptr as u32, true);
                    self.channel.set_transfer_length(len);

                    atomic::compiler_fence(Ordering::Release);

                    self.channel.ch().cr.modify(|_, w| {
                        w.mem2mem().clear_bit();
                        w.pl().medium();
                        w.msize().bits8();
                        w.psize().bits8();
                        w.circ().set_bit();
                        w.dir().clear_bit()
                    });

                    self.start();

                    CircBuffer::new(buffer, self)
                }
            }

            impl<B> crate::dma::ReadDma<B, u8> for $rxdma
            where
                B: WriteBuffer<Word = u8>,
            {
                fn read(mut self, mut buffer: B) -> Transfer<dma::W, B, Self> {
                    // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                    // until the end of the transfer.
                    let (ptr, len) = unsafe { buffer.write_buffer() };
                    self.channel.set_peripheral_address(
                        unsafe { (*<$USARTX>::ptr()).dr().as_ptr() as u32 },
                        false,
                    );
                    self.channel.set_memory_address(ptr as u32, true);
                    self.channel.set_transfer_length(len);

                    atomic::compiler_fence(Ordering::Release);
                    self.channel.ch().cr.modify(|_, w| {
                        w.mem2mem().clear_bit();
                        w.pl().medium();
                        w.msize().bits8();
                        w.psize().bits8();
                        w.circ().clear_bit();
                        w.dir().clear_bit()
                    });
                    self.start();

                    Transfer::w(buffer, self)
                }
            }

        )+

    }
}

#[cfg(any(feature = "py32f003", feature = "py32f030"))]
serialdmarx! {
    pac::USART1: (pac::DMA, PeriphMap::Usart1Rx, {
        Rx1Dma1: 1,
        Rx1Dma2: 2,
        Rx1Dma3: 3,
    }),
}
#[cfg(any(feature = "py32f003", feature = "py32f030",))]
serialdmarx! {
    pac::USART2: (pac::DMA, PeriphMap::Usart2Rx, {
        Rx2Dma1: 1,
        Rx2Dma2: 2,
        Rx2Dma3: 3,
    }),
}

#[cfg(feature = "with-dma")]
macro_rules! serialdmatx {
    ($USARTX:ty: ($dmaX:ty, $dmamux:expr, {
            $($txdma:ident: $ch:literal,)+
    }),) => {

        impl Tx<$USARTX> {
            pub fn with_dma<DMA: DmaExt, const C: u8>(
                self,
                mut channel: Ch<DMA, C>,
            ) -> TxDma<Tx<$USARTX>, Ch<DMA, C>> {
                unsafe {
                    // turn on syscfg clock and set mux
                    pac::SYSCFG::enable(&*pac::RCC::ptr());
                    channel.set_map($dmamux);
                    (*<$USARTX>::ptr()).cr3.modify(|_, w| w.dmat().set_bit());
                }
                TxDma {
                    payload: self,
                    channel,
                }
            }
        }

        $(
            pub type $txdma = TxDma<Tx<$USARTX>, Ch<$dmaX, $ch>>;

            impl Transmit for $txdma {
                type TxChannel = Ch<$dmaX, $ch>;
                type ReceivedWord = u8;
            }

            impl TransferPayload for $txdma {
                fn start(&mut self) {
                    self.channel.start();
                }
                fn stop(&mut self) {
                    self.channel.stop();
                }
            }

            impl $txdma {
                pub fn release(mut self) -> (Tx<$USARTX>, Ch<$dmaX, $ch>) {
                    self.stop();
                    unsafe {
                        (*<$USARTX>::ptr()).cr3.modify(|_, w| w.dmat().clear_bit());
                    }
                    // reset the mux to default
                    self.channel.set_map(PeriphMap::Adc);
                    let TxDma { payload, channel } = self;
                    (payload, channel)
                }
            }

            impl<B> crate::dma::WriteDma<B, u8> for $txdma
            where
                B: ReadBuffer<Word = u8>,
            {
                fn write(mut self, buffer: B) -> Transfer<dma::R, B, Self> {
                    // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                    // until the end of the transfer.
                    let (ptr, len) = unsafe { buffer.read_buffer() };

                    self.channel.set_peripheral_address(
                        unsafe { (*<$USARTX>::ptr()).dr().as_ptr() as u32 },
                        false,
                    );

                    self.channel.set_memory_address(ptr as u32, true);
                    self.channel.set_transfer_length(len);

                    atomic::compiler_fence(Ordering::Release);

                    self.channel.ch().cr.modify(|_, w| {
                        w.mem2mem().clear_bit();
                        w.pl().medium();
                        w.msize().bits8();
                        w.psize().bits8();
                        w.circ().clear_bit();
                        w.dir().set_bit()
                    });
                    self.start();

                    Transfer::r(buffer, self)
                }
            }
        )+
    }
}

#[cfg(any(feature = "py32f003", feature = "py32f030"))]
serialdmatx! {
    pac::USART1: (pac::DMA, PeriphMap::Usart1Tx, {
        Tx1Dma1: 1,
        Tx1Dma2: 2,
        Tx1Dma3: 3,
    }),
}
#[cfg(any(feature = "py32f003", feature = "py32f030",))]
serialdmatx! {
    pac::USART2: (pac::DMA, PeriphMap::Usart2Tx, {
        Tx2Dma1: 1,
        Tx2Dma2: 2,
        Tx2Dma3: 3,
    }),
}
