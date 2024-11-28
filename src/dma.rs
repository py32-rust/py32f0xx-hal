//! # Direct Memory Access
#![allow(dead_code)]

use crate::pac;
use core::{
    convert::TryFrom,
    marker::PhantomData,
    mem, ptr,
    sync::atomic::{self, compiler_fence, Ordering},
};
use embedded_dma::{ReadBuffer, WriteBuffer};

#[derive(Debug)]
#[non_exhaustive]
pub enum Error {
    Overrun,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Event {
    HalfTransfer,
    TransferComplete,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Half {
    First,
    Second,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Priority {
    Low,
    Medium,
    High,
    VeryHigh,
}

impl core::convert::From<Priority> for u8 {
    fn from(val: Priority) -> u8 {
        match val {
            Priority::Low => 0,
            Priority::Medium => 1,
            Priority::High => 2,
            Priority::VeryHigh => 3,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TransferDir {
    FromMemory,
    FromPeripheral,
}

impl core::convert::From<TransferDir> for bool {
    fn from(val: TransferDir) -> bool {
        match val {
            TransferDir::FromMemory => true,
            TransferDir::FromPeripheral => false,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DataSize {
    Bits8,
    Bits16,
    Bits32,
}

impl core::convert::From<DataSize> for u8 {
    fn from(val: DataSize) -> u8 {
        match val {
            DataSize::Bits8 => 0,
            DataSize::Bits16 => 1,
            DataSize::Bits32 => 2,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PeriphMap {
    Adc,
    Spi1Tx,
    Spi1Rx,
    Spi2Tx,
    Spi2Rx,
    Usart1Tx,
    Usart1Rx,
    Usart2Tx,
    Usart2Rx,
    I2cTx,
    I2cRx,
    Tim1Ch1,
    Tim1Ch2,
    Tim1Ch3,
    Tim1Ch4,
    Tim1Com,
    Tim1Up,
    Tim1Trig,
    Tim3Ch1,
    Tim3Ch3,
    Tim3Ch4,
    Tim3Trig,
    Tim3Up,
    Tim16Ch1,
    Tim16Up,
    Tim17Ch1,
    Tim17Up,
}

impl core::convert::From<PeriphMap> for u32 {
    fn from(val: PeriphMap) -> u32 {
        match val {
            PeriphMap::Adc => 0,
            PeriphMap::Spi1Tx => 1,
            PeriphMap::Spi1Rx => 2,
            PeriphMap::Spi2Tx => 3,
            PeriphMap::Spi2Rx => 4,
            PeriphMap::Usart1Tx => 5,
            PeriphMap::Usart1Rx => 6,
            PeriphMap::Usart2Tx => 7,
            PeriphMap::Usart2Rx => 8,
            PeriphMap::I2cTx => 9,
            PeriphMap::I2cRx => 10,
            PeriphMap::Tim1Ch1 => 11,
            PeriphMap::Tim1Ch2 => 12,
            PeriphMap::Tim1Ch3 => 13,
            PeriphMap::Tim1Ch4 => 14,
            PeriphMap::Tim1Com => 15,
            PeriphMap::Tim1Up => 16,
            PeriphMap::Tim1Trig => 17,
            PeriphMap::Tim3Ch1 => 18,
            PeriphMap::Tim3Ch3 => 19,
            PeriphMap::Tim3Ch4 => 20,
            PeriphMap::Tim3Trig => 21,
            PeriphMap::Tim3Up => 22,
            PeriphMap::Tim16Ch1 => 24,
            PeriphMap::Tim16Up => 25,
            PeriphMap::Tim17Ch1 => 26,
            PeriphMap::Tim17Up => 27,
        }
    }
}

pub struct CircBuffer<BUFFER, PAYLOAD>
where
    BUFFER: 'static,
{
    buffer: &'static mut [BUFFER; 2],
    payload: PAYLOAD,
    readable_half: Half,
}

impl<BUFFER, PAYLOAD> CircBuffer<BUFFER, PAYLOAD>
where
    &'static mut [BUFFER; 2]: WriteBuffer,
    BUFFER: 'static,
{
    pub(crate) fn new(buf: &'static mut [BUFFER; 2], payload: PAYLOAD) -> Self {
        CircBuffer {
            buffer: buf,
            payload,
            readable_half: Half::Second,
        }
    }
}

pub trait DmaExt {
    type Channels;

    fn split(self) -> Self::Channels;

    fn ptr() -> *const pac::dma::RegisterBlock;
}

pub trait TransferPayload {
    fn start(&mut self);
    fn stop(&mut self);
}

pub struct Transfer<MODE, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    _mode: PhantomData<MODE>,
    buffer: BUFFER,
    payload: PAYLOAD,
}

impl<BUFFER, PAYLOAD> Transfer<R, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    pub(crate) fn r(buffer: BUFFER, payload: PAYLOAD) -> Self {
        Transfer {
            _mode: PhantomData,
            buffer,
            payload,
        }
    }
}

impl<BUFFER, PAYLOAD> Transfer<W, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    pub(crate) fn w(buffer: BUFFER, payload: PAYLOAD) -> Self {
        Transfer {
            _mode: PhantomData,
            buffer,
            payload,
        }
    }
}

impl<MODE, BUFFER, PAYLOAD> Drop for Transfer<MODE, BUFFER, PAYLOAD>
where
    PAYLOAD: TransferPayload,
{
    fn drop(&mut self) {
        self.payload.stop();
        compiler_fence(Ordering::SeqCst);
    }
}

/// Read transfer
pub struct R;

/// Write transfer
pub struct W;

/// A singleton that represents a single DMAx channel (channel X in this case)
///
/// This singleton has exclusive access to the registers of the DMAx channel X
#[non_exhaustive]
pub struct Ch<DMA, const C: u8>(PhantomData<DMA>);

impl<DMA: DmaExt, const C: u8> Ch<DMA, C> {
    /// Set the Dma request `map` to the specified peripheral
    pub fn set_map(&mut self, map: PeriphMap) {
        unsafe {
            (*pac::SYSCFG::ptr()).cfgr3.modify(|r, w| {
                w.bits(
                    (r.bits() & !(0x1f << ((C - 1) * 8)))
                        | (Into::<u32>::into(map) << ((C - 1) * 8)),
                )
            });
        }
    }

    /// Associated peripheral `address`
    ///
    /// `inc` indicates whether the address will be incremented after every byte transfer
    pub fn set_peripheral_address(&mut self, address: u32, inc: bool) {
        self.ch().par.write(|w| w.pa().bits(address));
        self.ch().cr.modify(|_, w| w.pinc().bit(inc));
    }

    /// `address` where from/to data will be read/write
    ///
    /// `inc` indicates whether the address will be incremented after every byte transfer
    pub fn set_memory_address(&mut self, address: u32, inc: bool) {
        self.ch().mar.write(|w| w.ma().bits(address));
        self.ch().cr.modify(|_, w| w.minc().bit(inc));
    }

    /// 'priority' sets the dma channel priority
    pub fn set_priority(&mut self, priority: Priority) {
        let prio_bits: u8 = priority.into();
        self.ch().cr.modify(|_, w| w.pl().bits(prio_bits));
    }

    /// Number of bytes to transfer
    pub fn set_transfer_length(&mut self, len: usize) {
        self.ch()
            .ndtr
            .write(|w| w.ndt().bits(u16::try_from(len).unwrap()));
    }

    /// Starts the DMA transfer
    pub fn start(&mut self) {
        // TODO: clear all the flags for a channel. There is no
        // channel way of clearing the flags yet, so manipulate
        // the bits directly, not ideal
        // need an indexing method in the pac
        self.ifcr()
            .write(|w| unsafe { w.bits(0xF << ((C - 1) * 4)) });
        self.ch().cr.modify(|_, w| w.en().set_bit());
    }

    /// Stops the DMA transfer
    pub fn stop(&mut self) {
        // TODO: do bit ops to get cgif flag cleared, until pac has indexing method
        self.ifcr().write(|w| unsafe { w.bits(1 << ((C - 1) * 4)) });
        self.ch().cr.modify(|_, w| w.en().clear_bit());
    }

    /// Returns `true` if there's a transfer in progress
    pub fn in_progress(&self) -> bool {
        unsafe { self.isr().tcif(C).bit_is_clear() }
    }

    pub fn listen(&mut self, event: Event) {
        match event {
            Event::HalfTransfer => self.ch().cr.modify(|_, w| w.htie().set_bit()),
            Event::TransferComplete => self.ch().cr.modify(|_, w| w.tcie().set_bit()),
        }
    }

    pub fn unlisten(&mut self, event: Event) {
        match event {
            Event::HalfTransfer => self.ch().cr.modify(|_, w| w.htie().clear_bit()),
            Event::TransferComplete => self.ch().cr.modify(|_, w| w.tcie().clear_bit()),
        }
    }

    pub fn ch(&mut self) -> &pac::dma::CH {
        unsafe { &(*DMA::ptr()).ch[C as usize - 1] }
    }

    pub fn isr(&self) -> pac::dma::isr::R {
        // NOTE(unsafe) atomic read with no side effects
        unsafe { (*DMA::ptr()).isr.read() }
    }

    pub fn ifcr(&self) -> &pac::dma::IFCR {
        unsafe { &(*DMA::ptr()).ifcr }
    }

    pub fn get_ndtr(&self) -> u32 {
        // NOTE(unsafe) atomic read with no side effects
        unsafe { (*DMA::ptr()).ch[C as usize - 1].ndtr.read().bits() }
    }
}

impl<B, PAYLOAD, DMA: DmaExt, const C: u8> CircBuffer<B, RxDma<PAYLOAD, Ch<DMA, C>>>
where
    RxDma<PAYLOAD, Ch<DMA, C>>: TransferPayload,
{
    /// Peeks into the readable half of the buffer
    pub fn peek<R, F>(&mut self, f: F) -> Result<R, Error>
    where
        F: FnOnce(&B, Half) -> R,
    {
        let half_being_read = self.readable_half()?;

        let buf = match half_being_read {
            Half::First => &self.buffer[0],
            Half::Second => &self.buffer[1],
        };

        // XXX does this need a compiler barrier?
        let ret = f(buf, half_being_read);

        let isr = self.payload.channel.isr();
        let first_half_is_done = unsafe { isr.htif(C).bit_is_set() };
        let second_half_is_done = unsafe { isr.tcif(C).bit_is_set() };

        if (half_being_read == Half::First && second_half_is_done)
            || (half_being_read == Half::Second && first_half_is_done)
        {
            Err(Error::Overrun)
        } else {
            Ok(ret)
        }
    }

    /// Returns the `Half` of the buffer that can be read
    pub fn readable_half(&mut self) -> Result<Half, Error> {
        let isr = self.payload.channel.isr();
        let first_half_is_done = unsafe { isr.htif(C).bit_is_set() };
        let second_half_is_done = unsafe { isr.tcif(C).bit_is_set() };

        if first_half_is_done && second_half_is_done {
            return Err(Error::Overrun);
        }

        let last_read_half = self.readable_half;

        Ok(match last_read_half {
            Half::First => {
                if second_half_is_done {
                    self.payload
                        .channel
                        .ifcr()
                        // TODO: do bit ops to get ctif flag cleared, until pac has indexing method
                        .write(|w| unsafe { w.bits(1 << (((C - 1) * 4) + 1)) });

                    self.readable_half = Half::Second;
                    Half::Second
                } else {
                    last_read_half
                }
            }
            Half::Second => {
                if first_half_is_done {
                    self.payload
                        .channel
                        .ifcr()
                        // TODO: do bit ops to get htif flag cleared, until pac has indexing method
                        .write(|w| unsafe { w.bits(1 << (((C - 1) * 4) + 2)) });

                    self.readable_half = Half::First;
                    Half::First
                } else {
                    last_read_half
                }
            }
        })
    }

    /// Stops the transfer and returns the underlying buffer and RxDma
    pub fn stop(mut self) -> (&'static mut [B; 2], RxDma<PAYLOAD, Ch<DMA, C>>) {
        self.payload.stop();

        (self.buffer, self.payload)
    }
}

impl<BUFFER, PAYLOAD, MODE, DMA: DmaExt, const C: u8>
    Transfer<MODE, BUFFER, RxDma<PAYLOAD, Ch<DMA, C>>>
where
    RxDma<PAYLOAD, Ch<DMA, C>>: TransferPayload,
{
    /// Is the Transfer completed
    pub fn is_done(&self) -> bool {
        !self.payload.channel.in_progress()
    }

    /// Listen for Events on DMA channel
    pub fn listen(&mut self, evt: Event) {
        self.payload.channel.listen(evt);
    }

    /// Unlisten for Events on DMA channel
    pub fn unlisten(&mut self, evt: Event) {
        self.payload.channel.unlisten(evt);
    }

    /// Wait until Transfer is completed, blocking
    pub fn wait(mut self) -> (BUFFER, RxDma<PAYLOAD, Ch<DMA, C>>) {
        while !self.is_done() {}

        atomic::compiler_fence(Ordering::Acquire);

        self.payload.stop();

        // we need a read here to make the Acquire fence effective
        // we do *not* need this if `dma.stop` does a RMW operation
        unsafe {
            ptr::read_volatile(&0);
        }

        // we need a fence here for the same reason we need one in `Transfer.wait`
        atomic::compiler_fence(Ordering::Acquire);

        // `Transfer` needs to have a `Drop` implementation, because we accept
        // managed buffers that can free their memory on drop. Because of that
        // we can't move out of the `Transfer`'s fields, so we use `ptr::read`
        // and `mem::forget`.
        //
        // NOTE(unsafe) There is no panic branch between getting the resources
        // and forgetting `self`.
        unsafe {
            let buffer = ptr::read(&self.buffer);
            let payload = ptr::read(&self.payload);
            mem::forget(self);
            (buffer, payload)
        }
    }
}

impl<BUFFER, PAYLOAD, MODE, DMA: DmaExt, const C: u8>
    Transfer<MODE, BUFFER, TxDma<PAYLOAD, Ch<DMA, C>>>
where
    TxDma<PAYLOAD, Ch<DMA, C>>: TransferPayload,
{
    /// Is the Transfer completed
    pub fn is_done(&self) -> bool {
        !self.payload.channel.in_progress()
    }

    /// Listen for Events on DMA channel
    pub fn listen(&mut self, evt: Event) {
        self.payload.channel.listen(evt);
    }

    /// Unlisten for Events on DMA channel
    pub fn unlisten(&mut self, evt: Event) {
        self.payload.channel.unlisten(evt);
    }

    /// Wait until Transfer is completed, blocking
    pub fn wait(mut self) -> (BUFFER, TxDma<PAYLOAD, Ch<DMA, C>>) {
        while !self.is_done() {}

        atomic::compiler_fence(Ordering::Acquire);

        self.payload.stop();

        // we need a read here to make the Acquire fence effective
        // we do *not* need this if `dma.stop` does a RMW operation
        unsafe {
            ptr::read_volatile(&0);
        }

        // we need a fence here for the same reason we need one in `Transfer.wait`
        atomic::compiler_fence(Ordering::Acquire);

        // `Transfer` needs to have a `Drop` implementation, because we accept
        // managed buffers that can free their memory on drop. Because of that
        // we can't move out of the `Transfer`'s fields, so we use `ptr::read`
        // and `mem::forget`.
        //
        // NOTE(unsafe) There is no panic branch between getting the resources
        // and forgetting `self`.
        unsafe {
            let buffer = ptr::read(&self.buffer);
            let payload = ptr::read(&self.payload);
            mem::forget(self);
            (buffer, payload)
        }
    }
}

impl<BUFFER, PAYLOAD, MODE, DMA: DmaExt, const RXC: u8, const TXC: u8>
    Transfer<MODE, BUFFER, RxTxDma<PAYLOAD, Ch<DMA, RXC>, Ch<DMA, TXC>>>
where
    RxTxDma<PAYLOAD, Ch<DMA, RXC>, Ch<DMA, TXC>>: TransferPayload,
{
    /// Is the Transfer completed
    pub fn is_done(&self) -> bool {
        !self.payload.rxchannel.in_progress()
    }

    /// Listen for Events on DMA rx channel
    pub fn listen_rx(&mut self, evt: Event) {
        self.payload.rxchannel.listen(evt);
    }

    /// Unlisten for Events on DMA rx channel
    pub fn unlisten_rx(&mut self, evt: Event) {
        self.payload.rxchannel.unlisten(evt);
    }

    /// Wait until Transfer is completed, blocking
    pub fn wait(mut self) -> (BUFFER, RxTxDma<PAYLOAD, Ch<DMA, RXC>, Ch<DMA, TXC>>) {
        while !self.is_done() {}

        atomic::compiler_fence(Ordering::Acquire);

        self.payload.stop();

        // we need a read here to make the Acquire fence effective
        // we do *not* need this if `dma.stop` does a RMW operation
        unsafe {
            ptr::read_volatile(&0);
        }

        // we need a fence here for the same reason we need one in `Transfer.wait`
        atomic::compiler_fence(Ordering::Acquire);

        // `Transfer` needs to have a `Drop` implementation, because we accept
        // managed buffers that can free their memory on drop. Because of that
        // we can't move out of the `Transfer`'s fields, so we use `ptr::read`
        // and `mem::forget`.
        //
        // NOTE(unsafe) There is no panic branch between getting the resources
        // and forgetting `self`.
        unsafe {
            let buffer = ptr::read(&self.buffer);
            let payload = ptr::read(&self.payload);
            mem::forget(self);
            (buffer, payload)
        }
    }
}

impl<BUFFER, PAYLOAD, DMA: DmaExt, const C: u8> Transfer<W, BUFFER, RxDma<PAYLOAD, Ch<DMA, C>>>
where
    RxDma<PAYLOAD, Ch<DMA, C>>: TransferPayload,
{
    pub fn peek<T>(&self) -> &[T]
    where
        BUFFER: AsRef<[T]>,
    {
        let pending = self.payload.channel.get_ndtr() as usize;

        let slice = self.buffer.as_ref();
        let capacity = slice.len();

        &slice[..(capacity - pending)]
    }
}

impl<RXBUFFER, TXBUFFER, PAYLOAD, DMA: DmaExt, const RXC: u8, const TXC: u8>
    Transfer<W, (RXBUFFER, TXBUFFER), RxTxDma<PAYLOAD, Ch<DMA, RXC>, Ch<DMA, TXC>>>
where
    RxTxDma<PAYLOAD, Ch<DMA, RXC>, Ch<DMA, TXC>>: TransferPayload,
{
    pub fn peek<T>(&self) -> &[T]
    where
        RXBUFFER: AsRef<[T]>,
    {
        let pending = self.payload.rxchannel.get_ndtr() as usize;

        let slice = self.buffer.0.as_ref();
        let capacity = slice.len();

        &slice[..(capacity - pending)]
    }
}

macro_rules! dma {
    ($DMAX:ident: ($dmaX:ident, {
        $($CX:ident: ($ch: literal),)+
    }),) => {
        pub mod $dmaX {
            use crate::dma::DmaExt;
            use crate::pac::{$DMAX, RCC};

            #[non_exhaustive]
            #[allow(clippy::manual_non_exhaustive)]
            pub struct Channels((), $(pub $CX),+);

            $(
                pub type $CX = super::Ch<$DMAX, $ch>;
            )+

            impl DmaExt for $DMAX {
                type Channels = Channels;

                fn split(self) -> Channels {
                    let rcc = unsafe { &(*RCC::ptr()) };
                    // only 1 DMA for clock enable
                    rcc.ahbenr.modify(|_,w| w.dmaen().enabled());

                    // reset the DMA control registers (stops all on-going transfers)
                    $(
                        self.ch[$ch - 1].cr.reset();
                    )+

                    Channels((), $(super::Ch::<$DMAX, $ch>(super::PhantomData)),+)
                }

                fn ptr() -> *const crate::pac::dma::RegisterBlock {
                    Self::ptr()
                }
            }
        }
    }
}

#[cfg(any(feature = "py32f003", feature = "py32f030"))]
dma! {
    DMA: (dma1, {
        C1: (1),
        C2: (2),
        C3: (3),
    }),
}

/// DMA Receiver
pub struct RxDma<PAYLOAD, RXCH> {
    pub(crate) payload: PAYLOAD,
    pub channel: RXCH,
}

/// DMA Transmitter
pub struct TxDma<PAYLOAD, TXCH> {
    pub(crate) payload: PAYLOAD,
    pub channel: TXCH,
}

/// DMA Receiver/Transmitter
pub struct RxTxDma<PAYLOAD, RXCH, TXCH> {
    pub(crate) payload: PAYLOAD,
    pub rxchannel: RXCH,
    pub txchannel: TXCH,
}

pub trait Receive {
    type RxChannel;
    type TransmittedWord;
}

pub trait Transmit {
    type TxChannel;
    type ReceivedWord;
}

/// Trait for circular DMA readings from peripheral to memory.
pub trait CircReadDma<B, RS>: Receive
where
    &'static mut [B; 2]: WriteBuffer<Word = RS>,
    B: 'static,
    Self: core::marker::Sized,
{
    fn circ_read(self, buffer: &'static mut [B; 2]) -> CircBuffer<B, Self>;
}

/// Trait for DMA readings from peripheral to memory.
pub trait ReadDma<B, RS>: Receive
where
    B: WriteBuffer<Word = RS>,
    Self: core::marker::Sized + TransferPayload,
{
    fn read(self, buffer: B) -> Transfer<W, B, Self>;
}

/// Trait for DMA writing from memory to peripheral.
pub trait WriteDma<B, TS>: Transmit
where
    B: ReadBuffer<Word = TS>,
    Self: core::marker::Sized + TransferPayload,
{
    fn write(self, buffer: B) -> Transfer<R, B, Self>;
}

/// Trait for DMA simultaneously reading and writing within one synchronous operation. Panics if both buffers are not of equal length.
pub trait ReadWriteDma<RXB, TXB, TS>: Transmit
where
    RXB: WriteBuffer<Word = TS>,
    TXB: ReadBuffer<Word = TS>,
    Self: core::marker::Sized + TransferPayload,
{
    fn read_write(self, rx_buffer: RXB, tx_buffer: TXB) -> Transfer<W, (RXB, TXB), Self>;
}
