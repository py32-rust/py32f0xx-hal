//! API for using DMA with integrated SPI peripheral
use super::*;

// DMA

use crate::dma::{
    self, dma1, Ch, DmaExt, PeriphMap, ReadDma, ReadWriteDma, Receive, RxDma, RxTxDma, Transfer,
    TransferPayload, Transmit, TxDma, WriteDma,
};

/// Alias for [Spi] in master mode transmitting with DMA on [Ch]
pub type SpiTxDma<SPI, SCKPIN, MISOPIN, MOSIPIN, CHANNEL> =
    TxDma<Spi<SPI, SCKPIN, MISOPIN, MOSIPIN, u8>, CHANNEL>;
/// Alias for [Spi] in master mode receiving with DMA on [Ch]
pub type SpiRxDma<SPI, SCKPIN, MISOPIN, MOSIPIN, CHANNEL> =
    RxDma<Spi<SPI, SCKPIN, MISOPIN, MOSIPIN, u8>, CHANNEL>;
/// Alias for [Spi] in master mode transmitting and receiving with DMA on [Ch]
pub type SpiRxTxDma<SPI, SCKPIN, MISOPIN, MOSIPIN, RXCHANNEL, TXCHANNEL> =
    RxTxDma<Spi<SPI, SCKPIN, MISOPIN, MOSIPIN, u8>, RXCHANNEL, TXCHANNEL>;

/// Alias for [Spi] in slave mode transmitting with DMA on [Ch]
pub type SpiSlaveTxDma<SPI, SCKPIN, MISOPIN, MOSIPIN, CHANNEL> =
    TxDma<SpiSlave<SPI, SCKPIN, MISOPIN, MOSIPIN, u8>, CHANNEL>;
/// Alias for [Spi] in slave mode receiving with DMA on [Ch]
pub type SpiSlaveRxDma<SPI, SCKPIN, MISOPIN, MOSIPIN, CHANNEL> =
    RxDma<SpiSlave<SPI, SCKPIN, MISOPIN, MOSIPIN, u8>, CHANNEL>;
/// Alias for [Spi] in slave mode transmitting and receiving with DMA on [Ch]
pub type SpiSlaveRxTxDma<SPI, SCKPIN, MISOPIN, MOSIPIN, RXCHANNEL, TXCHANNEL> =
    RxTxDma<SpiSlave<SPI, SCKPIN, MISOPIN, MOSIPIN, u8>, RXCHANNEL, TXCHANNEL>;

macro_rules! spi_dmarx {
    (
        $SPIi:ty: ($dmamux:expr, {
            $($rxdma:ident,  $slaverxdma:ident: $ch:ty,)+
        }),) => {

        impl<SCKPIN, MISOPIN, MOSIPIN> Spi<$SPIi, SCKPIN, MISOPIN, MOSIPIN, u8> {
            /// Use SPI instance in master mode with DMA on [Ch]
            pub fn with_rx_dma<DMA: DmaExt, const C: u8>(self, mut channel: Ch<DMA, C>) ->
                SpiRxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, Ch<DMA, C>> {
                // turn on syscfg clock and set mux
                unsafe { pac::SYSCFG::enable(&*pac::RCC::ptr()) };
                channel.set_map($dmamux);
                self.spi.cr2.modify(|_, w| w.rxdmaen().set_bit());
                SpiRxDma {
                    payload: self,
                    channel,
                }
            }
        }
        impl<SCKPIN, MISOPIN, MOSIPIN> SpiSlave<$SPIi, SCKPIN, MISOPIN, MOSIPIN, u8> {
            /// Use SPI instance in slave mode with DMA on [Ch]
            pub fn with_rx_dma<DMA: DmaExt, const C: u8>(self, mut channel: Ch<DMA, C>) ->
                SpiSlaveRxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, Ch<DMA, C>> {
                // turn on syscfg clock and set mux
                unsafe { pac::SYSCFG::enable(&*pac::RCC::ptr()) };
                channel.set_map($dmamux);
                self.spi.cr2.modify(|_, w| w.rxdmaen().set_bit());
                SpiSlaveRxDma {
                    payload: self,
                    channel,
                }
            }
        }

        $(
            /// Alias for SPI receive instance in master mode with DMA
            pub type $rxdma<SCKPIN, MISOPIN, MOSIPIN> = SpiRxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $ch>;

            impl<SCKPIN, MISOPIN, MOSIPIN> Receive for SpiRxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $ch> {
                type RxChannel = $ch;
                type TransmittedWord = u8;
            }

            impl<SCKPIN, MISOPIN, MOSIPIN> SpiRxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $ch> {
                /// Release the SPI instance, pins, and DMA channel
                pub fn release(mut self) -> (Spi<$SPIi, SCKPIN, MISOPIN, MOSIPIN, u8>, $ch) {
                    self.stop();
                    // reset the mux to default
                    self.channel.set_map(PeriphMap::Adc);
                    let SpiRxDma { payload, channel } = self;
                    payload.spi.cr2.modify(|_, w| w.rxdmaen().clear_bit());
                    (payload, channel)
                }
            }

            impl<SCKPIN, MISOPIN, MOSIPIN> TransferPayload for SpiRxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $ch> {
                fn start(&mut self) {
                    self.channel.start();
                }
                fn stop(&mut self) {
                    self.channel.stop();
                }
            }

            impl<B, SCKPIN, MISOPIN, MOSIPIN> ReadDma<B, u8> for SpiRxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $ch>
            where
                B: WriteBuffer<Word = u8>,
            {
                fn read(mut self, mut buffer: B) -> Transfer<dma::W, B, Self> {
                    // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                    // until the end of the transfer.
                    let (ptr, len) = unsafe { buffer.write_buffer() };
                    self.channel.set_peripheral_address(
                        unsafe { (*<$SPIi>::ptr()).dr().as_ptr() as u32 },
                        false,
                    );
                    self.channel.set_memory_address(ptr as u32, true);
                    self.channel.set_transfer_length(len);

                    atomic::compiler_fence(Ordering::Release);
                    self.channel.ch().cr.modify(|_, w| {
                        // memory to memory mode disabled
                        w.mem2mem().clear_bit();
                        // medium channel priority level
                        w.pl().medium();
                        // 8-bit memory size
                        w.msize().bits8();
                        // 8-bit peripheral size
                        w.psize().bits8();
                        // circular mode disabled
                        w.circ().clear_bit();
                        // write to memory
                        w.dir().clear_bit()
                    });
                    self.start();

                    Transfer::w(buffer, self)
                }
            }

            /// Alias for SPI receive instance in slave mode with DMA
            pub type $slaverxdma<SCKPIN, MISOPIN, MOSIPIN> =
                SpiSlaveRxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $ch>;


            impl<SCKPIN, MISOPIN, MOSIPIN> Receive for SpiSlaveRxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $ch> {
                type RxChannel = $ch;
                type TransmittedWord = u8;
            }

            impl<SCKPIN, MISOPIN, MOSIPIN> SpiSlaveRxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $ch> {
                /// Release SPI instance, pins and DMA channel
                pub fn release(mut self) -> (SpiSlave<$SPIi, SCKPIN, MISOPIN, MOSIPIN, u8>, $ch) {
                    self.stop();
                    // reset the mux to default
                    self.channel.set_map(PeriphMap::Adc);
                    let SpiSlaveRxDma { payload, channel } = self;
                    payload.spi.cr2.modify(|_, w| w.rxdmaen().clear_bit());
                    (payload, channel)
                }
            }

            impl<SCKPIN, MISOPIN, MOSIPIN> TransferPayload for SpiSlaveRxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $ch> {
                fn start(&mut self) {
                    self.channel.start();
                }
                fn stop(&mut self) {
                    self.channel.stop();
                }
            }

            impl<B, SCKPIN, MISOPIN, MOSIPIN> ReadDma<B, u8> for SpiSlaveRxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $ch>
            where
                B: WriteBuffer<Word = u8>,
            {
                fn read(mut self, mut buffer: B) -> Transfer<dma::W, B, Self> {
                    // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                    // until the end of the transfer.
                    let (ptr, len) = unsafe { buffer.write_buffer() };
                    self.channel.set_peripheral_address(
                        unsafe { (*<$SPIi>::ptr()).dr().as_ptr() as u32 },
                        false,
                    );
                    self.channel.set_memory_address(ptr as u32, true);
                    self.channel.set_transfer_length(len);

                    atomic::compiler_fence(Ordering::Release);
                    self.channel.ch().cr.modify(|_, w| {
                        // memory to memory mode disabled
                        w.mem2mem().clear_bit();
                        // medium channel priority level
                        w.pl().medium();
                        // 8-bit memory size
                        w.msize().bits8();
                        // 8-bit peripheral size
                        w.psize().bits8();
                        // circular mode disabled
                        w.circ().clear_bit();
                        // write to memory
                        w.dir().clear_bit()
                    });
                    self.start();

                    Transfer::w(buffer, self)
                }
            }

        )+

    };
}

#[cfg(any(feature = "py32f003", feature = "py32f030"))]
spi_dmarx! {
    pac::SPI1: (PeriphMap::Spi1Rx, {
        Spi1RxDma1, SpiSlave1RxDma1: dma1::C1,
        Spi1RxDma2, SpiSlave1RxDma2: dma1::C2,
        Spi1RxDma3, SpiSlave1RxDma3: dma1::C3,
    }),
}
#[cfg(any(feature = "py32f030"))]
spi_dmarx! {
    pac::SPI2: (PeriphMap::Spi2Rx, {
        Spi2RxDma1, SpiSlave2RxDma1: dma1::C1,
        Spi2RxDma2, SpiSlave2RxDma2: dma1::C2,
        Spi2RxDma3, SpiSlave2RxDma3: dma1::C3,
    }),
}

macro_rules! spi_dmatx {
    (
        $SPIi:ty: ($dmamux:expr, {
            $($txdma:ident,  $slavetxdma:ident: $ch:ty,)+
        }),) => {

        impl<SCKPIN, MISOPIN, MOSIPIN> Spi<$SPIi, SCKPIN, MISOPIN, MOSIPIN, u8> {
            /// Use SPI instance in master mode with DMA on [Ch]
            pub fn with_tx_dma<DMA: DmaExt, const C: u8>(self, mut channel: Ch<DMA, C>) ->
                SpiTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, Ch<DMA, C>> {
                // turn on syscfg clock and set mux
                unsafe { pac::SYSCFG::enable(&*pac::RCC::ptr()) };
                channel.set_map($dmamux);
                self.spi.cr2.modify(|_, w| w.txdmaen().set_bit());
                SpiTxDma {
                    payload: self,
                    channel,
                }
            }
        }
        impl<SCKPIN, MISOPIN, MOSIPIN> SpiSlave<$SPIi, SCKPIN, MISOPIN, MOSIPIN, u8> {
            /// Use SPI instance in slave mode with DMA on [Ch]
            pub fn with_tx_dma<DMA: DmaExt, const C: u8>(self, mut channel: Ch<DMA, C>) ->
                SpiSlaveTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, Ch<DMA, C>> {
                // turn on syscfg clock and set mux
                unsafe { pac::SYSCFG::enable(&*pac::RCC::ptr()) };
                channel.set_map($dmamux);
                self.spi.cr2.modify(|_, w| w.txdmaen().set_bit());
                SpiSlaveTxDma {
                    payload: self,
                    channel,
                }
            }
        }

        $(
            /// Alias for SPI transmit instance in master mode with DMA
            pub type $txdma<SCKPIN, MISOPIN, MOSIPIN> = SpiTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $ch>;

            impl<SCKPIN, MISOPIN, MOSIPIN> Transmit for SpiTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $ch> {
                type TxChannel = $ch;
                type ReceivedWord = u8;
            }

            impl<SCKPIN, MISOPIN, MOSIPIN> SpiTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $ch> {
                /// Release the SPI instance, pins, and DMA channel
                pub fn release(mut self) -> (Spi<$SPIi, SCKPIN, MISOPIN, MOSIPIN, u8>, $ch) {
                    self.stop();
                    // reset the mux to default
                    self.channel.set_map(PeriphMap::Adc);
                    let SpiTxDma { payload, channel } = self;
                    payload.spi.cr2.modify(|_, w| w.rxdmaen().clear_bit());
                    (payload, channel)
                }
            }

            impl<SCKPIN, MISOPIN, MOSIPIN> TransferPayload for SpiTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $ch> {
                fn start(&mut self) {
                    self.channel.start();
                }
                fn stop(&mut self) {
                    self.channel.stop();
                }
            }

            impl<B, SCKPIN, MISOPIN, MOSIPIN> WriteDma<B, u8> for SpiTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $ch>
            where
                B: ReadBuffer<Word = u8>,
            {
                fn write(mut self, buffer: B) -> Transfer<dma::R, B, Self> {
                    // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                    // until the end of the transfer.
                    let (ptr, len) = unsafe { buffer.read_buffer() };
                    self.channel.set_peripheral_address(
                        unsafe { (*<$SPIi>::ptr()).dr().as_ptr() as u32 },
                        false,
                    );
                    self.channel.set_memory_address(ptr as u32, true);
                    self.channel.set_transfer_length(len);

                    atomic::compiler_fence(Ordering::Release);
                    self.channel.ch().cr.modify(|_, w| {
                        // memory to memory mode disabled
                        w.mem2mem().clear_bit();
                        // medium channel priority level
                        w.pl().medium();
                        // 8-bit memory size
                        w.msize().bits8();
                        // 8-bit peripheral size
                        w.psize().bits8();
                        // circular mode disabled
                        w.circ().clear_bit();
                        // write to memory
                        w.dir().clear_bit()
                    });
                    self.start();

                    Transfer::r(buffer, self)
                }
            }

            /// Alias for SPI instance in slave mode with DMA
            pub type $slavetxdma<SCKPIN, MISOPIN, MOSIPIN> =
                SpiSlaveTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $ch>;


            impl<SCKPIN, MISOPIN, MOSIPIN> Transmit for SpiSlaveTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $ch> {
                type TxChannel = $ch;
                type ReceivedWord = u8;
            }

            impl<SCKPIN, MISOPIN, MOSIPIN> SpiSlaveTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $ch> {
                /// Release the SPI instance, pins and DMA channel
                pub fn release(mut self) -> (SpiSlave<$SPIi, SCKPIN, MISOPIN, MOSIPIN, u8>, $ch) {
                    self.stop();
                    // reset the mux to default
                    self.channel.set_map(PeriphMap::Adc);
                    let SpiSlaveTxDma { payload, channel } = self;
                    payload.spi.cr2.modify(|_, w| w.txdmaen().clear_bit());
                    (payload, channel)
                }
            }

            impl<SCKPIN, MISOPIN, MOSIPIN> TransferPayload for SpiSlaveTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $ch> {
                fn start(&mut self) {
                    self.channel.start();
                }
                fn stop(&mut self) {
                    self.channel.stop();
                }
            }

            impl<B, SCKPIN, MISOPIN, MOSIPIN> WriteDma<B, u8> for SpiSlaveTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $ch>
            where
                B: ReadBuffer<Word = u8>,
            {
                fn write(mut self, buffer: B) -> Transfer<dma::R, B, Self> {
                    // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                    // until the end of the transfer.
                    let (ptr, len) = unsafe { buffer.read_buffer() };
                    self.channel.set_peripheral_address(
                        unsafe { (*<$SPIi>::ptr()).dr().as_ptr() as u32 },
                        false,
                    );
                    self.channel.set_memory_address(ptr as u32, true);
                    self.channel.set_transfer_length(len);

                    atomic::compiler_fence(Ordering::Release);
                    self.channel.ch().cr.modify(|_, w| {
                        // memory to memory mode disabled
                        w.mem2mem().clear_bit();
                        // medium channel priority level
                        w.pl().medium();
                        // 8-bit memory size
                        w.msize().bits8();
                        // 8-bit peripheral size
                        w.psize().bits8();
                        // circular mode disabled
                        w.circ().clear_bit();
                        // read from memory
                        w.dir().set_bit()
                    });
                    self.start();

                    Transfer::r(buffer, self)
                }
            }

        )+

    };
}

#[cfg(any(feature = "py32f003", feature = "py32f030"))]
spi_dmatx! {
    pac::SPI1: (PeriphMap::Spi1Tx, {
        Spi1TxDma1, SpiSlave1TxDma1: dma1::C1,
        Spi1TxDma2, SpiSlave1TxDma2: dma1::C2,
        Spi1TxDma3, SpiSlave1TxDma3: dma1::C3,
    }),
}
#[cfg(any(feature = "py32f030"))]
spi_dmatx! {
    pac::SPI2: (PeriphMap::Spi2Tx, {
        Spi2TxDma1, SpiSlave2TxDma1: dma1::C1,
        Spi2TxDma2, SpiSlave2TxDma2: dma1::C2,
        Spi2TxDma3, SpiSlave2TxDma3: dma1::C3,
    }),
}

macro_rules! spi_dmarxtx {
    (
        $SPIi:ty: ($dmarxmux:expr, $dmatxmux:expr, {
            $($rxtxdma:ident, $slaverxtxdma:ident, ($rxch:ty, $txch:ty),)+
        }),) => {
        impl<SCKPIN, MISOPIN, MOSIPIN> Spi<$SPIi, SCKPIN, MISOPIN, MOSIPIN, u8> {
            /// Use SPI instance in master mode with DMA on [Ch]
            pub fn with_rx_tx_dma<DMA: DmaExt, const CRX: u8, const CTX: u8>(
                self,
                mut rxchannel: Ch<DMA, CRX>,
                mut txchannel: Ch<DMA, CTX>,
            ) -> SpiRxTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, Ch<DMA, CRX>, Ch<DMA, CTX>> {
                // turn on syscfg clock and set mux
                unsafe { pac::SYSCFG::enable(&*pac::RCC::ptr()) };
                rxchannel.set_map($dmarxmux);
                txchannel.set_map($dmatxmux);
                self.spi.cr2.modify(|_, w| {
                    w.rxdmaen().set_bit();
                    w.txdmaen().set_bit()
                });
                SpiRxTxDma {
                    payload: self,
                    rxchannel,
                    txchannel,
                }
            }
        }
        impl<SCKPIN, MISOPIN, MOSIPIN> SpiSlave<$SPIi, SCKPIN, MISOPIN, MOSIPIN, u8> {
            /// Use SPI instance in slave mode with DMA on [Ch]
            pub fn with_rx_tx_dma<DMA: DmaExt, const CRX: u8, const CTX: u8>(
                self,
                mut rxchannel: Ch<DMA, CRX>,
                mut txchannel: Ch<DMA, CTX>,
            ) -> SpiSlaveRxTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, Ch<DMA, CRX>, Ch<DMA, CTX>> {
                // turn on syscfg clock and set mux
                unsafe { pac::SYSCFG::enable(&*pac::RCC::ptr()) };
                rxchannel.set_map($dmarxmux);
                txchannel.set_map($dmatxmux);
                self.spi.cr2.modify(|_, w| {
                    w.rxdmaen().set_bit();
                    w.txdmaen().set_bit()
                });
                SpiSlaveRxTxDma {
                    payload: self,
                    rxchannel,
                    txchannel,
                }
            }
        }

        $(
            /// Alias for SPI tx/rx instance in master mode with DMA
            pub type $rxtxdma<SCKPIN, MISOPIN, MOSIPIN> = SpiRxTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $rxch, $txch>;

            impl<SCKPIN, MISOPIN, MOSIPIN> Transmit for SpiRxTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $rxch, $txch> {
                type TxChannel = $txch;
                type ReceivedWord = u8;
            }

            impl<SCKPIN, MISOPIN, MOSIPIN> Receive for SpiRxTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $rxch, $txch> {
                type RxChannel = $rxch;
                type TransmittedWord = u8;
            }

            impl<SCKPIN, MISOPIN, MOSIPIN> SpiRxTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $rxch, $txch> {
                /// Release the SPI instance, pins, and DMA channels
                pub fn release(mut self) -> (Spi<$SPIi, SCKPIN, MISOPIN, MOSIPIN, u8>, $rxch, $txch) {
                    self.stop();
                    // reset the mux to default
                    self.rxchannel.set_map(PeriphMap::Adc);
                    self.txchannel.set_map(PeriphMap::Adc);
                    let SpiRxTxDma { payload, rxchannel, txchannel } = self;
                    payload.spi.cr2.modify(|_, w| {w.rxdmaen().clear_bit(); w.txdmaen().clear_bit()});
                    (payload, rxchannel, txchannel)
                }
            }

            impl<SCKPIN, MISOPIN, MOSIPIN> TransferPayload for SpiRxTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $rxch, $txch> {
                fn start(&mut self) {
                    self.rxchannel.start();
                    self.txchannel.start();
                }
                fn stop(&mut self) {
                    self.rxchannel.stop();
                    self.txchannel.stop();
                }
            }

            impl<RXB, TXB, SCKPIN, MISOPIN, MOSIPIN> ReadWriteDma<RXB, TXB, u8> for SpiRxTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $rxch, $txch>
            where
                RXB: WriteBuffer<Word = u8>,
                TXB: ReadBuffer<Word = u8>,
            {
                fn read_write(mut self, mut rxbuffer: RXB, txbuffer: TXB) -> Transfer<dma::W, (RXB, TXB), Self> {
                    // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                    // until the end of the transfer.
                    let (rxptr, rxlen) = unsafe { rxbuffer.write_buffer() };
                    let (txptr, txlen) = unsafe { txbuffer.read_buffer() };

                    if rxlen != txlen {
                        panic!("receive and send buffer lengths do not match!");
                    }

                    self.rxchannel.set_peripheral_address(
                        unsafe { (*<$SPIi>::ptr()).dr().as_ptr() as u32 },
                        false,
                    );
                    self.rxchannel.set_memory_address(rxptr as u32, true);
                    self.rxchannel.set_transfer_length(rxlen);

                    self.txchannel.set_peripheral_address(
                        unsafe { (*<$SPIi>::ptr()).dr().as_ptr() as u32 },
                        false,
                    );
                    self.txchannel.set_memory_address(txptr as u32, true);
                    self.txchannel.set_transfer_length(txlen);

                    atomic::compiler_fence(Ordering::Release);
                    self.rxchannel.ch().cr.modify(|_, w| {
                        // memory to memory mode disabled
                        w.mem2mem().clear_bit();
                        // medium channel priority level
                        w.pl().medium();
                        // 8-bit memory size
                        w.msize().bits8();
                        // 8-bit peripheral size
                        w.psize().bits8();
                        // circular mode disabled
                        w.circ().clear_bit();
                        // write to memory
                        w.dir().clear_bit()
                    });
                    self.txchannel.ch().cr.modify(|_, w| {
                        // memory to memory mode disabled
                        w.mem2mem().clear_bit();
                        // medium channel priority level
                        w.pl().medium();
                        // 8-bit memory size
                        w.msize().bits8();
                        // 8-bit peripheral size
                        w.psize().bits8();
                        // circular mode disabled
                        w.circ().clear_bit();
                        // read from memory
                        w.dir().set_bit()
                    });
                    self.start();

                    Transfer::w((rxbuffer, txbuffer), self)
                }
            }

            /// Alias for SPI rx/tx instance in slave mode with DMA
            pub type $slaverxtxdma<SCKPIN, MISOPIN, MOSIPIN> =
                SpiSlaveRxTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $rxch, $txch>;

            impl<SCKPIN, MISOPIN, MOSIPIN> Receive for SpiSlaveRxTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $rxch, $txch> {
                type RxChannel = $rxch;
                type TransmittedWord = u8;
            }

            impl<SCKPIN, MISOPIN, MOSIPIN> Transmit for SpiSlaveRxTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $rxch, $txch> {
                type TxChannel = $txch;
                type ReceivedWord = u8;
            }

            impl<SCKPIN, MISOPIN, MOSIPIN> SpiSlaveRxTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $rxch, $txch> {
                /// Release the SPI instance, pins, and DMA channels
                pub fn release(mut self) -> (SpiSlave<$SPIi, SCKPIN, MISOPIN, MOSIPIN, u8>, $rxch, $txch) {
                    self.stop();
                    // reset the mux to default
                    self.rxchannel.set_map(PeriphMap::Adc);
                    self.txchannel.set_map(PeriphMap::Adc);
                    let SpiSlaveRxTxDma { payload, rxchannel, txchannel } = self;
                    payload.spi.cr2.modify(|_, w| {w.rxdmaen().clear_bit(); w.txdmaen().clear_bit()});
                    (payload, rxchannel, txchannel)
                }
            }

            impl<SCKPIN, MISOPIN, MOSIPIN> TransferPayload for SpiSlaveRxTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $rxch, $txch> {
                fn start(&mut self) {
                    self.rxchannel.start();
                    self.txchannel.start();
                }
                fn stop(&mut self) {
                    self.rxchannel.stop();
                    self.txchannel.stop();
                }
            }

            impl<RXB, TXB, SCKPIN, MISOPIN, MOSIPIN> ReadWriteDma<RXB, TXB, u8> for SpiSlaveRxTxDma<$SPIi, SCKPIN, MISOPIN, MOSIPIN, $rxch, $txch>
            where
                RXB: WriteBuffer<Word = u8>,
                TXB: ReadBuffer<Word = u8>,
            {
                fn read_write(mut self, mut rxbuffer: RXB, txbuffer: TXB) -> Transfer<dma::W, (RXB, TXB), Self> {
                    // NOTE(unsafe) We own the buffer now and we won't call other `&mut` on it
                    // until the end of the transfer.
                    let (rxptr, rxlen) = unsafe { rxbuffer.write_buffer() };
                    let (txptr, txlen) = unsafe { txbuffer.read_buffer() };

                    if rxlen != txlen {
                        panic!("receive and send buffer lengths do not match!");
                    }

                    self.rxchannel.set_peripheral_address(
                        unsafe { (*<$SPIi>::ptr()).dr().as_ptr() as u32 },
                        false,
                    );
                    self.rxchannel.set_memory_address(rxptr as u32, true);
                    self.rxchannel.set_transfer_length(rxlen);
                    self.txchannel.set_peripheral_address(
                        unsafe { (*<$SPIi>::ptr()).dr().as_ptr() as u32 },
                        false,
                    );
                    self.txchannel.set_memory_address(txptr as u32, true);
                    self.txchannel.set_transfer_length(txlen);

                    atomic::compiler_fence(Ordering::Release);
                    self.rxchannel.ch().cr.modify(|_, w| {
                        // memory to memory mode disabled
                        w.mem2mem().clear_bit();
                        // medium channel priority level
                        w.pl().medium();
                        // 8-bit memory size
                        w.msize().bits8();
                        // 8-bit peripheral size
                        w.psize().bits8();
                        // circular mode disabled
                        w.circ().clear_bit();
                        // write to memory
                        w.dir().clear_bit()
                    });
                    self.txchannel.ch().cr.modify(|_, w| {
                        // memory to memory mode disabled
                        w.mem2mem().clear_bit();
                        // medium channel priority level
                        w.pl().medium();
                        // 8-bit memory size
                        w.msize().bits8();
                        // 8-bit peripheral size
                        w.psize().bits8();
                        // circular mode disabled
                        w.circ().clear_bit();
                        // read from memory
                        w.dir().set_bit()
                    });
                    self.start();

                    Transfer::w((rxbuffer, txbuffer), self)
                }
            }

        )+
    };
}

#[cfg(any(feature = "py32f003", feature = "py32f030"))]
spi_dmarxtx! {
    pac::SPI1: (PeriphMap::Spi1Rx, PeriphMap::Spi1Tx, {
        Spi1RxTxDma12, SpiSlave1RxTxDma12, (dma1::C1, dma1::C2),
        Spi1RxTxDma23, SpiSlave1RxTxDma23, (dma1::C2, dma1::C3),
        Spi1RxTxDma13, SpiSlave1RxTxDma13, (dma1::C1, dma1::C3),
    }),
}
#[cfg(any(feature = "py32f030"))]
spi_dmarxtx! {
    pac::SPI2: (PeriphMap::Spi2Rx, PeriphMap::Spi2Tx, {
        Spi2RxTxDma12, SpiSlave2RxTxDma12, (dma1::C1, dma1::C2),
        Spi2RxTxDma23, SpiSlave2RxTxDma23, (dma1::C2, dma1::C3),
        Spi2RxTxDma13, SpiSlave2RxTxDma13, (dma1::C1, dma1::C3),
    }),
}
