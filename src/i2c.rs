//! API for the integrated I2C peripheral
use core::ops::Deref;

use embedded_hal_02::blocking::i2c::{Read, Write, WriteRead};

use crate::{
    gpio::*,
    pac,
    rcc::{Clocks, Enable, Reset},
    time::{kHz, Hertz, KiloHertz},
};

/// I2C abstraction
pub struct I2c<I2C: Instance, SCLPIN, SDAPIN> {
    i2c: I2C,
    pins: (SCLPIN, SDAPIN),
}

/// Trait for identifying SCL pins
pub trait SclPin<I2C> {}
/// Trait for identifying SDA pins
pub trait SdaPin<I2C> {}

macro_rules! i2c_pins {
    ($($I2C:ident => {
        scl => [$($scl:ty),+ $(,)*],
        sda => [$($sda:ty),+ $(,)*],
    })+) => {
        $(
            $(
                impl SclPin<crate::pac::$I2C> for $scl {}
            )+
            $(
                impl SdaPin<crate::pac::$I2C> for $sda {}
            )+
        )+
    }
}

#[cfg(feature = "py32f030")]
i2c_pins! {
    I2C => {
        scl => [
            gpioa::PA3<Alternate<AF12>>,
            gpioa::PA8<Alternate<AF12>>,
            gpioa::PA9<Alternate<AF6>>,
            gpioa::PA10<Alternate<AF12>>,
            gpioa::PA11<Alternate<AF6>>,
            gpiob::PB6<Alternate<AF6>>,
            gpiob::PB8<Alternate<AF6>>,
            gpiof::PF1<Alternate<AF12>>
        ],
        sda => [
            gpioa::PA2<Alternate<AF12>>,
            gpioa::PA7<Alternate<AF12>>,
            gpioa::PA9<Alternate<AF12>>,
            gpioa::PA10<Alternate<AF6>>,
            gpioa::PA12<Alternate<AF6>>,
            gpiob::PB7<Alternate<AF6>>,
            gpiof::PF0<Alternate<AF12>>
        ],
    }
}

#[cfg(feature = "py32f002a")]
i2c_pins! {
    I2C => {
        scl => [
            gpioa::PA3<Alternate<AF12>>,
            gpioa::PA8<Alternate<AF12>>,
            gpioa::PA9<Alternate<AF6>>,
            gpioa::PA10<Alternate<AF12>>,
            gpiob::PB6<Alternate<AF6>>,
            gpiob::PB8<Alternate<AF6>>,
            gpiof::PF1<Alternate<AF12>>
        ],
        sda => [
            gpioa::PA2<Alternate<AF12>>,
            gpioa::PA7<Alternate<AF12>>,
            gpioa::PA9<Alternate<AF12>>,
            gpioa::PA10<Alternate<AF6>>,
            gpioa::PA12<Alternate<AF6>>,
            gpiob::PB7<Alternate<AF6>>,
            gpiof::PF0<Alternate<AF12>>
        ],
    }
}

#[cfg(feature = "py32f003")]
i2c_pins! {
    I2C => {
        scl => [
            gpioa::PA3<Alternate<AF12>>,
            gpiob::PB6<Alternate<AF6>>,
            gpiof::PF1<Alternate<AF12>>
        ],
        sda => [
            gpioa::PA2<Alternate<AF12>>,
            gpioa::PA7<Alternate<AF12>>,
            gpioa::PA12<Alternate<AF6>>,
            gpiob::PB7<Alternate<AF6>>,
            gpiof::PF0<Alternate<AF12>>
        ],
    }
}

#[cfg(feature = "py32f002b")]
i2c_pins! {
    I2C => {
        scl => [
            gpioa::PA2<Alternate<AF6>>,
            gpiob::PB3<Alternate<AF6>>,
        ],
        sda => [
            gpiob::PB4<Alternate<AF6>>,
            gpiob::PB6<Alternate<AF6>>,
        ],
    }
}

/// Error enum for I2C peripheral
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Error {
    /// data has been overwritten before being read
    OVERRUN,
    /// An acknowledge error occurs when the interface detects a no acknowledge bit
    NACK,
    /// A bus error is generated when the I2C interface detects an
    /// external stop or start condition during an address or data
    /// byte transfer
    BUS,
    /// Packet error check error
    PEC,
}

// It's s needed for the impls, but rustc doesn't recognize that
#[allow(dead_code)]
type I2cRegisterBlock = crate::pac::i2c::RegisterBlock;

/// trait for I2C peripheral instance
pub trait Instance: Deref<Target = I2cRegisterBlock> + crate::Sealed + Enable + Reset {
    #[doc(hidden)]
    fn ptr() -> *const I2cRegisterBlock;
}

macro_rules! i2c {
    ($($I2C:ident: $i2c:ident,)+) => {
        $(
            use crate::pac::$I2C;
            impl Instance for $I2C {
                fn ptr() -> *const I2cRegisterBlock {
                    <$I2C>::ptr()
                }
            }

            impl<SCLPIN, SDAPIN> I2c<$I2C, SCLPIN, SDAPIN> {
                /// Create an instance of I2C peripheral
                pub fn $i2c(i2c: $I2C, pins: (SCLPIN, SDAPIN), speed: KiloHertz, clocks: &Clocks) -> Self
                where
                    SCLPIN: SclPin<$I2C>,
                    SDAPIN: SdaPin<$I2C>,
                {
                    let rcc = unsafe { &(*pac::RCC::ptr()) };
                    // Enable clock for I2C
                    $I2C::enable(rcc);

                    // Reset I2C
                    $I2C::reset(rcc);
                    I2c { i2c, pins }.i2c_init(clocks.pclk(), speed)
                }
            }
        )+
    }
}

i2c! {
    I2C: i2c,
}

impl<I2C, SCLPIN, SDAPIN> I2c<I2C, SCLPIN, SDAPIN>
where
    I2C: Instance,
{
    fn i2c_init(self, freq: Hertz, speed: KiloHertz) -> Self {
        // Make sure the I2C unit is disabled so we can configure it
        self.i2c.cr1.modify(|_, w| w.pe().clear_bit());

        let clc_mhz = freq.raw() / 1_000_000;

        let scl_rate_hz = speed.raw() * 1000; // kHz to Hz

        self.i2c
            .cr2
            .write(|w| unsafe { w.freq().bits(clc_mhz.clamp(4, 48) as u8) });

        // Normal I2C speeds use a different scaling than fast mode below
        let (f_s, ccr) = if speed <= kHz(100) {
            // This is a normal I2C mode
            (false, freq.raw() / (scl_rate_hz * 2))
        } else {
            // This is a fast I2C mode
            (
                true,
                if self.i2c.ccr.read().duty().bit_is_set() {
                    freq.raw() / (scl_rate_hz * 25)
                } else {
                    freq.raw() / (scl_rate_hz * 3)
                },
            )
        };

        self.i2c
            .ccr
            .modify(|_, w| unsafe { w.f_s().bit(f_s).ccr().bits(ccr.clamp(4, 4095) as u16) });

        // trise
        let trise = if scl_rate_hz <= 100_000 {
            clc_mhz + 1
        } else {
            clc_mhz * 300 / 1000 + 1
        };

        // Configure correct rise times
        let trise_clamped = trise.clamp(1, 63) as u8;
        self.i2c.trise.write(|w| w.trise().bits(trise_clamped));

        // Enable the I2C processing
        self.i2c.cr1.modify(|_, w| w.pe().set_bit());

        self
    }

    /// Release the I2C instance
    pub fn release(self) -> (I2C, (SCLPIN, SDAPIN)) {
        (self.i2c, self.pins)
    }

    fn check_and_clear_error_flags(&self) -> Result<crate::pac::i2c::sr1::R, Error> {
        let sr = self.i2c.sr1.read();
        // If we have a set pec error flag, clear it and return an PEC error
        if sr.pecerr().bit_is_set() {
            self.i2c.sr1.write(|w| w.pecerr().clear_bit());
            return Err(Error::PEC);
        }

        // If we have a set overrun flag, clear it and return an OVERRUN error
        if sr.ovr().bit_is_set() {
            self.i2c.sr1.write(|w| w.ovr().clear_bit());
            return Err(Error::OVERRUN);
        }

        // If we have a set arbitration error or bus error flag, clear it and return an BUS error
        if sr.arlo().bit_is_set() | sr.berr().bit_is_set() {
            self.i2c
                .sr1
                .write(|w| w.arlo().clear_bit().berr().clear_bit());
            return Err(Error::BUS);
        }

        // If we received a NACK, then signal as a NACK error
        if sr.af().bit_is_set() {
            self.i2c.sr1.write(|w| w.af().clear_bit());
            return Err(Error::NACK);
        }

        Ok(sr)
    }

    #[inline(always)]
    fn prepare(&mut self, addr: u8, is_write: bool) -> Result<(), Error> {
        // Wait until a previous STOP condition finishes. When the previous
        // STOP was generated inside an ISR (e.g. DMA interrupt handler),
        // the ISR returns without waiting for the STOP condition to finish.
        // It is possible that the STOP condition is still being generated
        // when we reach here, so we wait until it finishes before proceeding
        // to start a new transaction.
        loop {
            self.check_and_clear_error_flags()?;
            if self.i2c.cr1.read().stop().bit_is_clear() {
                break;
            }
        } 

        // Clear all pending error bits
        self.i2c.sr1.write(|w| unsafe { w.bits(0) });
        
        // Set up current address, we're trying to talk to
        let wr_addr = if is_write {
            u32::from(addr) << 1
        } else {
            u32::from(addr) << 1 | 1
        };

        if is_write {
            // Send a START condition
            self.i2c.cr1.modify(|_, w| w.start().set_bit());
        } else {
            // Send a START condition
            // Set ACK bit for read operations
            self.i2c.cr1.modify(|_, w| w.start().set_bit().ack().set_bit());
        }

        // Wait until START condition was generated
        while self.check_and_clear_error_flags()?.sb().bit_is_clear() { }

        // Also wait until signalled we're master and everything is waiting for us
        loop {
            self.check_and_clear_error_flags()?;

            let sr2 = self.i2c.sr2.read();
            if !(sr2.msl().bit_is_clear() && sr2.busy().bit_is_clear()) {
                break;
            }
        }

        // Send out address         
        self.i2c
            .dr
            .write(|w| unsafe { w.bits(wr_addr) });

        // Wait until address was sent
        while self.check_and_clear_error_flags()?.addr().bit_is_clear() { }
        
        // Clear condition by reading SR2
        self.i2c.sr2.read();

        Ok(())
    }

    fn send_byte(&self, byte: u8) -> Result<(), Error> {
        // Wait until we're ready for sending
        // Check for any I2C errors. If a NACK occurs, the ADDR bit will never be set.
        while self.check_and_clear_error_flags()?.txe().bit_is_clear() { }
        
        // Push out a byte of data
        self.i2c.dr.write(|w| unsafe { w.bits(u32::from(byte)) });

        // Wait until byte is transferred
        // Check for any potential error conditions.
        while self.check_and_clear_error_flags()?.btf().bit_is_clear() { }

        Ok(())
    }

    fn recv_byte(&self) -> Result<u8, Error> {
        while self.check_and_clear_error_flags()?.rxne().bit_is_clear() { }        
        let value = self.i2c.dr.read().bits() as u8;
        Ok(value)
    }

    
    fn read_bytes(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        // Receive bytes into buffer
        for c in buffer {
            *c = self.recv_byte()?;
        }

        Ok(())
    }
    /// Reads like normal but does'n generate start and don't send address
    fn read_wo_prepare(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        if let Some((last, buffer)) = buffer.split_last_mut() {
            // Read all bytes but not last
            self.read_bytes(buffer)?;

            // Prepare to send NACK then STOP after next byte
            self.i2c
                .cr1
                .modify(|_, w| w.ack().clear_bit().stop().set_bit());

            // Receive last byte
            *last = self.recv_byte()?;

            
            // Wait for the STOP to be sent. Otherwise, the interface will still be
            // busy for a while after this function returns. Immediate following
            // operations through the DMA handle might thus encounter `WouldBlock`
            // error. Instead, we should make sure that the interface becomes idle
            // before returning.
            loop {
                self.check_and_clear_error_flags()?;
                if self.i2c.cr1.read().stop().bit_is_clear() {
                    break;
                }
            }
            

            // Fallthrough is success
            Ok(())
        } else {
            Err(Error::OVERRUN)
        }
    }

    fn write_bytes(&mut self, bytes: &[u8]) -> Result<(), Error> {
        // Send bytes
        for c in bytes {
            self.send_byte(*c)?;
        }

        // Fallthrough is success
        Ok(())
    }
    /// Writes like normal but does'n generate start and don't send address
    fn write_wo_prepare(&mut self, bytes: &[u8]) -> Result<(), Error> {
        self.write_bytes(bytes)?;

        // Send a STOP condition
        self.i2c.cr1.modify(|_, w| w.stop().set_bit());

        // Wait for the STOP to be sent. Otherwise, the interface will still be
        // busy for a while after this function returns. Immediate following
        // operations through the DMA handle might thus encounter `WouldBlock`
        // error. Instead, we should make sure that the interface becomes idle
        // before returning.
        while self.i2c.cr1.read().stop().bit_is_set() {}

        // Fallthrough is success
        Ok(())
    }
}



impl<I2C, SCLPIN, SDAPIN> WriteRead for I2c<I2C, SCLPIN, SDAPIN>
where
    I2C: Instance,
{
    type Error = Error;

    
    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
        // Prepare for write
        self.prepare(addr, true)?;

        // Send out all individual bytes
        for c in bytes {
            self.send_byte(*c)?;
        }

        // Prepare for read
        self.prepare(addr, false)?;
        
        self.read_wo_prepare(buffer)?;
        
        Ok(())
    }
}

impl<I2C, SCLPIN, SDAPIN> Read for I2c<I2C, SCLPIN, SDAPIN>
where
    I2C: Instance,
{
    type Error = Error;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        // Prepare for read
        self.prepare(addr, false)?;

        // Now read in all bytes
        self.read_wo_prepare(buffer)?;

        // Check and clear flags if they somehow ended up set
        self.check_and_clear_error_flags()?;

        Ok(())
    }
}

impl<I2C, SCLPIN, SDAPIN> Write for I2c<I2C, SCLPIN, SDAPIN>
where
    I2C: Instance,
{
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        // Prepare for write
        self.prepare(addr, true)?;
        
        // Now write all bytes
        self.write_wo_prepare(bytes)?;

        // Check and clear flags if they somehow ended up set
        self.check_and_clear_error_flags()?;

        Ok(())
    }
}
