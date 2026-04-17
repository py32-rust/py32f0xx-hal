use super::*;

pub use embedded_hal::i2c::{self, SevenBitAddress, TenBitAddress, Operation};

impl i2c::Error for Error {
    fn kind(&self) -> i2c::ErrorKind {
        match *self {
            Error::OVERRUN => i2c::ErrorKind::Overrun,
            Error::NACK =>i2c::ErrorKind::NoAcknowledge(i2c::NoAcknowledgeSource::Unknown),
            Error::BUS =>i2c::ErrorKind::Bus,
            Error::PEC => i2c::ErrorKind::Other,
        }
    }
}

impl<I2C, SCLPIN, SDAPIN, MODE> i2c::ErrorType for I2c<I2C, SCLPIN, SDAPIN, MODE> {
    type Error = Error;
}

impl<I2C, SCLPIN, SDAPIN> i2c::I2c<SevenBitAddress> for I2c<I2C, SCLPIN, SDAPIN, Master>
where
    I2C: Instance,
{
    fn transaction(&mut self, address: u8, operations: &mut [Operation<'_>]) -> Result<(), Self::Error> {
        for (i, op) in operations.iter_mut().enumerate() {
            match op {
                Operation::Read(buf) => {
                    if i == 0 {
                        self.prepare(address, false)?;
                    }
                    self.read_wo_prepare(buf)?;
                    self.check_and_clear_error_flags()?;
                },
                Operation::Write(buf) => {
                    if i == 0 {
                        self.prepare(address, true)?;
                    }
                    self.write_wo_prepare(buf)?;
                    self.check_and_clear_error_flags()?;
                },
            }
        }
        Ok(())
    }
}

impl<I2C, SCLPIN, SDAPIN> i2c::I2c<TenBitAddress> for I2c<I2C, SCLPIN, SDAPIN, Master>
where
I2C: Instance,
{
    fn transaction(&mut self, address: u16, operations: &mut [Operation<'_>]) -> Result<(), Self::Error> {
        unimplemented!();
    }
}
