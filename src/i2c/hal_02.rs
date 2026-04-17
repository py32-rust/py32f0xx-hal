use super::*;

pub use embedded_hal_02::blocking::i2c::{Read, Write, WriteRead};

impl<I2C, SCLPIN, SDAPIN> WriteRead for I2c<I2C, SCLPIN, SDAPIN, Master>
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

impl<I2C, SCLPIN, SDAPIN> Read for I2c<I2C, SCLPIN, SDAPIN, Master>
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

impl<I2C, SCLPIN, SDAPIN> Write for I2c<I2C, SCLPIN, SDAPIN, Master>
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
