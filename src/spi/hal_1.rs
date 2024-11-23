use super::*;
pub use embedded_hal::spi::{ErrorKind, ErrorType, Mode, Phase, Polarity};

impl From<Polarity> for super::Polarity {
    fn from(p: Polarity) -> Self {
        match p {
            Polarity::IdleLow => Self::IdleLow,
            Polarity::IdleHigh => Self::IdleHigh,
        }
    }
}

impl From<Phase> for super::Phase {
    fn from(p: Phase) -> Self {
        match p {
            Phase::CaptureOnFirstTransition => Self::CaptureOnFirstTransition,
            Phase::CaptureOnSecondTransition => Self::CaptureOnSecondTransition,
        }
    }
}

impl From<Mode> for super::Mode {
    fn from(m: Mode) -> Self {
        Self {
            polarity: m.polarity.into(),
            phase: m.phase.into(),
        }
    }
}

impl embedded_hal::spi::Error for Error {
    fn kind(&self) -> ErrorKind {
        match self {
            Self::Overrun => ErrorKind::Overrun,
            Self::ModeFault => ErrorKind::ModeFault,
        }
    }
}

impl<SPI: Instance, SCKPIN, MISOPIN, MOSIPIN, WIDTH> ErrorType
    for Spi<SPI, SCKPIN, MISOPIN, MOSIPIN, WIDTH>
{
    type Error = Error;
}

mod nb {
    use super::{Error, Instance, Spi};
    use embedded_hal_nb::spi::FullDuplex;

    impl<SPI, SCKPIN, MISOPIN, MOSIPIN, WIDTH> FullDuplex<WIDTH>
        for Spi<SPI, SCKPIN, MISOPIN, MOSIPIN, WIDTH>
    where
        SPI: Instance,
        WIDTH: Copy,
    {
        fn read(&mut self) -> nb::Result<WIDTH, Error> {
            self.read_nonblocking()
        }

        fn write(&mut self, data: WIDTH) -> nb::Result<(), Error> {
            self.write_nonblocking(data)
        }
    }
}

mod blocking {
    use super::super::{Instance, Spi, SpiReadWrite};
    use embedded_hal::spi::SpiBus;

    impl<SPI, SCKPIN, MISOPIN, MOSIPIN, WIDTH> SpiBus<WIDTH>
        for Spi<SPI, SCKPIN, MISOPIN, MOSIPIN, WIDTH>
    where
        SPI: Instance,
        WIDTH: Copy + 'static,
    {
        fn transfer_in_place(&mut self, _words: &mut [WIDTH]) -> Result<(), Self::Error> {
            todo!()
        }

        fn transfer(&mut self, _buff: &mut [WIDTH], _data: &[WIDTH]) -> Result<(), Self::Error> {
            todo!()
        }

        fn read(&mut self, _words: &mut [WIDTH]) -> Result<(), Self::Error> {
            todo!()
        }

        fn write(&mut self, words: &[WIDTH]) -> Result<(), Self::Error> {
            self.spi_write(words)
        }

        fn flush(&mut self) -> Result<(), Self::Error> {
            Ok(())
        }
    }
}
