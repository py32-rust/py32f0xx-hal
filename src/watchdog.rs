//! API for the IWDG (Watchdog)
//!
//! You can activate the watchdog by calling `start` or by setting the appropriate
//! device option bit when programming.
//!
//! After activating the watchdog, you'll have to regularly `feed` the watchdog.
//! If more time than `timeout` has gone by since the last `feed`, your
//! microcontroller will be reset.
//!
//! This is useful if you fear that your program may get stuck. In that case it
//! won't feed the watchdog anymore, the watchdog will reset the microcontroller
//! and thus your program will function again.
//!
//! **Attention**:
//!
//! The IWDG runs on a separate 40kHz low-accuracy clock (30kHz-60kHz). You may
//! want to some buffer in your interval.
//!
//! Per default the iwdg continues to run even when you stopped execution of code via a debugger.
//! You may want to disable the watchdog when the cpu is stopped
//!
//! ``` ignore
//! let dbgmcu = p.DBGMCU;
//! dbgmcu.apb1_fz.modify(|_, w| w.dbg_iwdg_stop().set_bit());
//! ```
//!
//! # Example
//! ``` no_run
//! use py32f0xx_hal as hal;
//!
//! use crate::hal::pac;
//! use crate::hal::prelude::*;
//! use crate::hal:watchdog::Watchdog;
//!
//! let mut p = pac::Peripherals::take().unwrap();
//! let mut rcc = p.RCC.configure().sysclk(8.MHz()).freeze(&mut p.FLASH);
//!
//! let mut iwdg = Watchdog::new(&mut rcc, p.iwdg);
//! iwdg.start(100.Hz());
//! loop {}
//! // Whoops, got stuck, the watchdog issues a reset after 10 ms
//! iwdg.feed();
//! ```
use embedded_hal_02::watchdog;

use crate::pac::IWDG;
use crate::rcc::Rcc;
use crate::time::Hertz;

/// Watchdog instance
pub struct Watchdog {
    iwdg: IWDG,
}

impl watchdog::Watchdog for Watchdog {
    /// Feed the watchdog, so that at least one `period` goes by before the next
    /// reset
    fn feed(&mut self) {
        self.iwdg.kr.write(|w| w.key().reset());
    }
}

/// Timeout configuration for the IWDG
#[derive(PartialEq, PartialOrd, Clone, Copy)]
pub struct IwdgTimeout {
    psc: u8,
    reload: u16,
}

impl From<Hertz> for IwdgTimeout {
    /// This converts the value so it's usable by the IWDG
    /// Due to conversion losses, the specified frequency is a maximum
    ///
    /// It can also only represent values < 10000 Hertz
    fn from(hz: Hertz) -> Self {
        let mut time = 32_768 / 4 / hz.raw();
        let mut psc = 0;
        let mut reload = 0;
        while psc < 7 {
            reload = time;
            if reload < 0x1000 {
                break;
            }
            psc += 1;
            time /= 2;
        }
        // As we get an integer value, reload is always below 0xFFF
        let reload = reload as u16;
        IwdgTimeout { psc, reload }
    }
}

impl Watchdog {
    /// Create a new [Watchdog]
    ///
    /// Modifies the RCC registers to turn on LSI clock
    pub fn new(rcc: &mut Rcc, iwdg: IWDG) -> Self {
        rcc.regs.csr.modify(|_, w| w.lsion().on());
        while rcc.regs.csr.read().lsirdy().is_not_ready() {}
        Self { iwdg }
    }
}

impl watchdog::WatchdogEnable for Watchdog {
    type Time = IwdgTimeout;
    fn start<T>(&mut self, period: T)
    where
        T: Into<IwdgTimeout>,
    {
        let time: IwdgTimeout = period.into();
        // Feed the watchdog in case it's already running
        // (Waiting for the registers to update takes sometime)
        self.iwdg.kr.write(|w| w.key().reset());
        // Enable the watchdog
        self.iwdg.kr.write(|w| w.key().start());
        self.iwdg.kr.write(|w| w.key().enable());
        // Wait until it's safe to write to the registers
        while self.iwdg.sr.read().pvu().bit() {}
        self.iwdg.pr.write(|w| w.pr().bits(time.psc));
        while self.iwdg.sr.read().rvu().bit() {}
        self.iwdg.rlr.write(|w| w.rl().bits(time.reload));
        // Wait until the registers are updated before issuing a reset with
        // (potentially false) values
        while self.iwdg.sr.read().bits() != 0 {}
        self.iwdg.kr.write(|w| w.key().reset());
    }
}
