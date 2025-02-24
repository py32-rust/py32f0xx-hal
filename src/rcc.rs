//! System Clocks setup and configuration

use crate::pac::rcc;
use crate::pac::rcc::{
    cfgr::{MCOPRE_A, MCOSEL_A},
    cr::HSIDIV_A,
    icscr::HSI_FS_A,
};

use crate::pac::{DBG, PWR, RCC};
use crate::time::{Hertz, Hz};

mod enable;

/// Extension trait that sets up the `RCC` peripheral
pub trait RccExt {
    /// Configure the clocks of the RCC peripheral
    fn configure(self) -> CFGR;
}

impl RccExt for RCC {
    fn configure(self) -> CFGR {
        CFGR {
            hclk: None,
            pclk: None,
            sysclk: None,
            clock_src: SysClkSource::HSISYS(HSIFreq::Freq24mhz),
            rcc: self,
        }
    }
}

/// Constrained RCC peripheral
pub struct Rcc {
    /// The frozen clock frequencies
    pub clocks: Clocks,
    pub(crate) regs: RCC,
}

impl Rcc {
    /// Configure the Main Clock Output
    /// set the MCO pin output source and prescalar
    pub fn configure_mco(&self, sel: MCOSrc, pre: MCODiv) {
        self.regs
            .cfgr
            .modify(|_, w| w.mcopre().variant(pre.into()).mcosel().variant(sel.into()));
    }
    /// Set the clock debug stop mode
    /// false - fclk and hclk are disabled in stop mode (Identical to after reset)
    /// true - fclk and hclk are not disabled in stop mode and set by hsi
    pub fn debug_stop_mode(&mut self, dbg: DBG, set: bool) {
        dbg.cr.write(|w| {
            if set {
                w.dbg_stop().enabled()
            } else {
                w.dbg_stop().disabled()
            }
        });
    }
}

/// AMBA High-performance Bus (AHB) registers
#[non_exhaustive]
pub struct AHB;

/// Advanced Peripheral Bus (APB) registers
#[non_exhaustive]
pub struct APB;

impl APB {
    /// Set power interface clock (PWREN) bit in RCC_APB1ENR
    pub fn set_pwren() {
        let rcc = unsafe { &*RCC::ptr() };
        PWR::enable(rcc);
    }
    /// Disable debug clock (DBGEN) bit in RCC_APB1ENR
    pub fn disable_dbg() {
        let rcc = unsafe { &*RCC::ptr() };
        DBG::disable(rcc);
    }
}

/// MCO source select
#[derive(Clone, Copy)]
pub enum MCOSrc {
    ///0: No clock
    NoClock = 0,
    ///1: SYSCLK clock selected
    Sysclk = 1,
    ///3: HSI oscillator clock selected
    Hsi = 3,
    ///4: HSE oscillator clock selected
    Hse = 4,
    ///5: PLL clock selected
    Pll = 5,
    ///6: LSI oscillator clock selected
    Lsi = 6,
    #[cfg(feature = "py32f030")]
    ///7: LSE oscillator clock selected
    Lse = 7,
}

impl From<MCOSrc> for MCOSEL_A {
    fn from(s: MCOSrc) -> Self {
        match s {
            MCOSrc::NoClock => MCOSEL_A::NoClock,
            MCOSrc::Sysclk => MCOSEL_A::Sysclk,
            MCOSrc::Hsi => MCOSEL_A::Hsi,
            MCOSrc::Hse => MCOSEL_A::Hse,
            MCOSrc::Pll => MCOSEL_A::Pll,
            MCOSrc::Lsi => MCOSEL_A::Lsi,
            #[cfg(feature = "py32f030")]
            MCOSrc::Lse => MCOSEL_A::Lse,
        }
    }
}

/// MCO prescaler
#[derive(Clone, Copy)]
pub enum MCODiv {
    ///0: No division
    NotDivided = 0,
    ///1: Division by 2
    Div2 = 1,
    ///2: Division by 4
    Div4 = 2,
    ///3: Division by 8
    Div8 = 3,
    ///4: Division by 16
    Div16 = 4,
    ///5: Division by 32
    Div32 = 5,
    ///6: Division by 64
    Div64 = 6,
    ///7: Division by 128
    Div128 = 7,
}

impl From<MCODiv> for MCOPRE_A {
    fn from(d: MCODiv) -> Self {
        match d {
            MCODiv::NotDivided => MCOPRE_A::NotDivided,
            MCODiv::Div2 => MCOPRE_A::Div2,
            MCODiv::Div4 => MCOPRE_A::Div4,
            MCODiv::Div8 => MCOPRE_A::Div8,
            MCODiv::Div16 => MCOPRE_A::Div16,
            MCODiv::Div32 => MCOPRE_A::Div32,
            MCODiv::Div64 => MCOPRE_A::Div64,
            MCODiv::Div128 => MCOPRE_A::Div128,
        }
    }
}

/// HSI divider
#[derive(Clone, Copy)]
pub enum HSIDiv {
    /// Full HSI frequency
    NotDivided = 1,
    /// Divide HSI by 2
    Div2 = 2,
    /// Divide HSI by 4
    Div4 = 4,
    /// Divide HSI by 8
    Div8 = 8,
    /// Divide HSI by 16
    Div16 = 16,
    /// Divide HSI by 32
    Div32 = 32,
    /// Divide HSI by 64
    Div64 = 64,
    /// Divide HSI by 128
    Div128 = 128,
}

impl From<HSIDiv> for HSIDIV_A {
    fn from(d: HSIDiv) -> Self {
        match d {
            HSIDiv::NotDivided => HSIDIV_A::NotDivided,
            HSIDiv::Div2 => HSIDIV_A::Div2,
            HSIDiv::Div4 => HSIDIV_A::Div4,
            HSIDiv::Div8 => HSIDIV_A::Div8,
            HSIDiv::Div16 => HSIDIV_A::Div16,
            HSIDiv::Div32 => HSIDIV_A::Div32,
            HSIDiv::Div64 => HSIDIV_A::Div64,
            HSIDiv::Div128 => HSIDIV_A::Div128,
        }
    }
}

/// HSI Frequency select
#[derive(Clone, Copy)]
pub enum HSIFreq {
    /// HSI frequency < 4 MHz
    Freq4mhz = 0,
    /// HSI frequency < 8 Mhz and > 4 MHz
    Freq8mhz = 1,
    /// HSI frequency < 16 Mhz and > 8 MHz
    Freq16mhz = 2,
    /// HSI frequency < 22 Mhz and > 12 MHz
    Freq22_12mhz = 3, // 22.12 MHz
    /// HSI frequency = 24 Mhz
    Freq24mhz = 4,
}

impl From<HSIFreq> for HSI_FS_A {
    fn from(f: HSIFreq) -> Self {
        match f {
            HSIFreq::Freq4mhz => HSI_FS_A::Freq4mhz,
            HSIFreq::Freq8mhz => HSI_FS_A::Freq8mhz,
            HSIFreq::Freq16mhz => HSI_FS_A::Freq16mhz,
            HSIFreq::Freq22_12mhz => HSI_FS_A::Freq2212mhz,
            HSIFreq::Freq24mhz => HSI_FS_A::Freq24mhz,
        }
    }
}

/// HSE Bypass mode
#[derive(Clone, Copy)]
pub enum HSEBypassMode {
    /// Not bypassed: for crystals
    NotBypassed,
    /// Bypassed: for external clock sources
    Bypassed,
}

/// RCC for F0x0 devices
mod inner {
    use crate::pac::{rcc::cfgr::SW_A, RCC};

    use super::{HSEBypassMode, HSIFreq};

    pub(super) const HSI_DEFAULT: u32 = 24_000_000; // Hz

    #[allow(clippy::upper_case_acronyms)]
    pub(super) enum SysClkSource {
        // PLL(Option<(u32, super::HSEBypassMode)>),
        HSISYS(HSIFreq),

        /// High-speed external clock(freq,bypassed)
        HSE(u32, HSEBypassMode),
        // LSI,
        // LSE(u32),
    }

    fn get_hsi_sel_freq(c_src: &SysClkSource) -> u32 {
        if let SysClkSource::HSISYS(fs) = c_src {
            match fs {
                HSIFreq::Freq4mhz => 4_000_000,
                HSIFreq::Freq8mhz => 8_000_000,
                HSIFreq::Freq16mhz => 16_000_000,
                HSIFreq::Freq22_12mhz => 22_120_000,
                HSIFreq::Freq24mhz => 24_000_000,
            }
        } else {
            4_000_000
        }
    }

    pub(super) fn get_freq(c_src: &SysClkSource) -> u32 {
        // Select clock source based on user input and capability
        // Highest selected frequency source available takes precedent.
        match c_src {
            SysClkSource::HSISYS(_) => {
                let hsi_freq = get_hsi_sel_freq(c_src);
                hsi_freq
            }
            SysClkSource::HSE(freq, _) => *freq,
        }
    }

    #[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002a"))]
    fn hse_enable(rcc: &mut RCC, freq: u32, bypassed: &HSEBypassMode) {
        let freq_bits = match freq {
            f if f >= 4_000_000 && f < 8_000_000 => 0b01,
            f if f >= 8_000_000 && f < 16_000_000 => 0b10,
            f if f >= 16_000_000 && f < 32_000_000 => 0b11,
            _ => 0b00,
        };
        rcc.ecscr.modify(|_, w| w.hse_freq().bits(freq_bits));
        match bypassed {
            super::HSEBypassMode::NotBypassed => {
                rcc.cr
                    .modify(|_, w| w.csson().on().hseon().on().hsebyp().not_bypassed());
            }
            super::HSEBypassMode::Bypassed => {
                rcc.cr
                    .modify(|_, w| w.csson().on().hseon().on().hsebyp().bypassed());
            }
        }
        while !rcc.cr.read().hserdy().bit_is_set() {}
    }

    #[cfg(feature = "py32f002b")]
    fn hse_enable(rcc: &mut RCC) {
        // PY32F002B HSE only support the bypass mode
        rcc.cr.modify(|_, w| w.hsebyp().bypassed());
    }

    fn hsi_enable(rcc: &mut RCC, fs: &HSIFreq) {
        rcc.icscr
            .modify(|_, w| w.hsi_fs().variant(fs.clone().into()));
        rcc.cr.modify(|_, w| w.hsion().set_bit());
        while rcc.cr.read().hsirdy().bit_is_clear() {}
    }

    pub(super) fn enable_clock(rcc: &mut RCC, c_src: &SysClkSource) {
        // Enable the requested clock
        match c_src {
            // HSE is not used on some families
            #[allow(unused_variables)]
            SysClkSource::HSE(freq, bypassed) => {
                #[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002a"))]
                hse_enable(rcc, *freq, bypassed);
                #[cfg(feature = "py32f002b")]
                if let HSEBypassMode::Bypassed = bypassed {
                    hse_enable(rcc);
                } else {
                    panic!("HSE in PY32F002B must be bypassed")
                }
            }
            SysClkSource::HSISYS(fs) => {
                hsi_enable(rcc, fs);
            }
        }
    }

    #[cfg(feature = "py32f030")]
    pub(super) fn enable_pll(rcc: &mut RCC, c_src: &SysClkSource, ppre_bits: u8, hpre_bits: u8) {
        let pllsrc_bit = match c_src {
            SysClkSource::HSISYS(_) => false,
            SysClkSource::HSE(_, _) => true,
        };
        rcc.pllcfgr.modify(|_, w| w.pllsrc().bit(pllsrc_bit));

        rcc.cr.modify(|_, w| w.pllon().set_bit());
        while rcc.cr.read().pllrdy().bit_is_clear() {}

        rcc.cfgr
            .modify(|_, w| unsafe { w.ppre().bits(ppre_bits).hpre().bits(hpre_bits).sw().pll() });
    }

    #[cfg(any(feature = "py32f003", feature = "py32f002a", feature = "py32f002b"))]
    #[inline(always)]
    pub(super) fn enable_pll(
        _rcc: &mut RCC,
        _c_src: &SysClkSource,
        _ppre_bits: u8,
        _hpre_bits: u8,
    ) {
        panic!("PLL only supported on py32f030. Please select a sysclk and clock source combination so as to not require the use of PLL")
    }

    pub(super) fn get_sww(c_src: &SysClkSource) -> SW_A {
        match c_src {
            SysClkSource::HSISYS(_) => SW_A::Hsisys,
            SysClkSource::HSE(_, _) => SW_A::Hse,
        }
    }
}

/// Frequency on bus that peripheral is connected in
pub trait BusClock {
    /// Calculates frequency depending on `Clock` state
    fn clock(clocks: &Clocks) -> Hertz;
}

/// Frequency on bus that timer is connected in
pub trait BusTimerClock {
    /// Calculates base frequency of timer depending on `Clock` state
    fn timer_clock(clocks: &Clocks) -> Hertz;
}

impl<T> BusClock for T
where
    T: RccBus,
    T::Bus: BusClock,
{
    fn clock(clocks: &Clocks) -> Hertz {
        T::Bus::clock(clocks)
    }
}

impl<T> BusTimerClock for T
where
    T: RccBus,
    T::Bus: BusTimerClock,
{
    fn timer_clock(clocks: &Clocks) -> Hertz {
        T::Bus::timer_clock(clocks)
    }
}

impl BusClock for AHB {
    fn clock(clocks: &Clocks) -> Hertz {
        clocks.hclk
    }
}

impl BusClock for APB {
    fn clock(clocks: &Clocks) -> Hertz {
        clocks.pclk
    }
}

impl BusTimerClock for APB {
    fn timer_clock(clocks: &Clocks) -> Hertz {
        clocks.pclk_tim()
    }
}

/// Bus associated to peripheral
pub trait RccBus: crate::Sealed {
    /// Bus type;
    type Bus;
}

/// Enable/disable peripheral clock
pub trait Enable: RccBus {
    /// Enable the peripheral clock
    fn enable(rcc: &rcc::RegisterBlock);
    /// Disable the peripheral clock
    fn disable(rcc: &rcc::RegisterBlock);
}
/// Reset peripheral
pub trait Reset: RccBus {
    /// Reset the peripheral
    fn reset(rcc: &rcc::RegisterBlock);
}

/// Frozen clock frequencies
///
/// The existence of this value indicates that the clock configuration can no longer be changed
#[derive(Clone, Copy)]
pub struct Clocks {
    hclk: Hertz,
    pclk: Hertz,
    sysclk: Hertz,
    ppre: u8,
}

impl Clocks {
    /// Returns the frequency of the AHB
    pub const fn hclk(&self) -> Hertz {
        self.hclk
    }

    /// Returns the system (core) frequency
    pub const fn sysclk(&self) -> Hertz {
        self.sysclk
    }

    /// Returns the frequency of the APB
    pub const fn pclk(&self) -> Hertz {
        self.pclk
    }

    /// Returns the frequency of the APB Timers
    /// If pclk is prescaled from hclk, the frequency fed into the timers is doubled
    pub const fn pclk_tim(&self) -> Hertz {
        Hertz::from_raw(self.pclk.raw() * if self.ppre() == 1 { 1 } else { 2 })
    }

    pub(crate) const fn ppre(&self) -> u8 {
        self.ppre
    }
}

use self::inner::SysClkSource;

/// Configuration for system Clocks
pub struct CFGR {
    hclk: Option<u32>,
    pclk: Option<u32>,
    sysclk: Option<u32>,
    clock_src: SysClkSource,
    rcc: RCC,
}

impl CFGR {
    /// use the HSE as system clock
    #[cfg(any(feature = "py32f030", feature = "py32f003", feature = "py32f002a"))]
    pub fn hse<F>(mut self, freq: F, bypass: HSEBypassMode) -> Self
    where
        F: Into<Hertz>,
    {
        self.clock_src = SysClkSource::HSE(freq.into().raw(), bypass);
        self
    }
    /// use the HSE as system clock
    #[cfg(feature = "py32f002b")]
    pub fn hse_bypass<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.clock_src = SysClkSource::HSE(freq.into().raw(), HSEBypassMode::Bypassed);
        self
    }

    /// use the HSI as system clock
    pub fn hsi(mut self, fs: HSIFreq) -> Self {
        self.clock_src = SysClkSource::HSISYS(fs);
        self
    }

    /// set frequency of HCLK
    pub fn hclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.hclk = Some(freq.into().raw());
        self
    }

    /// set frequency of PCLK
    pub fn pclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.pclk = Some(freq.into().raw());
        self
    }

    /// set frequency of SYSCLK
    pub fn sysclk<F>(mut self, freq: F) -> Self
    where
        F: Into<Hertz>,
    {
        self.sysclk = Some(freq.into().raw());
        self
    }

    /// Freeze configuration of clocks and set the hardware to that configuration
    /// Returns the [Rcc]
    pub fn freeze(mut self, flash: &mut crate::pac::FLASH) -> Rcc {
        // Default to lowest frequency clock on all systems.
        let sysclk = self.sysclk.unwrap_or(self::inner::HSI_DEFAULT);

        // Select clock source based on user input and capability
        // Highest selected frequency source available takes precedent.
        let src_clk_freq = self::inner::get_freq(&self.clock_src);

        let (pll_en, hsi_div_bits) = if sysclk == src_clk_freq {
            (false, None)
        } else if sysclk == src_clk_freq * 2 {
            (true, None)
        } else {
            if let SysClkSource::HSISYS(_) = self.clock_src {
                let div = match src_clk_freq / sysclk {
                    0 => unreachable!(),
                    1 => 0b000,
                    2 => 0b001,
                    3..=5 => 0b010,
                    6..=11 => 0b011,
                    12..=23 => 0b100,
                    24..=47 => 0b101,
                    48..=95 => 0b110,
                    _ => 0b111,
                };
                (false, Some(div))
            } else {
                unreachable!()
            }
        };

        let r_sysclk;
        if let Some(div_bits) = hsi_div_bits {
            r_sysclk = src_clk_freq / (1 << div_bits);
            self.rcc.cr.modify(|_, w| w.hsidiv().bits(div_bits));
        } else {
            r_sysclk = sysclk;
        }

        let hpre_bits = self
            .hclk
            .map(|hclk| match r_sysclk / hclk {
                0 => unreachable!(),
                1 => 0b0111,
                2 => 0b1000,
                3..=5 => 0b1001,
                6..=11 => 0b1010,
                12..=39 => 0b1011,
                40..=95 => 0b1100,
                96..=191 => 0b1101,
                192..=383 => 0b1110,
                _ => 0b1111,
            })
            .unwrap_or(0b0111);

        let hclk = r_sysclk / (1 << (hpre_bits - 0b0111));

        let ppre_bits = self
            .pclk
            .map(|pclk| match hclk / pclk {
                0 => unreachable!(),
                1 => 0b011,
                2 => 0b100,
                3..=5 => 0b101,
                6..=11 => 0b110,
                _ => 0b111,
            })
            .unwrap_or(0b011);

        let ppre: u8 = 1 << (ppre_bits - 0b011);
        let pclk = hclk / cast::u32(ppre);

        // adjust flash wait states
        flash.acr.write(|w| {
            if r_sysclk <= 24_000_000 {
                w.latency().ws0()
            } else {
                w.latency().ws1()
            }
        });

        // Enable the requested clock
        self::inner::enable_clock(&mut self.rcc, &self.clock_src);

        // Enable PLL
        if pll_en {
            self::inner::enable_pll(&mut self.rcc, &self.clock_src, ppre_bits, hpre_bits);
        } else {
            let sw_var = self::inner::get_sww(&self.clock_src);

            // use HSISYS or HSE as source
            self.rcc.cfgr.modify(|_, w| unsafe {
                w.ppre()
                    .bits(ppre_bits)
                    .hpre()
                    .bits(hpre_bits)
                    .sw()
                    .variant(sw_var)
            });
        }
        Rcc {
            clocks: Clocks {
                hclk: Hz(hclk),
                pclk: Hz(pclk),
                sysclk: Hz(r_sysclk),
                ppre,
            },
            regs: self.rcc,
        }
    }
}
