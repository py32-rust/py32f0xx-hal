/*!
  # Pulse width modulation

  The general purpose timers can be used to output pulse width
  modulated signals on some pins. The timers support up to 4
  simultaneous pwm outputs in separate `Channels`

  ## Usage for pre-defined channel combinations

  This crate defines channel combinations where all the channels are
  enabled. Start by setting all the pins for the timer you want to use
  to the correct alternate functions:

  ```rust
  let gpioa = ..; // Set up and split GPIOA
  // Select the pins you want to use
  let pins = (
      gpioa.pa8.into_alternate_af2(),
      gpioa.pa9.into_alternate_af2(),
      gpioa.pa10.into_alternate_af2(),
      gpioa.pa11.into_alternate_af2(),
  );

  // Set up the timer as a PWM output.
  let (mut c1, mut c2, mut c3, mut c4) = p.TIM1.pwm_hz(pins, 1.kHz(), &clocks).split();

  // Start using the channels
  c1.set_duty(c1.get_max_duty());
  // PWM outputs are disabled by default
  c1.enable()
  // ...
  ```
*/

use super::{
    compute_arr_presc, Channel, FTimer, Instance, Ocm, OcmNPolarity, OcmPolarity, Timer, WithPwm,
};
use crate::rcc::Clocks;
use core::marker::PhantomData;
use core::ops::{Deref, DerefMut};
use fugit::{HertzU32 as Hertz, TimerDurationU32};

/// Trait for a list of PWM pins on a given [Timer]
pub trait Pins<TIM, P> {
    /// Channel 1 Output Pin used
    const C1: bool = false;
    /// Channel 1 Complementary Output Pin used
    const C1N: bool = false;
    /// Channel 2 Output Pin used
    const C2: bool = false;
    /// Channel 2 Complementary Output Pin used
    const C2N: bool = false;
    /// Channel 3 Output Pin used
    const C3: bool = false;
    /// Channel 3 Complementary Output Pin used
    const C3N: bool = false;
    /// Channel 4 Output Pin used
    const C4: bool = false;
    /// The channel list type
    type Channels;

    /// Check if a given channel number is in use
    fn check_used(c: Channel) -> Channel {
        if (c == Channel::C1 && Self::C1)
            || (c == Channel::C2 && Self::C2)
            || (c == Channel::C3 && Self::C3)
            || (c == Channel::C4 && Self::C4)
        {
            c
        } else {
            panic!("Unused channel")
        }
    }

    /// Split the Channels
    fn split() -> Self::Channels;
}

use crate::timer::PinC1;
use crate::timer::PinC1N;
use crate::timer::PinC2;
use crate::timer::PinC2N;
use crate::timer::PinC3;
use crate::timer::PinC3N;
use crate::timer::PinC4;

/// trait for Output Pin/Complementary Pin
pub trait OcPin {
    /// Channel number of pin
    const CN: u8 = 0;
    /// True if the pin is a complementary pin
    const COMP: bool = false;
}
/// Channel 1 Pin
pub struct C1;
impl OcPin for C1 {
    const CN: u8 = 0;
    const COMP: bool = false;
}
/// Channel 1 Complementary Pin
pub struct C1N;
impl OcPin for C1N {
    const CN: u8 = 0;
    const COMP: bool = true;
}
/// Channel 2 Pin
pub struct C2;
impl OcPin for C2 {
    const CN: u8 = 1;
    const COMP: bool = false;
}
/// Channel 2 Complementary Pin
pub struct C2N;
impl OcPin for C2N {
    const CN: u8 = 1;
    const COMP: bool = true;
}
/// Channel 3 Pin
pub struct C3;
impl OcPin for C3 {
    const CN: u8 = 2;
    const COMP: bool = false;
}
/// Channel 3 Complementary Pin
pub struct C3N;
impl OcPin for C3N {
    const CN: u8 = 2;
    const COMP: bool = true;
}
/// Channel 4 Pin
pub struct C4;
impl OcPin for C4 {
    const CN: u8 = 3;
    const COMP: bool = false;
}

/// PWM Channel
pub struct PwmChannel<TIM, OP> {
    _op: PhantomData<OP>,
    _tim: PhantomData<TIM>,
}

macro_rules! pins_impl {
    ( $( ( $($PINX:ident),+ ), ( $($TRAIT:ident),+ ), ( $($ENCHX:ident),* ); )+ ) => {
        $(
            #[allow(unused_parens)]
            impl<TIM, $($PINX,)+> Pins<TIM, ($($ENCHX),+)> for ($($PINX),+)
            where
                TIM: Instance + WithPwm,
                $($PINX: $TRAIT<TIM>,)+
            {
                $(const $ENCHX: bool = true;)+
                type Channels = ($(PwmChannel<TIM, $ENCHX>),+);
                fn split() -> Self::Channels {
                    ($(PwmChannel::<TIM, $ENCHX>::new()),+)
                }
            }
        )+
    };
}

pins_impl!(
    (P1, P2, P3, P4), (PinC1, PinC2, PinC3, PinC4), (C1, C2, C3, C4);
    (P1, P1N, P2, P2N, P3, P3N), (PinC1, PinC1N, PinC2, PinC2N, PinC3, PinC3N), (C1, C1N, C2, C2N, C3, C3N);
    (P1, P1N, P2, P2N), (PinC1, PinC1N, PinC2, PinC2N), (C1, C1N, C2, C2N);
    (P2, P2N, P3, P3N), (PinC2, PinC2N, PinC3, PinC3N), (C2, C2N, C3, C3N);
    (P1, P1N, P3, P3N), (PinC1, PinC1N, PinC3, PinC3N), (C1, C1N, C3, C3N);
    (P2, P3, P4), (PinC2, PinC3, PinC4), (C2, C3, C4);
    (P1, P3, P4), (PinC1, PinC3, PinC4), (C1, C3, C4);
    (P1, P2, P4), (PinC1, PinC2, PinC4), (C1, C2, C4);
    (P1, P2, P3), (PinC1, PinC2, PinC3), (C1, C2, C3);
    (P3, P4), (PinC3, PinC4), (C3, C4);
    (P2, P4), (PinC2, PinC4), (C2, C4);
    (P2, P3), (PinC2, PinC3), (C2, C3);
    (P1, P4), (PinC1, PinC4), (C1, C4);
    (P1, P3), (PinC1, PinC3), (C1, C3);
    (P1, P2), (PinC1, PinC2), (C1, C2);
    (P1, P1N), (PinC1, PinC1N), (C1, C1N);
    (P2, P2N), (PinC2, PinC2N), (C2, C2N);
    (P3, P3N), (PinC3, PinC3N), (C3, C3N);
    (P1), (PinC1), (C1);
    (P2), (PinC2), (C2);
    (P3), (PinC3), (C3);
    (P4), (PinC4), (C4);
);

impl<TIM, P1: PinC1<TIM>, P2: PinC1<TIM>> PinC1<TIM> for (P1, P2) {}
impl<TIM, P1: PinC2<TIM>, P2: PinC2<TIM>> PinC2<TIM> for (P1, P2) {}
impl<TIM, P1: PinC3<TIM>, P2: PinC3<TIM>> PinC3<TIM> for (P1, P2) {}
impl<TIM, P1: PinC4<TIM>, P2: PinC4<TIM>> PinC4<TIM> for (P1, P2) {}

impl<TIM, P1: PinC1<TIM>, P2: PinC1<TIM>, P3: PinC1<TIM>> PinC1<TIM> for (P1, P2, P3) {}
impl<TIM, P1: PinC2<TIM>, P2: PinC2<TIM>, P3: PinC2<TIM>> PinC2<TIM> for (P1, P2, P3) {}
impl<TIM, P1: PinC3<TIM>, P2: PinC3<TIM>, P3: PinC3<TIM>> PinC3<TIM> for (P1, P2, P3) {}
impl<TIM, P1: PinC4<TIM>, P2: PinC4<TIM>, P3: PinC4<TIM>> PinC4<TIM> for (P1, P2, P3) {}

impl<TIM, P1: PinC1<TIM>, P2: PinC1<TIM>, P3: PinC1<TIM>, P4: PinC1<TIM>> PinC1<TIM>
    for (P1, P2, P3, P4)
{
}
impl<TIM, P1: PinC2<TIM>, P2: PinC2<TIM>, P3: PinC2<TIM>, P4: PinC2<TIM>> PinC2<TIM>
    for (P1, P2, P3, P4)
{
}
impl<TIM, P1: PinC3<TIM>, P2: PinC3<TIM>, P3: PinC3<TIM>, P4: PinC3<TIM>> PinC3<TIM>
    for (P1, P2, P3, P4)
{
}
impl<TIM, P1: PinC4<TIM>, P2: PinC4<TIM>, P3: PinC4<TIM>, P4: PinC4<TIM>> PinC4<TIM>
    for (P1, P2, P3, P4)
{
}

/// Extension trait to alter a [Timer] to a [Pwm]
pub trait PwmExt
where
    Self: Sized + Instance + WithPwm,
{
    /// Configure a [Timer] into a [Pwm] with a list of pins and a duration
    fn pwm<P, PINS, const FREQ: u32>(
        self,
        pins: PINS,
        time: TimerDurationU32<FREQ>,
        clocks: &Clocks,
    ) -> Pwm<Self, P, PINS, FREQ>
    where
        PINS: Pins<Self, P>;

    /// Configure a [Timer] into a [PwmHz] with a list of pins and a frequency
    fn pwm_hz<P, PINS>(self, pins: PINS, freq: Hertz, clocks: &Clocks) -> PwmHz<Self, P, PINS>
    where
        PINS: Pins<Self, P>;

    /// Configure a [Timer] into a [PwmHz] with a list of pins and a frequency can be set in the future
    fn pwm_hz_mutable_frequency<P, PINS>(self, pins: PINS, clocks: &Clocks) -> PwmHz<Self, P, PINS>
    where
        PINS: Pins<Self, P>;

    /// Configure a [Timer] into a [Pwm] with a list of pins and a duration in μs
    fn pwm_us<P, PINS>(
        self,
        pins: PINS,
        time: TimerDurationU32<1_000_000>,
        clocks: &Clocks,
    ) -> Pwm<Self, P, PINS, 1_000_000>
    where
        PINS: Pins<Self, P>,
    {
        self.pwm::<_, _, 1_000_000>(pins, time, clocks)
    }
}

impl<TIM> PwmExt for TIM
where
    Self: Sized + Instance + WithPwm,
{
    /// Configure a [Timer] into a [Pwm] with a list of pins and a duration
    fn pwm<P, PINS, const FREQ: u32>(
        self,
        pins: PINS,
        time: TimerDurationU32<FREQ>,
        clocks: &Clocks,
    ) -> Pwm<TIM, P, PINS, FREQ>
    where
        PINS: Pins<TIM, P>,
    {
        FTimer::<Self, FREQ>::new(self, clocks).pwm(pins, time)
    }

    /// Configure a [Timer] into a [PwmHz] with a list of pins and a frequency
    fn pwm_hz<P, PINS>(self, pins: PINS, time: Hertz, clocks: &Clocks) -> PwmHz<TIM, P, PINS>
    where
        PINS: Pins<TIM, P>,
    {
        Timer::new(self, clocks).pwm_hz(pins, time)
    }

    /// Configure a [Timer] into a [PwmHz] with a list of pins and a frequency can be set in the future
    fn pwm_hz_mutable_frequency<P, PINS>(self, pins: PINS, clocks: &Clocks) -> PwmHz<TIM, P, PINS>
    where
        PINS: Pins<TIM, P>,
    {
        Timer::new(self, clocks).pwm_hz_option_frequency(pins, None)
    }
}

impl<TIM: Instance + WithPwm, OP> PwmChannel<TIM, OP> {
    pub(crate) fn new() -> Self {
        Self {
            _op: core::marker::PhantomData,
            _tim: core::marker::PhantomData,
        }
    }
}

impl<TIM: Instance + WithPwm, OP: OcPin> PwmChannel<TIM, OP> {
    /// Disable the PWM Channel
    #[inline]
    pub fn disable(&mut self) {
        if OP::COMP {
            TIM::enable_comp(OP::CN, false);
        } else {
            TIM::enable_channel(OP::CN, false);
        }
    }

    /// Enable the PWM Channel
    #[inline]
    pub fn enable(&mut self) {
        if OP::COMP {
            TIM::enable_comp(OP::CN, true);
        } else {
            TIM::enable_channel(OP::CN, true);
        }
    }

    /// Get the duty on the PWM Channel
    #[inline]
    pub fn get_duty(&self) -> u16 {
        TIM::read_cc_value(OP::CN) as u16
    }

    /// If `0` returned means max_duty is 2^16
    #[inline]
    pub fn get_max_duty(&self) -> u16 {
        (TIM::read_auto_reload() as u16).wrapping_add(1)
    }

    /// Set the duty on the PWM Channel
    #[inline]
    pub fn set_duty(&mut self, duty: u16) {
        TIM::set_cc_value(OP::CN, duty as u32)
    }
}

/// PWM at specificy frequence with list of pins
pub struct PwmHz<TIM, P, PINS>
where
    TIM: Instance + WithPwm,
    PINS: Pins<TIM, P>,
{
    timer: Timer<TIM>,
    freq: Option<Hertz>,
    _pins: PhantomData<(P, PINS)>,
}

impl<TIM, P, PINS> PwmHz<TIM, P, PINS>
where
    TIM: Instance + WithPwm,
    PINS: Pins<TIM, P>,
{
    /// Release the Timer instance and pins
    pub fn release(mut self) -> Timer<TIM> {
        // stop timer
        self.tim.cr1_reset();
        self.timer
    }

    /// Split the DMA channels from [PwmHz]
    pub fn split(self) -> PINS::Channels {
        PINS::split()
    }

    pub fn channels(&self) -> PINS::Channels {
        PINS::split()
    }

    /// start_pwm
    pub fn start_pwm(&mut self) {
        if let Some(freq) = self.freq {
            let (psc, arr) = compute_arr_presc(freq.raw(), self.timer.clk.raw());

            let tim = &mut self.timer.tim;
            tim.set_prescaler(psc);
            tim.set_auto_reload(arr).unwrap();

            // Trigger update event to load the registers
            tim.trigger_update();
            tim.start_pwm();
        }
    }

    pub fn stop_pwm(&mut self) {
        if let Some(_freq) = self.freq {
            let tim = &mut self.timer.tim;
            tim.stop_pwm();
        }
    }

    pub fn set_frequency(&mut self, freq: Hertz) {
        self.stop_pwm();
        self.freq = Some(freq);
        self.start_pwm();
    }

    // pub fn enable_channel<PIN:OcPin>(&self, _p: PIN) {
    //     TIM::enable_channel(PIN::CN, true);
    // }

    // pub fn disable_channel<PIN:OcPin>(&self, _p: PIN) {
    //     TIM::enable_channel(PIN::CN, false);
    // }

    // pub fn get_max_duty(&self) -> u16 {
    //     (TIM::read_auto_reload() as u16).wrapping_add(1)
    // }

    // pub fn set_duty(&self, _p:PIN, duty: u32 ) {
    //     TIM::set_cc_value(PIN::CN, duty as u32)
    // }
}

impl<TIM, P, PINS> Deref for PwmHz<TIM, P, PINS>
where
    TIM: Instance + WithPwm,
    PINS: Pins<TIM, P>,
{
    type Target = Timer<TIM>;
    fn deref(&self) -> &Self::Target {
        &self.timer
    }
}

impl<TIM, P, PINS> DerefMut for PwmHz<TIM, P, PINS>
where
    TIM: Instance + WithPwm,
    PINS: Pins<TIM, P>,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.timer
    }
}

impl<TIM: Instance + WithPwm> Timer<TIM> {

    /// Configure a PWM timer with a list of pins and a period in option[Hertz], if it is Some(Hertz) it starts pwm immediately. If it is None it will not start pwm immediately.  You can set freq in PwmHz to start pwm.
    pub fn pwm_hz_option_frequency<P, PINS>(mut self, _pins: PINS, freq: Option<Hertz>) -> PwmHz<TIM, P, PINS>
    where
        PINS: Pins<TIM, P>, 
    {
        if PINS::C1N | PINS::C2N | PINS::C3N {
            self.tim.set_comp_off_state_run_mode(false);
        }
        if PINS::C1 {
            self.tim
                .preload_output_channel_in_mode(Channel::C1, Ocm::PwmMode1, OcmPolarity::High);
        }
        if PINS::C2 && TIM::CH_NUM > 1 {
            self.tim
                .preload_output_channel_in_mode(Channel::C2, Ocm::PwmMode1, OcmPolarity::High);
        }
        if PINS::C3 && TIM::CH_NUM > 2 {
            self.tim
                .preload_output_channel_in_mode(Channel::C3, Ocm::PwmMode1, OcmPolarity::High);
        }
        if PINS::C4 && TIM::CH_NUM > 3 {
            self.tim
                .preload_output_channel_in_mode(Channel::C4, Ocm::PwmMode1, OcmPolarity::High);
        }
        if PINS::C1N && TIM::CHN_NUM > 0 {
            self.tim
                .set_output_compn_polarity(Channel::C1, OcmNPolarity::High);
        }
        if PINS::C2N && TIM::CHN_NUM > 1 {
            self.tim
                .set_output_compn_polarity(Channel::C2, OcmNPolarity::High);
        }
        if PINS::C3N && TIM::CHN_NUM > 2 {
            self.tim
                .set_output_compn_polarity(Channel::C3, OcmNPolarity::High);
        }
        // The reference manual is a bit ambiguous about when enabling this bit is really
        // necessary, but since we MUST enable the preload for the output channels then we
        // might as well enable for the auto-reload too
        self.tim.enable_preload(true);

        let mut pwm_hz = PwmHz {
            timer: self,
            freq: freq,
            _pins: PhantomData,
        };

        if let Some(_) = freq {
            pwm_hz.start_pwm();
        }
        pwm_hz
    }

    /// Configure a PWM timer with a list of pins and a period in [Hertz]
    pub fn pwm_hz<P, PINS>(self, pins: PINS, freq: Hertz) -> PwmHz<TIM, P, PINS>
    where
        PINS: Pins<TIM, P>,
    {
        self.pwm_hz_option_frequency(pins, Some(freq))
    }
}

/// Functions for a Timer in PWM mode
impl<TIM, P, PINS> PwmHz<TIM, P, PINS>
where
    TIM: Instance + WithPwm,
    PINS: Pins<TIM, P>,
{
    /// Enable a [Channel]
    ///
    /// Panic's if channel number is not possible for this timer
    /// To avoid panics, split the channels from the timer and
    /// use the individual channel
    pub fn enable(&mut self, channel: Channel) {
        TIM::enable_channel(PINS::check_used(channel) as u8, true)
    }

    /// Disable a [Channel]
    ///
    /// Panic's if channel number is not possible for this timer
    /// To avoid panic's, split the channels from the timer and
    /// use the individual channel
    pub fn disable(&mut self, channel: Channel) {
        TIM::enable_channel(PINS::check_used(channel) as u8, false)
    }

    /// Get the current duty of a [Channel]
    ///
    /// Panic's if channel number is not possible for this timer
    /// To avoid panic's, split the channels from the timer and
    /// use the individual channel
    pub fn get_duty(&self, channel: Channel) -> u16 {
        TIM::read_cc_value(PINS::check_used(channel) as u8) as u16
    }

    /// Set the duty on a [Channel]
    ///
    /// Panic's if channel number is not possible for this timer
    /// To avoid panic's, split the channels from the timer and
    /// use the individual channel
    pub fn set_duty(&mut self, channel: Channel, duty: u16) {
        TIM::set_cc_value(PINS::check_used(channel) as u8, duty as u32)
    }

    /// Get the maximum possible duty for this timer
    ///
    /// If `0` returned means max_duty is 2^16
    pub fn get_max_duty(&self) -> u16 {
        (TIM::read_auto_reload() as u16).wrapping_add(1)
    }

    /// Get the Clock rate of this timer
    pub fn get_period(&self) -> Hertz {
        let clk = self.clk;
        let psc = self.tim.read_prescaler() as u32;
        let arr = TIM::read_auto_reload();

        // Length in ms of an internal clock pulse
        clk / ((psc + 1) * (arr + 1))
    }

    /// Set the Clock rate for this timer
    pub fn set_period(&mut self, period: Hertz) {
        let clk = self.clk;

        let (psc, arr) = compute_arr_presc(period.raw(), clk.raw());
        self.tim.set_prescaler(psc);
        self.tim.set_auto_reload(arr).unwrap();
    }
}

/// PWM timer
pub struct Pwm<TIM, P, PINS, const FREQ: u32>
where
    TIM: Instance + WithPwm,
    PINS: Pins<TIM, P>,
{
    timer: FTimer<TIM, FREQ>,
    _pins: PhantomData<(P, PINS)>,
}

impl<TIM, P, PINS, const FREQ: u32> Pwm<TIM, P, PINS, FREQ>
where
    TIM: Instance + WithPwm,
    PINS: Pins<TIM, P>,
{
    /// Split DMA channels from self
    pub fn split(self) -> PINS::Channels {
        PINS::split()
    }

    /// Release the timer
    pub fn release(mut self) -> FTimer<TIM, FREQ> {
        // stop counter
        self.tim.cr1_reset();
        self.timer
    }
}

impl<TIM, P, PINS, const FREQ: u32> Deref for Pwm<TIM, P, PINS, FREQ>
where
    TIM: Instance + WithPwm,
    PINS: Pins<TIM, P>,
{
    type Target = FTimer<TIM, FREQ>;
    fn deref(&self) -> &Self::Target {
        &self.timer
    }
}

impl<TIM, P, PINS, const FREQ: u32> DerefMut for Pwm<TIM, P, PINS, FREQ>
where
    TIM: Instance + WithPwm,
    PINS: Pins<TIM, P>,
{
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.timer
    }
}

impl<TIM: Instance + WithPwm, const FREQ: u32> FTimer<TIM, FREQ> {
    /// Create a [Pwm] with a list of pins and a maximum cycle duration
    pub fn pwm<P, PINS>(
        mut self,
        _pins: PINS,
        time: TimerDurationU32<FREQ>,
    ) -> Pwm<TIM, P, PINS, FREQ>
    where
        PINS: Pins<TIM, P>,
    {
        if PINS::C1N | PINS::C2N | PINS::C3N {
            self.tim.set_comp_off_state_run_mode(false);
        }
        if PINS::C1 {
            self.tim
                .preload_output_channel_in_mode(Channel::C1, Ocm::PwmMode1, OcmPolarity::High);
        }
        if PINS::C2 && TIM::CH_NUM > 1 {
            self.tim
                .preload_output_channel_in_mode(Channel::C2, Ocm::PwmMode1, OcmPolarity::High);
        }
        if PINS::C3 && TIM::CH_NUM > 2 {
            self.tim
                .preload_output_channel_in_mode(Channel::C3, Ocm::PwmMode1, OcmPolarity::High);
        }
        if PINS::C4 && TIM::CH_NUM > 3 {
            self.tim
                .preload_output_channel_in_mode(Channel::C4, Ocm::PwmMode1, OcmPolarity::High);
        }
        if PINS::C1N && TIM::CHN_NUM > 0 {
            self.tim
                .set_output_compn_polarity(Channel::C1, OcmNPolarity::High);
        }
        if PINS::C2N && TIM::CHN_NUM > 1 {
            self.tim
                .set_output_compn_polarity(Channel::C2, OcmNPolarity::High);
        }
        if PINS::C3N && TIM::CHN_NUM > 2 {
            self.tim
                .set_output_compn_polarity(Channel::C3, OcmNPolarity::High);
        }

        // The reference manual is a bit ambiguous about when enabling this bit is really
        // necessary, but since we MUST enable the preload for the output channels then we
        // might as well enable for the auto-reload too
        self.tim.enable_preload(true);

        self.tim.set_auto_reload(time.ticks() - 1).unwrap();

        // Trigger update event to load the registers
        self.tim.trigger_update();

        self.tim.start_pwm();

        Pwm {
            timer: self,
            _pins: PhantomData,
        }
    }
}

impl<TIM, P, PINS, const FREQ: u32> Pwm<TIM, P, PINS, FREQ>
where
    TIM: Instance + WithPwm,
    PINS: Pins<TIM, P>,
{
    /// Enable a channel
    ///
    /// Panic's if the channel is not available on this timer
    /// To avoid panic's, split the channels from the timer and
    /// use the individual channel
    pub fn enable(&mut self, channel: Channel) {
        TIM::enable_channel(PINS::check_used(channel) as u8, true)
    }

    /// Disable a channel
    ///
    /// Panic's if the channel is not available on this timer
    /// To avoid panic's, split the channels from the timer and
    /// use the individual channel
    pub fn disable(&mut self, channel: Channel) {
        TIM::enable_channel(PINS::check_used(channel) as u8, false)
    }

    /// Get the duty cycle on a channel
    ///
    /// Panic's if the channel is not available on this timer
    /// To avoid panic's, split the channels from the timer and
    /// use the individual channel
    pub fn get_duty(&self, channel: Channel) -> u16 {
        TIM::read_cc_value(PINS::check_used(channel) as u8) as u16
    }

    /// Set the duty cycle on a channel
    ///
    /// Panic's if the channel is not available on this timer
    /// To avoid panic's, split the channels from the timer and
    /// use the individual channel
    pub fn set_duty(&mut self, channel: Channel, duty: u16) {
        TIM::set_cc_value(PINS::check_used(channel) as u8, duty.into())
    }

    /// If `0` returned means max_duty is 2^16
    pub fn get_max_duty(&self) -> u16 {
        (TIM::read_auto_reload() as u16).wrapping_add(1)
    }

    /// Get the current duty period on the timer
    pub fn get_period(&self) -> TimerDurationU32<FREQ> {
        TimerDurationU32::from_ticks(TIM::read_auto_reload() + 1)
    }

    /// Set the duty period on the timer
    ///
    /// Sets the period using the auto reload, it will take effect once
    /// the timer counter hits previous period
    pub fn set_period(&mut self, period: TimerDurationU32<FREQ>) {
        self.tim.set_auto_reload(period.ticks() - 1).unwrap();
    }
}
