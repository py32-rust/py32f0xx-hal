/*!
  # Pulse width modulation

  The general purpose timers (`TIM1`, `TIM3`, `TIM14`, `TIM16`, and `TIM17`) can be used to output
  pulse width modulated signals on some pins. The timers support up to 4 simultaneous pwm outputs in separate `Channels`

  ## Usage for pre-defined channel combinations

  This crate defines channel combinations where all the channels are enabled. Start by setting all the pins for the
  timer you want to use to alternate functions:

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

use super::{compute_arr_presc, Channel, FTimer, Instance, Ocm, Timer, WithPwm};
use crate::rcc::Clocks;
use core::marker::PhantomData;
use core::ops::{Deref, DerefMut};
use fugit::{HertzU32 as Hertz, TimerDurationU32};

pub trait Pins<TIM, P> {
    const C1: bool = false;
    const C1N: bool = false;
    const C2: bool = false;
    const C2N: bool = false;
    const C3: bool = false;
    const C3N: bool = false;
    const C4: bool = false;
    type Channels;

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

    fn split() -> Self::Channels;
}

use crate::timer::PinC1;
use crate::timer::PinC1N;
use crate::timer::PinC2;
use crate::timer::PinC2N;
use crate::timer::PinC3;
use crate::timer::PinC3N;
use crate::timer::PinC4;

pub trait OcPin {
    const CN: u8 = 0;
    const COMP: bool = false;
}
pub struct C1;
impl OcPin for C1 {
    const CN: u8 = 0;
    const COMP: bool = false;
}
pub struct C1N;
impl OcPin for C1N {
    const CN: u8 = 0;
    const COMP: bool = true;
}
pub struct C2;
impl OcPin for C2 {
    const CN: u8 = 1;
    const COMP: bool = false;
}
pub struct C2N;
impl OcPin for C2N {
    const CN: u8 = 1;
    const COMP: bool = true;
}
pub struct C3;
impl OcPin for C3 {
    const CN: u8 = 2;
    const COMP: bool = false;
}
pub struct C3N;
impl OcPin for C3N {
    const CN: u8 = 2;
    const COMP: bool = true;
}
pub struct C4;
impl OcPin for C4 {
    const CN: u8 = 3;
    const COMP: bool = false;
}

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

pub trait PwmExt
where
    Self: Sized + Instance + WithPwm,
{
    fn pwm<P, PINS, const FREQ: u32>(
        self,
        pins: PINS,
        time: TimerDurationU32<FREQ>,
        clocks: &Clocks,
    ) -> Pwm<Self, P, PINS, FREQ>
    where
        PINS: Pins<Self, P>;

    fn pwm_hz<P, PINS>(self, pins: PINS, freq: Hertz, clocks: &Clocks) -> PwmHz<Self, P, PINS>
    where
        PINS: Pins<Self, P>;

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

    fn pwm_hz<P, PINS>(self, pins: PINS, time: Hertz, clocks: &Clocks) -> PwmHz<TIM, P, PINS>
    where
        PINS: Pins<TIM, P>,
    {
        Timer::new(self, clocks).pwm_hz(pins, time)
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
    #[inline]
    pub fn disable(&mut self) {
        if OP::COMP {
            TIM::enable_comp(OP::CN, false);
        } else {
            TIM::enable_channel(OP::CN, false);
        }
    }

    #[inline]
    pub fn enable(&mut self) {
        if OP::COMP {
            TIM::enable_comp(OP::CN, true);
        } else {
            TIM::enable_channel(OP::CN, true);
        }
    }

    #[inline]
    pub fn get_duty(&self) -> u16 {
        TIM::read_cc_value(OP::CN) as u16
    }

    /// If `0` returned means max_duty is 2^16
    #[inline]
    pub fn get_max_duty(&self) -> u16 {
        (TIM::read_auto_reload() as u16).wrapping_add(1)
    }

    #[inline]
    pub fn set_duty(&mut self, duty: u16) {
        TIM::set_cc_value(OP::CN, duty as u32)
    }
}

pub struct PwmHz<TIM, P, PINS>
where
    TIM: Instance + WithPwm,
    PINS: Pins<TIM, P>,
{
    timer: Timer<TIM>,
    _pins: PhantomData<(P, PINS)>,
}

impl<TIM, P, PINS> PwmHz<TIM, P, PINS>
where
    TIM: Instance + WithPwm,
    PINS: Pins<TIM, P>,
{
    pub fn release(mut self) -> Timer<TIM> {
        // stop timer
        self.tim.cr1_reset();
        self.timer
    }

    pub fn split(self) -> PINS::Channels {
        PINS::split()
    }
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
    pub fn pwm_hz<P, PINS>(mut self, _pins: PINS, freq: Hertz) -> PwmHz<TIM, P, PINS>
    where
        PINS: Pins<TIM, P>,
    {
        if PINS::C1N | PINS::C1N | PINS::C1N {
            self.tim.set_comp_off_state_run_mode(true);
        }
        if PINS::C1 {
            self.tim
                .preload_output_channel_in_mode(Channel::C1, Ocm::PwmMode1);
        }
        if PINS::C2 && TIM::CH_NUM > 1 {
            self.tim
                .preload_output_channel_in_mode(Channel::C2, Ocm::PwmMode1);
        }
        if PINS::C3 && TIM::CH_NUM > 2 {
            self.tim
                .preload_output_channel_in_mode(Channel::C3, Ocm::PwmMode1);
        }
        if PINS::C4 && TIM::CH_NUM > 3 {
            self.tim
                .preload_output_channel_in_mode(Channel::C4, Ocm::PwmMode1);
        }

        // The reference manual is a bit ambiguous about when enabling this bit is really
        // necessary, but since we MUST enable the preload for the output channels then we
        // might as well enable for the auto-reload too
        self.tim.enable_preload(true);

        let (psc, arr) = compute_arr_presc(freq.raw(), self.clk.raw());
        self.tim.set_prescaler(psc);
        self.tim.set_auto_reload(arr).unwrap();

        // Trigger update event to load the registers
        self.tim.trigger_update();

        self.tim.start_pwm();

        PwmHz {
            timer: self,
            _pins: PhantomData,
        }
    }
}

impl<TIM, P, PINS> PwmHz<TIM, P, PINS>
where
    TIM: Instance + WithPwm,
    PINS: Pins<TIM, P>,
{
    pub fn enable(&mut self, channel: Channel) {
        TIM::enable_channel(PINS::check_used(channel) as u8, true)
    }

    pub fn disable(&mut self, channel: Channel) {
        TIM::enable_channel(PINS::check_used(channel) as u8, false)
    }

    pub fn get_duty(&self, channel: Channel) -> u16 {
        TIM::read_cc_value(PINS::check_used(channel) as u8) as u16
    }

    pub fn set_duty(&mut self, channel: Channel, duty: u16) {
        TIM::set_cc_value(PINS::check_used(channel) as u8, duty as u32)
    }

    /// If `0` returned means max_duty is 2^16
    pub fn get_max_duty(&self) -> u16 {
        (TIM::read_auto_reload() as u16).wrapping_add(1)
    }

    pub fn get_period(&self) -> Hertz {
        let clk = self.clk;
        let psc = self.tim.read_prescaler() as u32;
        let arr = TIM::read_auto_reload();

        // Length in ms of an internal clock pulse
        clk / ((psc + 1) * (arr + 1))
    }

    pub fn set_period(&mut self, period: Hertz) {
        let clk = self.clk;

        let (psc, arr) = compute_arr_presc(period.raw(), clk.raw());
        self.tim.set_prescaler(psc);
        self.tim.set_auto_reload(arr).unwrap();
    }
}

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
    pub fn split(self) -> PINS::Channels {
        PINS::split()
    }

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
    pub fn pwm<P, PINS>(
        mut self,
        _pins: PINS,
        time: TimerDurationU32<FREQ>,
    ) -> Pwm<TIM, P, PINS, FREQ>
    where
        PINS: Pins<TIM, P>,
    {
        if PINS::C1N | PINS::C1N | PINS::C1N {
            self.tim.set_comp_off_state_run_mode(true);
        }
        if PINS::C1 {
            self.tim
                .preload_output_channel_in_mode(Channel::C1, Ocm::PwmMode1);
        }
        if PINS::C2 && TIM::CH_NUM > 1 {
            self.tim
                .preload_output_channel_in_mode(Channel::C2, Ocm::PwmMode1);
        }
        if PINS::C3 && TIM::CH_NUM > 2 {
            self.tim
                .preload_output_channel_in_mode(Channel::C3, Ocm::PwmMode1);
        }
        if PINS::C4 && TIM::CH_NUM > 3 {
            self.tim
                .preload_output_channel_in_mode(Channel::C4, Ocm::PwmMode1);
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
    pub fn enable(&mut self, channel: Channel) {
        TIM::enable_channel(PINS::check_used(channel) as u8, true)
    }

    pub fn disable(&mut self, channel: Channel) {
        TIM::enable_channel(PINS::check_used(channel) as u8, false)
    }

    pub fn get_duty(&self, channel: Channel) -> u16 {
        TIM::read_cc_value(PINS::check_used(channel) as u8) as u16
    }

    pub fn set_duty(&mut self, channel: Channel, duty: u16) {
        TIM::set_cc_value(PINS::check_used(channel) as u8, duty.into())
    }

    /// If `0` returned means max_duty is 2^16
    pub fn get_max_duty(&self) -> u16 {
        (TIM::read_auto_reload() as u16).wrapping_add(1)
    }

    pub fn get_period(&self) -> TimerDurationU32<FREQ> {
        TimerDurationU32::from_ticks(TIM::read_auto_reload() + 1)
    }

    pub fn set_period(&mut self, period: TimerDurationU32<FREQ>) {
        self.tim.set_auto_reload(period.ticks() - 1).unwrap();
    }
}
