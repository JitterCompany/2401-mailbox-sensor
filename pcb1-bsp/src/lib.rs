#![no_std]

// examples: https://github.com/rubberduck203/stm32f3-discovery
// https://github.com/nrf-rs/nrf52-hal

pub use switch_hal;
use switch_hal::{ActiveHigh, Switch, IntoSwitch, OutputSwitch};

use stm32g0xx_hal::{
    prelude::*,
    stm32,
    gpio::gpiob,
    gpio::{Output, PushPull},
};


type RedLed = Switch<gpiob::PB5<Output<PushPull>>, ActiveHigh>;
type GreenLed = Switch<gpiob::PB9<Output<PushPull>>, ActiveHigh>;

pub struct Leds {
    pub led_red: RedLed,
    pub led_green: GreenLed
}

impl Leds {

    /// Initializes the user LEDs to OFF
    pub fn new(pb5: RedLed, pb9: GreenLed) -> Self {
        Self {
            led_red: pb5,
            led_green: pb9
        }
    }

}


pub struct Board {
    pub leds: Leds
}

impl Board {

    pub fn new() -> Self {

        let dp = stm32::Peripherals::take().expect("cannot take peripherals");
        let mut rcc = dp.RCC.constrain();
        let gpiob = dp.GPIOB.split(&mut rcc);
        let pb5 = gpiob.pb5.into_push_pull_output().into_active_high_switch();
        let pb9 = gpiob.pb9.into_push_pull_output().into_active_high_switch();

        let mut leds = Leds::new(pb5, pb9);
        leds.led_green.off().ok();
        leds.led_red.off().ok();

        Board {leds}
    }
}
