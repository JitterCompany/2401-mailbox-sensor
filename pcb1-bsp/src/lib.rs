
#![no_std]

use switch_hal::{ActiveHigh, Switch};

use stm32g0xx_hal::{
    prelude::*,
    stm32,
    gpio::gpiob,
    gpio::{Output, PushPull},
    serial::Config
};


pub struct Leds {
    pub led_red: gpiob::PB5<Output<PushPull>>,
    pub led_green: gpiob::PB9<Output<PushPull>>
}

impl Leds {

    /// Initializes the user LEDs to OFF
    pub fn new(pb5: gpiob::PB5<Output<PushPull>>, pb9: gpiob::PB9<Output<PushPull>>) -> Self {
        Self {
            led_red: pb5,
            led_green: pb9
        }
    }

}


pub struct Board {
    leds: Leds
}

impl Board {

    pub fn new() -> Self {

        let dp = stm32::Peripherals::take().expect("cannot take peripherals");
        let mut rcc = dp.RCC.constrain();
        let gpiob = dp.GPIOB.split(&mut rcc);
        let pb5 = gpiob.pb5.into_push_pull_output();
        let pb9 = gpiob.pb9.into_push_pull_output();

        let leds = Leds::new(pb5, pb9);
        // led1.set_high().unwrap();

        // led2.set_low().unwrap();

        // return ne

        Board {leds}
    }
}

#[cfg(test)]
mod tests {
    #[test]
    fn it_works() {
        assert_eq!(2 + 2, 4);
    }
}
