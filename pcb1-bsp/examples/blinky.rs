
#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate panic_halt;

use pcb1_bsp::switch_hal::{OutputSwitch};
use pcb1_bsp::{Board};

use rt::entry;


#[entry]
fn main() -> ! {

    let mut board = Board::new();

    loop {
        for _ in 0..10_000 {
            board.leds.led_green.on().ok();
        }
        for _ in 0..10_000 {
            board.leds.led_green.off().ok();
        }

    }
}