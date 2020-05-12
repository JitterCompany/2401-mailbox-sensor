#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate panic_halt;

use rt::entry;

// use embedded_hal::digital::v2::OutputPin;
use core::fmt::Write;

use stm32g0xx_hal::{
    prelude::*,
    stm32,
    serial::Config,
    i2c
};

use dps422::{DPS422, self};

use shared_bus;

#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.constrain();
    let gpiob = dp.GPIOB.split(&mut rcc);
    let mut led1 = gpiob.pb5.into_push_pull_output();
    led1.set_high().unwrap();

    let mut led2 = gpiob.pb9.into_push_pull_output();
    led2.set_low().unwrap();


    let gpioa = dp.GPIOA.split(&mut rcc);


    let tx = gpioa.pa9;
    let rx = gpioa.pa10;
    let mut usart = dp
        .USART1
        .usart(tx, rx, Config::default().baudrate(115200.bps()), &mut rcc)
        .unwrap();

    writeln!(usart, "Hallo, brievenbus!\n").unwrap();

    // I2C pins
    let scl = gpiob.pb6.into_open_drain_output();
    let sda = gpiob.pb7.into_open_drain_output();

    let i2c = dp
        .I2C1
        .i2c(sda, scl, i2c::Config::with_timing(0x2020151b), &mut rcc);

    writeln!(usart, "i2c initialized!\n").unwrap();

    let manager = shared_bus::CortexMBusManager::new(i2c);

    let address = 0x76;
    let mut dps = DPS422::new(manager.acquire(), address, &dps422::Config::new()).unwrap();

    writeln!(usart, "pressure sensor init done").unwrap();
    dps.trigger_measurement(true, true, false).unwrap();

    loop {
        led2.toggle().unwrap();

        if dps.data_ready().unwrap() {
            led1.toggle().unwrap();
            let pressure = dps.read_pressure_calibrated().unwrap();
            let temp = dps.read_temp_calibrated().unwrap();
            writeln!(usart, "pressure: {:.1} [kPa]\t temp: {:.1} [˚C]", pressure, temp).unwrap();
            dps.trigger_measurement(true, true, false).unwrap();
        }
    }
}
