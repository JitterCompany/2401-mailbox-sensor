#![deny(unsafe_code)]
#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;
extern crate panic_halt;
extern crate stm32g0xx_hal as hal;

use rt::entry;
use nb::block;

// use embedded_hal::digital::v2::OutputPin;
use core::fmt::Write;

use stm32g0xx_hal::{
    prelude::*,
    stm32,
    serial::Config,
    i2c,
    spi
};

// use spi_memory::prelude::*;
use spi_memory::series25::Flash;

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

    // I2C pins
    // PB6 = SCL, yellow, links boven TP5(rood)
    // PB7 = SDA, green, midden boven TP6(zwart)
    let scl = gpiob.pb6.into_open_drain_output();
    let sda = gpiob.pb7.into_open_drain_output();

    let sck = gpioa.pa5;
    let miso = gpioa.pa6;
    let mosi = gpioa.pa7;
    let mut flash_cs = gpioa.pa8.into_push_pull_output();
    flash_cs.set_high().unwrap();

    writeln!(usart, "Hallo, brievenbus!\n").unwrap();

    let mut spi = dp.SPI1.spi(
        (sck, miso, mosi),
        spi::MODE_0,
        1000.khz(),
        &mut rcc);


    let read_dev_id: u8 = 0x9F;
    flash_cs.set_low().unwrap();

    block!(spi.send(0x05)).unwrap();
    block!(spi.read()).unwrap();

    block!(spi.send(0x00)).unwrap();
    block!(spi.read()).unwrap();

    flash_cs.set_high().unwrap();

    for _ in 0..20 {
        // led1.set_low().unwrap();
    }

    flash_cs.set_low().unwrap();

    block!(spi.send(read_dev_id)).unwrap();
    block!(spi.read()).unwrap();
    block!(spi.send(0x0)).unwrap();
    block!(spi.read()).unwrap();
    block!(spi.send(0x0)).unwrap();
    block!(spi.read()).unwrap();
    block!(spi.send(0x0)).unwrap();
    let res1 = block!(spi.read());

    flash_cs.set_high().unwrap();

    let mut flash = Flash::init(spi, flash_cs).unwrap();
    let id = flash.read_jedec_id().unwrap();

    writeln!(usart, "s25 spi  ID: {}", res1.unwrap()).unwrap();

    writeln!(usart, "s25 spi memory id: {:?}", id).unwrap();



    let mut i2c = dp
        .I2C1
        .i2c(sda, scl, i2c::Config::with_timing(0x2020151b), &mut rcc);

    writeln!(usart, "i2c initialized!\n").unwrap();

    // vl6180x
    let address = 0x29; //0x31;

    let mut data = [0, 0];
    let reg = [0x00, 0x00]; //[0x0f];

    i2c.write_read(address,  &reg, &mut data).unwrap();
    writeln!(usart, "Got {} from reg {}!", data[0], reg[0]).unwrap();

    let reg = [0x00, 0x01]; //[0x0f];

    i2c.write_read(address,  &reg, &mut data).unwrap();
    writeln!(usart, "Got {} from reg {}!", data[0], reg[1]).unwrap();

    let reg = [0x00, 0x02]; //[0x0f];

    i2c.write_read(address,  &reg, &mut data).unwrap();
    writeln!(usart, "Got {} from reg {}!", data[0], reg[1]).unwrap();


    //accel
    let address = 0x19;

    let reg = [0x0f];
    let mut data = [0];

    i2c.write_read(address,  &reg, &mut data).unwrap();
    writeln!(usart, "accel: Got {} from reg {}!", data[0], reg[0]).unwrap();


    //pressure
    let address = 0x76;

    let reg = [0x1D];
    let mut data = [0];

    i2c.write_read(address,  &reg, &mut data).unwrap();
    writeln!(usart, "pressure: Got {} from reg {}!", data[0], reg[0]).unwrap();



    loop {
        for _ in 0..10_000 {
            led1.set_low().unwrap();
        }
        for _ in 0..10_000 {
            led1.set_high().unwrap();
        }
        led2.toggle().unwrap();

    }
}
