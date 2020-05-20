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
    i2c,
    spi,
    analog::adc::{Precision, SampleTime},
};

use spi_memory::series25::Flash;

use dps422::{DPS422, self};
use vl6180x::{VL6180X};
use lis3dh::{Lis3dh, accelerometer::Accelerometer};

use shared_bus;

mod log_storage;
use log_storage::StorageEngine;

use embedded_hal::digital::v2::OutputPin;

// use serde::{Serialize, Deserialize};
use postcard::{from_bytes};
// use heapless::{Vec, consts::*};

mod sensors;
use sensors::SensorData;


#[entry]
fn main() -> ! {
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");
    let mut rcc = dp.RCC.constrain();
    let gpiob = dp.GPIOB.split(&mut rcc);
    let mut led1 = gpiob.pb5.into_push_pull_output();
    led1.set_high().unwrap();

    let mut led2 = gpiob.pb9.into_push_pull_output();
    led2.set_low().unwrap();

    // Get the delay provider.
    let cp = cortex_m::Peripherals::take().unwrap();
    let mut delay = cp.SYST.delay(&mut rcc);

    let gpioa = dp.GPIOA.split(&mut rcc);


    let tx = gpioa.pa9;
    let rx = gpioa.pa10;
    let mut usart = dp
        .USART1
        .usart(tx, rx, Config::default().baudrate(115200.bps()), &mut rcc)
        .unwrap();

    writeln!(usart, "Hallo, brievenbus!\n").unwrap();


    let spi = {

        let sck = gpioa.pa5;
        let miso = gpioa.pa6;
        let mosi = gpioa.pa7;
        dp.SPI1.spi(
            (sck, miso, mosi),
            spi::MODE_0,
            1000.khz(),
            &mut rcc)
    };

    let mut flash_cs = gpioa.pa8.into_push_pull_output();
    flash_cs.set_high().unwrap();
    let mut flash = Flash::init(spi, flash_cs).unwrap();
    let id = flash.read_jedec_id().unwrap();
    let mut storage = StorageEngine::new(flash, 0x800000, 0x1000, 0x1000);
    let offset = storage.init().unwrap();

    writeln!(usart, "Init flash with id: {:?}, offset: {}\n", id, offset).unwrap();

    // storage.erase(0).unwrap();
    // loop {}

    let mut readbuf = [0u8;27];
    storage.read(0, &mut readbuf).unwrap();

    writeln!(usart, "read from flash: {:?}\n", readbuf).unwrap();

    let deserialized: SensorData = from_bytes(&readbuf[1..]).unwrap();
    writeln!(usart, "deserialized sensors: {:?}", deserialized).unwrap();




    let mut adc = dp.ADC.constrain(&mut rcc);
    adc.set_sample_time(SampleTime::T_2);
    adc.set_precision(Precision::B_12);

    // photo transistor 1 = PA1
    // photo transistor 2 = PA0
    // photo diode 1 =      PA4
    // photo diode 2 =      PA2
    let mut phototransistor2_pin = gpioa.pa0.into_analog();
    let mut phototransistor1_pin = gpioa.pa1.into_analog();
    let mut photodiode1_pin = gpioa.pa4.into_analog();
    let mut photodiode2_pin = gpioa.pa2.into_analog();

    let mut vbat_pin = gpiob.pb2.into_analog();


    // I2C pins
    let scl = gpiob.pb6.into_open_drain_output();
    let sda = gpiob.pb7.into_open_drain_output();

    let i2c = dp
        .I2C1
        .i2c(sda, scl, i2c::Config::with_timing(0x2020151b), &mut rcc);

    writeln!(usart, "i2c initialized!\n").unwrap();

    let manager = shared_bus::CortexMBusManager::new(i2c);

    let mut dps = DPS422::new(manager.acquire(), 0x76, &dps422::Config::new()).unwrap();

    writeln!(usart, "DPS422 init done").unwrap();

    let mut vl6180x = VL6180X::new(manager.acquire()).unwrap();
    writeln!(usart, "VL6180X init done..\n").unwrap();

    let mut lis3dh = Lis3dh::new(manager.acquire(), 0x19).unwrap();
    writeln!(usart, "Lis3dh init done..\n").unwrap();

    lis3dh.set_range(lis3dh::Range::G8).unwrap();

    dps.trigger_measurement(true, true, false).unwrap();
    vl6180x.start_ranging().unwrap();
    let mut sensordata = SensorData::new();
    loop {

        if dps.data_ready().unwrap() {
            led1.toggle().unwrap();
            let pressure = dps.read_pressure_calibrated().unwrap();
            let temp = dps.read_temp_calibrated().unwrap();
            writeln!(usart, "pressure: {:.1} [kPa]\t temp: {:.1} [˚C]", pressure, temp).unwrap();
            dps.trigger_measurement(true, true, false).unwrap();
            sensordata.pressure(pressure);
            sensordata.temp(temp);
        }

        let status = vl6180x.int_status().unwrap();
        if (status & 0b100) == 0b100 {
            let range = vl6180x.read_range().unwrap();
            // writeln!(usart, "range = {} [mm]\n", range).unwrap();
            vl6180x.clear_int().unwrap();
            vl6180x.start_ranging().unwrap();
            led2.toggle().unwrap();
            sensordata.distance(range);
        }

        let accel = lis3dh.acceleration().unwrap();
        writeln!(usart, "accel = {}, {}, {}", accel.x, accel.y, accel.z).unwrap();
        sensordata.accel_x(accel.x);
        sensordata.accel_y(accel.y);
        sensordata.accel_z(accel.z);

        let pt1: u16 = adc.read(&mut phototransistor1_pin).expect("adc read failed");
        let pt2: u16 = adc.read(&mut phototransistor2_pin).expect("adc read failed");
        let pd1: u16 = adc.read(&mut photodiode1_pin).expect("adc read failed");
        let pd2: u16 = adc.read(&mut photodiode2_pin).expect("adc read failed");
        sensordata.photo_t1(pt1);
        sensordata.photo_t2(pt2);
        sensordata.photo_d1(pd1);
        sensordata.photo_d2(pd2);

        let vbat: u16 = adc.read(&mut vbat_pin).expect("adc read failed");
        sensordata.vbat(vbat);

        // writeln!(usart, "photo transistors: {},  {}", pt1, pt2).unwrap();
        // writeln!(usart, "photo diodes: {},  {}", pd1, pd2).unwrap();
        // writeln!(usart, "vbat: {} mV?", vbat*2*3000/4096).unwrap();


        if sensordata.ready() {
            writeln!(usart, "write to flash: {:?}", sensordata).unwrap();
            let _res = storage.write(sensordata).unwrap();
            sensordata = SensorData::new();
        }

        delay.delay_ms(5000_u16);

    }
}
