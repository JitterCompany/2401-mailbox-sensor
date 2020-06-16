#![no_main]
#![no_std]

extern crate cortex_m;
extern crate cortex_m_rt as rt;

use rt::entry;
use core::fmt::Write;

use stm32g0xx_hal::{
    prelude::*,
    stm32::{self, interrupt},
    serial::Config,
    timer,
    i2c,
    analog::adc::{Precision, SampleTime, VRef},
};

use stm32g0xx_hal::spi;


#[cfg(feature = "low_power")]
use stm32g0xx_hal::rcc::{Config as RCCConfig, Prescaler};

use cortex_m::{interrupt::Mutex};
use core::{cell::RefCell, ops::DerefMut, cell::UnsafeCell};

use vl6180x::{VL6180X};

use spi_memory::series25::Flash;

mod log_storage;
use log_storage::StorageEngine;

use embedded_hal::digital::v2::OutputPin;

mod sensors;
use sensors::SensorData;

mod counter;
use counter::CSCounter;

mod detector;
use detector::{Detector, TriggerEdge};


static TIME_MS: CSCounter<u32> = CSCounter(UnsafeCell::new(0));
static TIMER1: Mutex<RefCell<Option<timer::Timer<stm32::TIM1>>>> = Mutex::new(RefCell::new(None));

use core::panic::PanicInfo;
use core::sync::atomic::{self, Ordering};

#[inline(never)]
#[panic_handler]
fn panic(_info: &PanicInfo) -> ! {
    const GPIOB_BSRR: *mut u32 = 0x5000_0418 as *mut u32;
    const ON: u32 = 1_000;
    const OFF: u32 = 20_000;
    loop {

        unsafe {

            // turn on red led
            ptr::write_volatile(GPIOB_BSRR, 1 << 5);

            for _i in 1..ON {
                atomic::compiler_fence(Ordering::SeqCst);
            }

            // turn off red led
            ptr::write_volatile(GPIOB_BSRR, 1 << 21);

            for _i in 1..OFF {
                atomic::compiler_fence(Ordering::SeqCst);
            }

        }
        // led1.toggle().unwrap();
        // delay.delay_ms(2000u16);
    }
}

// tscal1 address 0x1FFF 75A8 - 0x1FFF 75A9 datasheet 3.13.1
// vref int address 0x1FFF 75AA - 0x1FFF 75AB datasheet 3.13.2

// use core::marker::PhantomData;

// pub struct VREFINT {
//     _marker: PhantomData<*const ()>,
// }

// impl VREFINT {
//     #[doc = r"Returns a pointer to the calibration value"]
//     #[inline(always)]
//     pub const fn ptr() -> *const u16 {
//         0x1FFF_75AA as *const _
//     }
// }
// impl Deref for VREFINT {
//     type Target = u16;
//     #[inline(always)]
//     fn deref(&self) -> &u16 {
//         unsafe { &*VREFINT::ptr() }
//     }
// }
use core::ptr;

// const VREFINT_CAL: *mut u16 = 0x1FFF_75AA as *mut u16;

/// Returns a pointer to VREFINT_CAL, the VREFINT calibration value
#[inline(always)]
const fn vrefint_ptr() -> *const u16 {
    // DS12766 3.13.2
    0x1FFF_75AA as *const _
}

fn calc_vbat(vref_val: u32, vbat_val: u32) -> (u32,u32) {

    // safe because read only memory area
    let vref_cal: u32 = unsafe {
        ptr::read_volatile(vrefint_ptr()) as u32
    };

    // RM0454 14.9 Calculating the actual VDDA voltage using the internal reference voltage
    // V_DDA = 3 V x VREFINT_CAL / VREFINT_DATA
    let vdda_mv = 3000u32 * vref_cal / vref_val;

    // RM0454 14.9 Converting a supply-relative ADC measurement to an absolute voltage value
    //                   Vdda
    // VCHANNELx = --------------- × ADC_DATAx
    //                FULL_SCALE
    let vbat_mv = (vdda_mv * vbat_val / 4095u32) * 2; // x2 because of voltage divide

    (vdda_mv, vbat_mv)
}

#[cfg(feature = "crash")]
#[derive(Debug)]
enum CrashError {
    Intentional = 1
}
#[cfg(feature = "crash")]
/// intentionally crash
/// let _crash = crash().unwrap();
fn crash() -> Result<(), CrashError> {
    Err(CrashError::Intentional)
}

#[derive(Eq, PartialEq)]
enum AppState {
    Idle = 0,
    Triggered = 1,
    Ranging = 2
}

const TRIGGER_TIME: usize = 15;
const RANGING_TIME: usize = 50;

const FLASH_SIZE: u32 = 0x800000;



#[entry]
fn main() -> ! {

    let cp = cortex_m::Peripherals::take().unwrap();
    let dp = stm32::Peripherals::take().expect("cannot take peripherals");

    #[cfg(feature = "low_power")]
    let mut rcc = dp.RCC.freeze(RCCConfig::hsi(Prescaler::Div16));
    #[cfg(feature = "low_power")]
    rcc.enable_low_power_mode();
    #[cfg(not(feature = "low_power"))]
    let mut rcc = dp.RCC.constrain();


    let mut delay = cp.SYST.delay(&mut rcc);

    let gpioa = dp.GPIOA.split(&mut rcc);
    let gpiob = dp.GPIOB.split(&mut rcc);

    let mut led1 = gpiob.pb5.into_push_pull_output(); // red
    let mut led2 = gpiob.pb9.into_push_pull_output(); // green

    led1.set_low().unwrap();
    led2.set_high().unwrap();

    let mut usart = {
        let tx = gpioa.pa9;
        let rx = gpioa.pa10;

        let _baudrate = 9600;
        #[cfg(not(feature = "low_power"))]
        let _baudrate = 500000;

        dp.USART1
        .usart(tx, rx, Config::default().baudrate(_baudrate.bps()), &mut rcc)
        .unwrap()
    };

    writeln!(usart, "Start brievenbusranging!\n").unwrap();

      // Configure the timer.
      let mut timer = dp.TIM1.timer(&mut rcc);
      timer.start(1.khz());
      timer.listen();

      cortex_m::interrupt::free(|cs| {
          *TIMER1.borrow(cs).borrow_mut() = Some(timer);
      });


    let mut storage = {
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
        let flash = Flash::init(spi, flash_cs).unwrap();
        StorageEngine::new(flash, FLASH_SIZE, 0x1000, 0x100)
    };



    // storage.erase(0).unwrap();
    // storage.erase(1).unwrap();


    #[cfg(feature = "use_flash")]
    {
        let mut i = 0u32;
        let n  = FLASH_SIZE / (SensorData::size() + 1);

        led2.set_high().unwrap();
        writeln!(usart, "Wait 3 seconds before umping flash").unwrap();
        delay.delay_ms(3*1000u16);
        led2.set_low().unwrap();

        writeln!(usart, "Start flash dump (max {} lines).", n).unwrap();
        while i < n {
            match storage.read_sensors(i) {
                Ok(s) => {
                    #[allow(unsafe_code)]
                    unsafe {
                        writeln!(usart, "csv{},{},{},{},{},{},{}",
                            s.time_ms,
                            s.photo_t1,
                            s.photo_t2,
                            s.vbat,
                            s.distance,
                            s.trigger_light,
                            s.trigger_distance
                        ).unwrap();
                    }
                },
                Err(_err) => break,
            };
            i += 1;
            if i % 3600 == 0 {
                writeln!(usart, "logged [{}] lines..", i).unwrap();
            }
        }
        writeln!(usart, "Mem dump done").unwrap();
    }

    let mut adc = dp.ADC.constrain(&mut rcc);
    adc.set_sample_time(SampleTime::T_2);
    adc.set_precision(Precision::B_12);

    // photo transistor 1 = PA1
    // photo transistor 2 = PA0
    let mut phototransistor2_pin = gpioa.pa0.into_analog();
    let mut phototransistor1_pin = gpioa.pa1.into_analog();

    let mut vbat_pin = gpiob.pb2.into_analog();

    let mut vref = VRef::new();
    vref.enable(&mut adc);
    let vref_val: u16 = adc.read(&mut vref).expect("adc read failed");
    let vbat_val: u16 = adc.read(&mut vbat_pin).expect("adc read failed");
    let (vdda_mv, vbat_mv) = calc_vbat(vref_val as u32, vbat_val as u32);

    writeln!(usart, "vref: {}, vdd {} mV, vbat: {} mV", vref_val, vdda_mv, vbat_mv).unwrap();






    // I2C pins
    let scl = gpiob.pb6.into_open_drain_output();
    let sda = gpiob.pb7.into_open_drain_output();

    let i2c = dp
        .I2C1
        // for default
        // .i2c(sda, scl, i2c::Config::with_timing(0x2020151b), &mut rcc);
        // for sys_clk = 1MHz => 400kHz
        .i2c(sda, scl, i2c::Config::with_timing(0x00000000), &mut rcc);
        // for sys_clk = 1MHz => 100kHz
        // .i2c(sda, scl, i2c::Config::with_timing(0x00000103), &mut rcc);
        // for sys_clk = 2MHz
        // .i2c(sda, scl, i2c::Config::with_timing(0x00000509), &mut rcc);

    let mut vl6180x = VL6180X::new(i2c).expect("vl6180x failed to intialize");
    writeln!(usart, "VL6180X init done..\n").unwrap();

    vl6180x.start_ranging().unwrap();
    let mut sensordata = SensorData::new();

    let mut light_detector = Detector::new(0.25, TriggerEdge::Both);
    let mut distance_detector = Detector::new(0.10, TriggerEdge::Less);

    #[cfg(feature = "use_flash")]
    {
        let offset = storage.init().unwrap();
        writeln!(usart, "Init flash with offset: {}\n", offset  / (SensorData::size() + 1)).unwrap();
    }

    writeln!(usart, "Enable timer..\n").unwrap();

    #[allow(unsafe_code)]
    unsafe {
        stm32::NVIC::unmask(stm32::Interrupt::TIM1_BRK_UP_TRG_COMP);
    }

    writeln!(usart, "start loop\n").unwrap();

    #[cfg(feature = "full_erase")]
    {
        writeln!(usart, "Erase storage now..").unwrap();
        let start_time = TIME_MS.get();
        match storage.erase_all() {
            Err(err) => {
                led1.set_high().unwrap();
                writeln!(usart, "Error erasing storage\n {:?}", err).unwrap();
            }
            Ok(_) => {
                let total_erase_time = (TIME_MS.get() - start_time) / 1000;
                writeln!(usart, "Erased storage in {} [sec] ", total_erase_time).unwrap();
            }
        }
    }

    #[cfg(feature = "use_flash")]
    {
        writeln!(usart, "Wait 10 seconds before initializing sensors").unwrap();
        delay.delay_ms(10*1000u16);
    }
    led2.set_low().unwrap();

    writeln!(usart, "Initializing averaging buffers..").unwrap();

    while !light_detector.initialized {
        let light: u16 = adc.read(&mut phototransistor2_pin).expect("adc read failed");
        vl6180x.start_ranging().unwrap();
        delay.delay_ms(50_u16);

        let range = vl6180x.read_range().unwrap();
        light_detector.add(light as i32);
        distance_detector.add(range as i32);

        delay.delay_ms(150_u16);
    }

    let distance_av: i32 = distance_detector.mean();
    let light_av: i32 = light_detector.mean();

    writeln!(usart, "Done, distance: {}, light: {} \n start detector loop", distance_av, light_av).unwrap();

    let mut state: AppState = AppState::Idle;
    let mut triggered: usize = 0;
    let mut state_timer: usize = 0;
    let mut light_sample: i32 = 0;
    let mut distance_sample: i32 = 0;

    loop {

        let status = vl6180x.int_status().unwrap();
        if (status & 0b100) == 0b100 {
            let range = vl6180x.read_range().unwrap();

            // writeln!(usart, "range = {} [mm]\n", range).unwrap();
            vl6180x.clear_int().unwrap();
            sensordata.distance(range);

            distance_sample = range as i32;

            writeln!(usart, "plot {}, {}", 0, (range as u16)*10).unwrap();

            if state == AppState::Idle {
                // delay until phototransistors can be used again
                delay.delay_ms(100_u16);
            }
        }

        if state != AppState::Ranging {
            let pt1: u16 = adc.read(&mut phototransistor1_pin).expect("adc read failed");
            let pt2: u16 = adc.read(&mut phototransistor2_pin).expect("adc read failed");

            sensordata.photo_t1(pt1);
            sensordata.photo_t2(pt2);

            light_sample = pt2 as i32;
            writeln!(usart, "plot {}", pt2).unwrap();
        }


        let vref_val: u16 = adc.read(&mut vref).expect("adc read failed");
        let vbat_val: u16 = adc.read(&mut vbat_pin).expect("adc read failed");
        let (vdda_mv, _vbat_mv) = calc_vbat(vref_val as u32, vbat_val as u32);

        sensordata.vbat(vdda_mv as u16);


        match state {
            AppState::Idle => {

                if state_timer == 0 {
                    // first time in state
                    writeln!(usart, "info [Idle] state").unwrap();
                    state_timer += 1;
                }
                // if state_timer == (usize::max_value() - 1) {
                //     // prevent overflow
                //     state_timer = 1;
                // }
                // state_timer += 1;

                light_detector.add(light_sample);

                // if diff > (0.25 * (av as f32)) {
                if light_detector.is_triggered(light_sample) {
                    state = AppState::Triggered;
                    state_timer = 0;
                    sensordata.trigger_light(true);
                    writeln!(usart, "light changed: start ranging").unwrap();
                }

                if distance_sample > 0 {
                    distance_detector.add(distance_sample as i32);
                    distance_sample = 0;
                }

                delay.delay_ms(500_u16);

                if sensordata.properties_set() > 1 {
                    let t: u32 = TIME_MS.get();

                    // every n seconds
                    if (t / 1000) % 2 == 0 {
                        sensordata.time_ms(t);
                        writeln!(usart, "{:?}", sensordata).unwrap();

                        #[cfg(feature = "use_flash")]
                        match storage.write(sensordata) {
                            Err(err) => writeln!(usart, "error while writing to flash: {:?}", err).unwrap(),
                            _ => {}
                        };
                    }

                    // every minute
                    if (t / 1000) % 60 == 0 {
                        vl6180x.start_ranging().unwrap();
                    }


                    sensordata = SensorData::new();
                }
            },
            AppState::Triggered => {

                if state_timer == 0 {
                    // first time in state
                    triggered = 0;
                    writeln!(usart, "warning [Triggered] state").unwrap();
                }

                if light_detector.is_triggered(light_sample) {

                    triggered += 1;
                }

                if sensordata.properties_set() > 1 {
                    let t: u32 = TIME_MS.get();
                    sensordata.time_ms(t);
                    writeln!(usart, "{:?}", sensordata).unwrap();
                    #[cfg(feature = "use_flash")]
                    match storage.write(sensordata) {
                        Err(err) => writeln!(usart, "error while writing to flash: {:?}", err).unwrap(),
                        _ => {}
                    };
                }
                delay.delay_ms(1_u16);

                state_timer += 1;
                if state_timer >= TRIGGER_TIME {
                    state = if triggered > 5 {
                        AppState::Ranging
                    } else {
                        AppState::Idle
                    };
                    state_timer = 0;
                }
            },
            AppState::Ranging => {

                vl6180x.start_ranging().unwrap();

                if state_timer == 0 {
                    // first time in state
                    triggered = 0;
                    writeln!(usart, "err [Ranging] state").unwrap();
                }

                state_timer += 1;
                if state_timer >= RANGING_TIME {
                    state = AppState::Idle;
                    state_timer = 0;
                }
                delay.delay_ms(20_u16);


                if distance_sample > 0 {
                    if distance_detector.is_triggered(distance_sample as i32) {
                        writeln!(usart, "success Detect mail!").unwrap();
                        sensordata.trigger_distance(true);
                    }
                    distance_sample = 0;
                }

                if sensordata.properties_set() > 1 {
                    let t: u32 = TIME_MS.get();
                    sensordata.time_ms(t);
                    writeln!(usart, "{:?}", sensordata).unwrap();
                    #[cfg(feature = "use_flash")]
                    match storage.write(sensordata) {
                        Err(err) => writeln!(usart, "error while writing to flash: {:?}", err).unwrap(),
                        _ => {}
                    };

                    sensordata = SensorData::new();
                }
            }
        }
    }
}



#[interrupt]
fn TIM1_BRK_UP_TRG_COMP() {
    cortex_m::interrupt::free(|cs| {
        if let Some(ref mut tim1) = TIMER1.borrow(cs).borrow_mut().deref_mut() {
            tim1.clear_irq();
            // count 1 millisecond
            TIME_MS.increment(cs);
        }
    });
}