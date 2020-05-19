#![allow(dead_code)]

use spi_memory::{BlockDevice, Read, series25};
use embedded_hal::blocking::spi;
use embedded_hal::blocking::spi::Transfer;
use embedded_hal::digital::v2::OutputPin;
use core::marker::PhantomData;

// pub struct StorageEngine<CHIP> {
//     flash: CHIP,
//     size_bytes: usize,
//     block_size_bytes: usize,
//     page_size_bytes: usize,
//     offset: usize,
//     erase_mode: EraseMode
// }

// impl<SPI, CS, CHIP, PinError> StorageEngine<CHIP>
// where
//     SPI: spi::Transfer<u8>,
//     CS: OutputPin<Error = PinError>,
//     CHIP: BlockDevice<u8, SPI, CS>
// {

// }

pub struct StorageEngine<CHIP, SPI, CS> {
    flash: CHIP,
    _a: PhantomData<SPI>,
    _b: PhantomData<CS>,
}


impl<CHIP, SPI, CS, E, PinError> StorageEngine<CHIP, SPI, CS>
where
    SPI: spi::Transfer<u8, Error=E>,
    CS: OutputPin<Error = PinError>,
    CHIP: BlockDevice<u32, SPI, CS> + Read<u32, SPI, CS>
{

    pub fn new(flash: CHIP) -> Self {

        Self {
            flash: flash,
            _a: PhantomData,
            _b: PhantomData
        }
    }
}

// pub struct StorageEngine<SPI: Transfer<u8>, CS: OutputPin, CHIP: BlockDevice<u8, SPI, CS>> {

enum EraseMode {
    Never = 0,        // when full, all attempted writes fail
}

// impl<'a, Flash> FlashStorage<'a, Flash> {

//     pub fn new(flash: &'a Flash, size: usize, block_size: usize, page_size: usize) -> Self {

//         Self {
//             flash,
//             size_bytes: size,
//             block_size_bytes: block_size,
//             page_size_bytes: page_size,
//             offset: 0,
//             erase_mode: EraseMode::Never
//         }
//     }
// }