#![allow(dead_code)]

use spi_memory::{BlockDevice, Read};
use embedded_hal::blocking::spi;
use embedded_hal::digital::v2::OutputPin;
use core::marker::PhantomData;

enum EraseMode {
    Never = 0,        // when full, all attempted writes fail
}

pub struct StorageEngine<CHIP, SPI, CS> {
    flash: CHIP,
    size_bytes: usize,
    block_size_bytes: usize,
    page_size_bytes: usize,
    offset: usize,
    erase_mode: EraseMode,
    _a: PhantomData<SPI>,
    _b: PhantomData<CS>,
}


impl<CHIP, SPI, CS, E, PinError> StorageEngine<CHIP, SPI, CS>
where
    SPI: spi::Transfer<u8, Error=E>,
    CS: OutputPin<Error = PinError>,
    CHIP: BlockDevice<u32, SPI, CS> + Read<u32, SPI, CS>
{

    pub fn new(flash: CHIP, size: usize, block_size: usize, page_size: usize) -> Self {

        Self {
            flash: flash,
            size_bytes: size,
            block_size_bytes: block_size,
            page_size_bytes: page_size,
            offset: 0,
            erase_mode: EraseMode::Never,
            _a: PhantomData,
            _b: PhantomData
        }
    }
}

