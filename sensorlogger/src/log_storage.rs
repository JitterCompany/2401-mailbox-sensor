#![allow(dead_code)]

use spi_memory::{BlockDevice, Read};
use embedded_hal::blocking::spi;
use embedded_hal::digital::v2::OutputPin;
use core::marker::PhantomData;

use crate::sensors::SensorData;

use postcard::{from_bytes, to_vec};
use heapless::{Vec, consts::*};

use core::mem::size_of;

enum EraseMode {
    Never = 0,        // when full, all attempted writes fail
}

const PACKET_HEADER: u8 = 0xE0;

pub struct StorageEngine<CHIP, SPI, CS> {
    flash: CHIP,
    size_bytes: u32,
    block_size_bytes: u32,
    page_size_bytes: u32,
    offset: u32,
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

    pub fn new(flash: CHIP, size: u32, block_size: u32, page_size: u32) -> Self {

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


    pub fn init(&mut self) -> Result<u32, spi_memory::Error<SPI, CS>> {

        let packet_size: u32 = (size_of::<SensorData>() as u32) + 1;

        let mut offset: u32 = 0;

        let mut buf: [u8; 1] = [0];
        while offset < self.size_bytes {
            self.read(offset, &mut buf)?;

            if buf[0] != PACKET_HEADER {
                // we're done
                break;
            }

            offset += packet_size;
        }

        self.offset = offset;

        Ok(offset)
    }

    pub fn erase(&mut self, block: u32) -> Result<(), spi_memory::Error<SPI, CS>> {
        self.flash.erase_sectors(block, 1 as usize)
    }

    pub fn write(&mut self, sensor_data: SensorData)  -> Result<Vec<u8, U27>, spi_memory::Error<SPI, CS>> {

        let vec: Vec<u8, U26> = to_vec(&sensor_data).unwrap(); // todo: no unwrap

        let mut packet = Vec::<u8, U27>::new();
        // add packet prefix
        packet.push(PACKET_HEADER).unwrap();
        // append data
        packet.extend_from_slice(&vec).unwrap();

        self.flash.write_bytes(self.offset, &mut packet)?;

        self.offset += packet.len() as u32;

        Ok(packet)
    }


    pub fn read(&mut self, offset: u32, buffer: &mut [u8]) -> Result<(), spi_memory::Error<SPI, CS>>{

        if offset > self.size_bytes {
            // todo throw error
        }
        self.flash.read(offset, buffer)
    }
}

