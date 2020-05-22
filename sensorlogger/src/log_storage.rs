#![allow(dead_code)]

use spi_memory::{BlockDevice, Read};
use embedded_hal::blocking::spi;
use embedded_hal::digital::v2::OutputPin;
use core::marker::PhantomData;

use crate::sensors::SensorData;

use postcard::{from_bytes, to_vec};
use heapless::{Vec, consts::*};

use core::mem::size_of;
use core::cmp::min;

enum EraseMode {
    Never = 0,        // when full, all attempted writes fail
}

const PACKET_HEADER: u8 = 0xE0;

pub struct StorageEngine<CHIP, SPI, CS> {
    flash: CHIP,
    size_bytes: u32,
    erase_block_size_bytes: u32,
    page_size_bytes: u32,
    waddr: u32,
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

    pub fn new(flash: CHIP, size: u32, erase_block_size: u32, page_size: u32) -> Self {

        Self {
            flash: flash,
            size_bytes: size,
            erase_block_size_bytes: erase_block_size,
            page_size_bytes: page_size,
            waddr: 0,
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

        self.waddr = offset;

        Ok(offset)
    }

    pub fn erase(&mut self, block: u32) -> Result<(), spi_memory::Error<SPI, CS>> {
        self.flash.erase_sectors(block, 1 as usize)
    }

    pub fn write(&mut self, sensor_data: SensorData)  -> Result<u32, spi_memory::Error<SPI, CS>> {

        let vec: Vec<u8, U26> = to_vec(&sensor_data).unwrap(); // todo: no unwrap

        let mut packet = Vec::<u8, U27>::new();
        // add packet prefix
        packet.push(PACKET_HEADER).unwrap();
        // append data
        packet.extend_from_slice(&vec).unwrap();

        let mut remaining: u32 = packet.len() as u32;
        if (self.waddr + remaining) > (self.size_bytes - 1) {
            // todo throw error, flash full
            return Ok(self.size_bytes);
        }
        let mut buffer_offset: usize = 0;
        while remaining > 0 {

            let page_offset: u32 = self.waddr & (self.page_size_bytes-1);
            let to_write = min(remaining, self.page_size_bytes as u32 - page_offset) as usize;
            let end = buffer_offset + to_write;
            self.flash.write_bytes(self.waddr, &mut packet[buffer_offset..end])?;

            self.waddr += to_write as u32;
            buffer_offset += to_write;
            remaining -= to_write as u32;
        }

        Ok(self.waddr)
    }


    pub fn read(&mut self, addr: u32, buffer: &mut [u8]) -> Result<(), spi_memory::Error<SPI, CS>>{

        if addr > self.size_bytes {
            // todo throw error
        }
        self.flash.read(addr, buffer)
    }
}

