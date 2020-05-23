#![allow(dead_code)]

use spi_memory::{BlockDevice, Read, self};
use embedded_hal::blocking::spi;
use embedded_hal::digital::v2::OutputPin;
use core::marker::PhantomData;

use crate::sensors::SensorData;

use postcard::{self, from_bytes, to_vec};
use heapless::{Vec, consts::*};

use core::cmp::min;
use core::fmt::{self, Debug};

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
    _spi: PhantomData<SPI>,
    _cs: PhantomData<CS>,
}

pub enum Error<SPI: spi::Transfer<u8>, CS: OutputPin> {

    /// Errors from spi-memory crate (spi, gpio, protocol)
    SpiMem(spi_memory::Error<SPI, CS>),

    /// Packet header is not as expected
    FlashFull,

    /// Packet header is not as expected
    CorruptPacket,

    /// Address out of bounds
    OutOfBounds,

    /// postcard error
    Pack(postcard::Error),

    /// Hints that destructuring should not be exhaustive.
    ///
    /// This enum may grow additional variants, so this makes sure clients
    /// don't count on exhaustive matching. (Otherwise, adding a new variant
    /// could break existing code.)
    #[doc(hidden)]
    __Nonexhaustive,
}

impl<SPI: spi::Transfer<u8>, CS: OutputPin> core::convert::From<spi_memory::Error<SPI, CS>> for Error<SPI, CS> {
    fn from(error: spi_memory::Error<SPI, CS>) -> Self {
        Error::SpiMem(error)
    }
}

impl<SPI: spi::Transfer<u8>, CS: OutputPin> core::convert::From<postcard::Error> for Error<SPI, CS> {
    fn from(error: postcard::Error) -> Self {
        Error::Pack(error)
    }
}

impl<SPI: spi::Transfer<u8>, CS: OutputPin> fmt::Debug for Error<SPI, CS>
where
    SPI::Error: Debug,
    CS::Error: Debug,
{

    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::FlashFull => write!(f, "Error::FlashFull"),
            Error::OutOfBounds => write!(f, "Error::OutOfBounds"),
            Error::CorruptPacket => write!(f, "Error::CorruptPacket"),
            Error::Pack(pack) => write!(f, "Error::postcard: {:?}", pack),
            Error::SpiMem(spi_mem) => write!(f, "Error::spi-memory: {:?}", spi_mem),
            Error::__Nonexhaustive => unreachable!(),
        }
    }
}

// #[derive(Debug)]
// enum Error<E> {
//     FlashFull,
//     CorruptPacket,
//     SpiMem(E),
// }

// impl<E> core::convert::From<E> for Error<E> {
//     fn from(error: E) -> Self {
//         Error::SpiMem(error)
//     }
// }



impl<CHIP, SPI, CS, CommError, PinError> StorageEngine<CHIP, SPI, CS>
where
    SPI: spi::Transfer<u8, Error=CommError>,
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
            _spi: PhantomData,
            _cs: PhantomData
        }
    }


//Error<<SpiDevice<SPI, NCS> as device::Device>::Error>>

    pub fn init(&mut self) -> Result<u32, Error<SPI, CS>> {

        let packet_size: u32 = SensorData::size() + 1;

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

    pub fn erase(&mut self, block: u32) -> Result<(), Error<SPI, CS>> {
        self.flash.erase_sectors(block, 1 as usize).map_err(Error::SpiMem)
    }

    pub fn write(&mut self, sensor_data: SensorData)  -> Result<u32, Error<SPI, CS>> {

        let vec: Vec<u8, U26> = to_vec(&sensor_data).unwrap(); // todo: no unwrap

        let mut packet = Vec::<u8, U27>::new();
        // add packet prefix
        packet.push(PACKET_HEADER).unwrap();
        // append data
        packet.extend_from_slice(&vec).unwrap();

        let mut remaining: u32 = packet.len() as u32;
        if (self.waddr + remaining) > (self.size_bytes - 1) {
            // todo throw error, flash full
            return Err(Error::FlashFull);
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


    pub fn read(&mut self, addr: u32, buffer: &mut [u8]) -> Result<(), Error<SPI, CS>>{

        if addr > self.size_bytes {
            return Err(Error::OutOfBounds);
        }
        self.flash.read(addr, buffer)?;
        Ok(())
    }

    pub fn read_sensors(&mut self, index: u32) -> Result<SensorData, Error<SPI, CS>> {

        const ELEM_SIZE: u32 = SensorData::size() + 1;
        let address = index * ELEM_SIZE;

        let mut buf: [u8; ELEM_SIZE as usize] = [0; ELEM_SIZE as usize];
        self.read(address, &mut buf)?;

        if buf[0] == PACKET_HEADER {
            let deserialized: SensorData = from_bytes(&buf[1..])?;
            return Ok(deserialized);
        }
        Err(Error::CorruptPacket)

    }
}

