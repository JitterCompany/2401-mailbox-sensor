use serde::{Serialize, Deserialize};

#[repr(C)]
#[repr(packed(2))]
#[derive(Serialize, Deserialize, Copy, Clone, Debug)]
pub struct SensorData {
    pressure: f32,
    temp: f32,
    accel_x: i16,
    accel_y: i16,
    accel_z: i16,
    photo_t1: u16,
    photo_t2: u16,
    photo_d1: u16,
    photo_d2: u16,
    vbat: u16,
    distance: u16,
}


impl SensorData {

    pub fn new() -> Self {
        Self {
            pressure: 0.0,
            temp: 0.0,
            accel_x: 0,
            accel_y: 0,
            accel_z: 0,
            photo_t1: 0,
            photo_t2: 0,
            photo_d1: 0,
            photo_d2: 0,
            vbat: 0,
            distance: 0,
        }
    }

    pub const fn size() -> u32 {
        //size_of::<SensorData>() as u32 // not const accorindg to rustc...
        26
    }

    pub fn ready(&self) -> bool {
        let mut count = 0;
        if self.pressure != 0.0 {
            count += 1;
        }
        if self.temp != 0.0 {
            count += 1;
        }
        if self.accel_x != 0 {
            count += 1;
        }
        if self.accel_y != 0 {
            count += 1;
        }
        if self.accel_z != 0 {
            count += 1;
        }
        if self.photo_t1 != 0 {
            count += 1;
        }
        if self.photo_t2 != 0 {
            count += 1;
        }
        if self.photo_d1 != 0 {
            count += 1;
        }
        if self.photo_d2 != 0 {
            count += 1;
        }
        if self.vbat != 0 {
            count += 1;
        }
        if self.distance != 0 {
            count += 1;
        }

        count == 11
    }

    pub fn pressure(&mut self, val: f32) {
        self.pressure = val;
    }
    pub fn temp(&mut self, val: f32) {
        self.temp = val;
    }
    pub fn accel_x(&mut self, val: i16) {
        self.accel_x = val;
    }
    pub fn accel_y(&mut self, val: i16) {
        self.accel_y = val;
    }
    pub fn accel_z(&mut self, val: i16) {
        self.accel_z = val;
    }
    pub fn photo_t1(&mut self, val: u16) {
        self.photo_t1 = val;
    }
    pub fn photo_t2(&mut self, val: u16) {
        self.photo_t2 = val;
    }
    pub fn photo_d1(&mut self, val: u16) {
        self.photo_d1 = val;
    }
    pub fn photo_d2(&mut self, val: u16) {
        self.photo_d2 = val;
    }
    pub fn vbat(&mut self, val: u16) {
        self.vbat = val;
    }
    pub fn distance(&mut self, val: u8) {
        self.distance = val as u16;
    }
}
