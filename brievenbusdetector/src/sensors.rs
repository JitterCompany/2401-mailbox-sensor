use serde::{Serialize, Deserialize};

#[repr(C)]
#[repr(packed(2))]
#[derive(Serialize, Deserialize, Copy, Clone, Debug)]
pub struct SensorData {
    pub time_ms: u32,
    pub photo_t1: u16,
    pub photo_t2: u16,
    pub vbat: u16,
    pub distance: u16,
    pub trigger_light: bool,
    pub trigger_distance: bool
}

const UNINITIALIZED: u16 = 0xFFFF;

impl SensorData {

    pub fn new() -> Self {
        Self {
            time_ms: 0,
            photo_t1: UNINITIALIZED,
            photo_t2: UNINITIALIZED,
            vbat: UNINITIALIZED,
            distance: UNINITIALIZED,
            trigger_light: false,
            trigger_distance: false
        }
    }

    pub const fn size() -> u32 {
        //size_of::<SensorData>() as u32 // not const according to rustc...
        4+6*2
    }

    pub fn properties_set(&self) -> u32 {
        let mut count = 0;

        if self.photo_t1 != UNINITIALIZED {
            count += 1;
        }
        if self.photo_t2 != UNINITIALIZED {
            count += 1;
        }
        if self.vbat != UNINITIALIZED {
            count += 1;
        }
        if self.distance != UNINITIALIZED {
            count += 1;
        }

        count
    }

    pub fn time_ms(&mut self, val: u32) {
        self.time_ms = val;
    }
    pub fn photo_t1(&mut self, val: u16) {
        self.photo_t1 = val;
    }
    pub fn photo_t2(&mut self, val: u16) {
        self.photo_t2 = val;
    }
    pub fn vbat(&mut self, val: u16) {
        self.vbat = val;
    }
    pub fn distance(&mut self, val: u8) {
        self.distance = val as u16;
    }
    pub fn trigger_light(&mut self, trigger: bool) {
        self.trigger_light = trigger;
    }
    pub fn trigger_distance(&mut self, trigger: bool) {
        self.trigger_distance = trigger;
    }
}
