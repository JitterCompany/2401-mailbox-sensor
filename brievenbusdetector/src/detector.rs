

const AVERAGING_LEN: usize = 50;

#[allow(unused)]
pub enum TriggerEdge {
    Less,
    Greater,
    Both
}
pub struct Detector {
    array: [i32; AVERAGING_LEN],
    index: usize,
    pub initialized: bool,
    trigger_level: f32,
    edge: TriggerEdge
}


impl Detector {

    pub fn new(trigger_level: f32, edge: TriggerEdge) -> Self {
        Self {
            array: [0; AVERAGING_LEN],
            index: 0,
            initialized: false,
            trigger_level,
            edge
        }
    }

    pub fn add(&mut self, value: i32) {
        self.array[self.index] = value;
        self.index += 1;
        self.index %= AVERAGING_LEN;
        if self.index == 0 {
            self.initialized = true;
        }
    }

    pub fn mean(&self) -> i32 {
        self.array.iter().fold(0, |s, &x| s+x) / (AVERAGING_LEN as i32)
    }

    pub fn is_triggered(&self, val: i32) -> bool {

        match self.edge {
            TriggerEdge::Less => {
                let threshold: i32 = (self.mean() as f32 * (1.0-self.trigger_level) as f32) as i32;
                val  < threshold
            },
            TriggerEdge::Greater => {

                let threshold: i32 = (self.mean() as f32 * (1.0+self.trigger_level) as f32) as i32;
                val > threshold
            },
            TriggerEdge::Both => {
                let mean = self.mean();
                let diff: f32 = (mean - val).abs() as f32;
                diff > (self.trigger_level * (mean as f32))
            }
        }



    }
}


