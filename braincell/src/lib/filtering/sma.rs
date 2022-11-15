use num_traits::{Num, NumAssignOps, NumCast, Signed};

pub struct SmaFilter<ItemT, const SIZE: usize> {
    buff: [ItemT; SIZE],
    idx: usize,
    sum: ItemT,
    full: bool,
}
impl<ItemT, const SIZE: usize> SmaFilter<ItemT, SIZE>
where
    ItemT: Num + NumAssignOps + NumCast + Signed + core::marker::Copy,
{
    pub fn new() -> SmaFilter<ItemT, SIZE> {
        return SmaFilter::default();
    }

    pub fn reset(&mut self) {
        self.buff = [ItemT::zero(); SIZE];
        self.idx = 0;
        self.sum = ItemT::zero();
        self.full = false;
    }

    pub fn insert(&mut self, data: ItemT) {
        self.sum += data - self.buff[self.idx];
        self.buff[self.idx] = data;
        self.idx = (self.idx + 1) % SIZE;

        if self.idx == 0 {
            // we have looped around at least once
            self.full = true;
        }
    }

    pub fn filtered(&self) -> Option<ItemT> {
        match self.full {
            true => Some(self.sum / ItemT::from(SIZE).unwrap()),
            false => None,
        }
    }
}

impl<ItemT, const SIZE: usize> Default for SmaFilter<ItemT, SIZE>
where
    ItemT: Num + NumAssignOps + NumCast + core::marker::Copy,
{
    fn default() -> SmaFilter<ItemT, SIZE> {
        SmaFilter {
            buff: [ItemT::zero(); SIZE],
            idx: 0,
            sum: ItemT::zero(),
            full: false,
        }
    }
}
