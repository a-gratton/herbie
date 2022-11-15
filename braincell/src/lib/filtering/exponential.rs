use num_traits::{Num, NumAssignOps, NumCast, Signed};

pub struct ExponentialFilter<ItemT> {
    alpha: ItemT,
    prev: ItemT,
}
impl<ItemT> ExponentialFilter<ItemT>
where
    ItemT: Num + NumAssignOps + NumCast + Signed + core::marker::Copy,
{
    pub fn new(alpha: ItemT) -> ExponentialFilter<ItemT> {
        ExponentialFilter {
            alpha: alpha,
            prev: ItemT::zero(),
        }
    }

    pub fn reset(&mut self) {
        self.prev = ItemT::zero();
    }

    pub fn update(&mut self, data: ItemT) -> ItemT {
        self.prev = self.alpha * data + (ItemT::one() - self.alpha) * self.prev;
        self.prev
    }
}
