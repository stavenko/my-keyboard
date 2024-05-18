use core::fmt;
use std::{
    fmt::{Debug, Display},
    ops::{AddAssign, Div, Sub},
};

use num_traits::{One, Zero};

pub trait SplitHyperLine<Scalar> {
    fn split_hyper_line(&self, t: Scalar) -> (Self, Self)
    where
        Self: Sized;

    fn split_by_weights(&self, weights: Vec<Scalar>) -> Vec<Self>
    where
        Self: Sized + fmt::Debug,
        Scalar: Zero,
        Scalar: One,
        Scalar: Sub<Output = Scalar>,
        Scalar: Div<Output = Scalar>,
        Scalar: AddAssign,
        Scalar: Copy + Display + Debug,
    {
        let total = weights.iter().fold(Scalar::zero(), |a, b| a + *b);
        let mut sum = Scalar::zero();
        let wlen = weights.len();
        let new_weights = weights
            .into_iter()
            .map(|w| {
                let l = w / total;
                let p = l + sum;
                sum += l;
                p
            })
            .take(wlen - 1)
            .collect();
        self.split_by(new_weights)
    }

    fn split_by(&self, cfs: Vec<Scalar>) -> Vec<Self>
    where
        Self: Sized + fmt::Debug,
        Scalar: One,
        Scalar: Sub<Output = Scalar>,
        Scalar: Div<Output = Scalar>,
        Scalar: AddAssign,
        Scalar: Copy + Display + Debug,
    {
        let mut modified_ts = vec![cfs[0]];
        let mut modified_lens = vec![Scalar::one()];
        for i in 1..cfs.len() {
            let ls = cfs[i] - cfs[i - 1];
            modified_ts.push(ls)
        }

        for cf in cfs.iter().take(cfs.len() - 1) {
            modified_lens.push(Scalar::one() - *cf);
        }

        let (mut initial, _) = self.split_hyper_line(Scalar::one());
        let mut result = Vec::new();
        // dbg!(&modified_ts);
        for (t, m) in modified_ts.into_iter().zip(modified_lens) {
            let (item, rest) = initial.split_hyper_line(t / m);
            result.push(item);
            initial = rest;
        }

        result.push(initial);
        result
    }
}
