use nalgebra::Vector3;

use super::path::Path;

pub struct Polyline {
    path: Vec<Box<dyn Path>>,
    lengths: Vec<f32>,
}

pub struct PolylineBuilder {
    path: Vec<Box<dyn Path>>,
    lengths: Vec<f32>,
}

impl Polyline {
    pub(crate) fn elements(&self) -> usize {
        self.path.len()
    }

    pub(crate) fn as_segments<'a>(
        &'a self,
        segments: usize,
    ) -> anyhow::Result<Box<dyn Iterator<Item = [Vector3<f32>; 2]> + 'a>> {
        if segments <= self.lengths.len() {
            Ok(Box::new(self.lengths.iter().scan(0.0, |start, l| {
                let start_stop = [self.get_t(*start).ok()?, self.get_t(*l).ok()?];
                *start = *l;
                Some(start_stop)
            })))
        } else {
            Ok(Box::new(
                self.lengths
                    .iter()
                    .scan(0.0, move |start: &mut f32, l: &f32| {
                        let average_len = 1.0 / segments as f32;
                        let segments_on_this_item = (*l / average_len).floor().max(1.0) as u32;
                        let segment_len = l / segments_on_this_item as f32;
                        let mut lines = Vec::new();
                        for segment in 0..(segments_on_this_item - 1) {
                            let seg = segment as f32;
                            let next_seg = seg + 1.0;
                            lines.push([
                                self.get_t(*start + seg * segment_len).ok()?,
                                self.get_t(*start + next_seg * segment_len).ok()?,
                            ]);
                        }
                        Some(lines)
                    })
                    .flatten(),
            ))
        }
    }

    pub(crate) fn new(items: Vec<Box<dyn Path>>) -> anyhow::Result<Self> {
        let mut peekable = items.iter().peekable();
        let mut lengths = Vec::new();
        while let Some(item) = peekable.next() {
            lengths.push(item.len());
            if let Some(next) = peekable.peek() {
                if item.last() != next.first() {
                    return Err(anyhow::Error::msg("path items not chained"));
                }
            }
        }
        let total = lengths.iter().fold(0.0, |a, b| a + b);
        Ok(Self {
            path: items,
            lengths: lengths.into_iter().map(|l| l / total).collect(),
        })
    }

    pub(crate) fn get_t(&self, t: f32) -> anyhow::Result<Vector3<f32>> {
        if t <= 1.0 && t >= 0.0 {
            Err(anyhow::Error::msg("t shall be between 0 and 1"))
        } else {
            let mut rest = t;
            for (ix, l) in self.lengths.iter().enumerate() {
                if rest > *l {
                    rest -= l;
                } else {
                    let item_t = rest / l;
                    if let Some(item) = self.path.iter().nth(ix) {
                        return Ok(item.get_t(item_t));
                    }
                }
            }
            Err(anyhow::Error::msg(
                "t is still too big, impossible to get item vector",
            ))
        }
    }
}
