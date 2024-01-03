use nalgebra::{Vector2, Vector3};

pub mod origin;
pub mod plane;
pub type Face = [Vector3<f32>; 3];

#[derive(Clone)]
pub struct PointInPlane<T> {
    pub point: Vector3<T>,
    pub normal: Vector3<T>,
    pub dir: Option<Vector3<T>>,
}

pub struct Segments {
    segments: usize,
    current_segment: usize,
}

/*
pub struct QuadSurfaceSegments {
    segments_r: Segments,
    segments_c: Segments,
    cur_r: Option<Segment<f32>>,
    cur_c: Option<Segment<f32>>,
}

impl QuadSurfaceSegments {
    pub fn new(segments_r: usize, segments_c: usize) -> Self {
        Self {
            segments_r: Segments::new(segments_r),
            segments_c: Segments::new(segments_c),
            cur_r: None,
            cur_c: None,
        }
    }
}
*/

pub struct IndexIterator<const D: usize>(usize);

impl<const D: usize> Iterator for IndexIterator<D> {
    type Item = (usize, usize);

    fn next(&mut self) -> Option<Self::Item> {
        if self.0 < D - 1 {
            self.0 += 1;
            Some((self.0 - 1, self.0))
        } else {
            None
        }
    }
}

impl<const D: usize> IndexIterator<D> {
    pub fn new() -> Self {
        Self(0)
    }
}

impl Segments {
    pub(crate) fn new(segments: usize) -> Self {
        Self {
            segments,
            current_segment: 0,
        }
    }
}

impl Iterator for Segments {
    type Item = (f32, f32);

    fn next(&mut self) -> Option<Self::Item> {
        let first = self.current_segment;
        let next = first + 1;
        self.current_segment += 1;
        if next > self.segments {
            None
        } else {
            let first = first as f32 / self.segments as f32;
            let next = next as f32 / self.segments as f32;
            Some((first, next))
        }
    }
}
#[derive(Debug, PartialEq)]
pub struct Segment<T> {
    pub from: T,
    pub to: T,
}
#[cfg(test)]
mod tests {
    use super::Segments;

    #[test]
    fn simple_segments() {
        let s = Segments::new(2).collect::<Vec<_>>();
        assert_eq!(s, vec!((0.0, 0.5), (0.5, 1.0)));
        let s = Segments::new(3).collect::<Vec<_>>();
        assert_eq!(
            s,
            vec![(0.0, 0.33333334), (0.33333334, 0.6666667), (0.6666667, 1.0)]
        );
    }
}
