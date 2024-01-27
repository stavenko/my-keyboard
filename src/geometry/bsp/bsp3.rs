use crate::geometry::primitives::{plane::Plane, polygon::Polygon, Face};

#[derive(Debug)]
pub struct Bsp3 {
    plane: Plane,
    pub front: Option<Box<Bsp3>>,
    pub back: Option<Box<Bsp3>>,
    pub faces: Vec<Polygon>,
}

impl Bsp3 {
    fn faces_amount(&self) -> usize {
        let mut amount = self.faces.len();
        if let Some(f) = self.front.as_ref().map(|f| f.faces_amount()) {
            amount += f;
        }
        if let Some(f) = self.back.as_ref().map(|f| f.faces_amount()) {
            amount += f;
        }
        amount
    }

    pub(crate) fn clip_by(mut self, mm: &Bsp3) -> Self {
        self.faces = mm.clip_polygons(self.faces);
        if let Some(tree) = self.front.take() {
            self.front = Some(Box::new(tree.clip_by(mm)));
        }
        if let Some(tree) = self.back.take() {
            self.back = Some(Box::new(tree.clip_by(mm)));
        }
        self
    }

    pub(crate) fn invert(mut self) -> Self {
        self.faces = self.faces.into_iter().map(|f| f.flip()).collect();
        self.plane = self.plane.flip();
        let back = self.front.take().map(|tree| Box::new(tree.invert()));
        let front = self.back.take().map(|tree| Box::new(tree.invert()));

        self.front = front;
        self.back = back;

        self
    }

    fn clip_polygons(&self, faces: Vec<Polygon>) -> Vec<Polygon> {
        let (mut front, mut back, mut coplanar_front, mut coplanar_back) = faces
            .into_iter()
            .map(|face| self.plane.split_polygon(face))
            .fold(
                (Vec::new(), Vec::new(), Vec::new(), Vec::new()),
                |(mut front, mut back, mut coplanar_front, mut coplanar_back), mut split| {
                    front.append(&mut split.front);
                    back.append(&mut split.back);
                    coplanar_front.append(&mut split.coplanar_front);
                    coplanar_back.append(&mut split.coplanar_back);

                    (front, back, coplanar_front, coplanar_back)
                },
            );
        front.append(&mut coplanar_front);
        back.append(&mut coplanar_back);

        if let Some(tree) = self.front.as_ref() {
            front = tree.clip_polygons(front);
        }
        if let Some(tree) = self.back.as_ref() {
            back = tree.clip_polygons(back);
        }
        front.append(&mut back);
        front
    }

    pub(crate) fn calculate_triangles(&self) -> usize {
        let ff: usize = self.faces.iter().map(|p| p.calculate_triangles()).sum();
        ff + self
            .front
            .as_ref()
            .map(|tree| tree.calculate_triangles())
            .unwrap_or(0usize)
            + self
                .back
                .as_ref()
                .map(|tree| tree.calculate_triangles())
                .unwrap_or(0usize)
    }
}

pub struct FacesBspIterator<I>
where
    I: Iterator<Item = Face>,
{
    len: usize,
    inner: I,
}

pub struct PolyBspIterator<I>
where
    I: Iterator<Item = Polygon>,
{
    len: usize,
    inner: I,
}

impl<I> ExactSizeIterator for PolyBspIterator<I> where I: Iterator<Item = Polygon> {}

impl<I> Iterator for PolyBspIterator<I>
where
    I: Iterator<Item = Polygon>,
{
    type Item = Polygon;

    fn next(&mut self) -> Option<Self::Item> {
        self.inner.next()
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        (self.len, Some(self.len))
    }
}

impl IntoIterator for Bsp3 {
    type Item = Polygon;

    type IntoIter = PolyBspIterator<Box<dyn Iterator<Item = Polygon>>>;

    fn into_iter(self) -> Self::IntoIter {
        let len = self.faces_amount();
        let mut my: Box<dyn Iterator<Item = Polygon>> = Box::new(self.faces.into_iter());
        if let Some(fronts) = self.front {
            my = Box::new(my.chain(fronts.into_iter()));
        }
        if let Some(backs) = self.back {
            my = Box::new(my.chain(backs.into_iter()));
        }

        PolyBspIterator {
            inner: Box::new(my),
            len,
        }
    }
}

impl Bsp3 {
    pub fn merge(&mut self, other: impl IntoIterator<Item = Polygon>) {
        let (front, back, mut coplanar_front, mut coplanar_back) =
            other.into_iter().map(|f| self.plane.split_polygon(f)).fold(
                (Vec::new(), Vec::new(), Vec::new(), Vec::new()),
                |(mut front, mut back, mut coplanar_front, mut coplanar_back), mut split| {
                    front.append(&mut split.front);
                    back.append(&mut split.back);
                    coplanar_front.append(&mut split.coplanar_front);
                    coplanar_back.append(&mut split.coplanar_back);

                    (front, back, coplanar_front, coplanar_back)
                },
            );
        if !front.is_empty() {
            if let Some(tree) = self.front.as_mut() {
                tree.merge(front);
            } else {
                self.front = Self::build(front).map(Box::new);
            }
        }
        if !back.is_empty() {
            if let Some(tree) = self.back.as_mut() {
                tree.merge(back);
            } else {
                self.back = Self::build(back).map(Box::new);
            }
        }
        self.faces.append(&mut coplanar_back);
        self.faces.append(&mut coplanar_front);
    }
    pub fn build(faces: impl IntoIterator<Item = Polygon>) -> Option<Self> {
        let mut iter = faces.into_iter();
        let face = iter.next();
        face.and_then(|face| {
            let plane = face.get_plane().ok()?;
            let (front, back, mut coplanar_front, mut coplanar_back) =
                iter.map(|f| plane.split_polygon(f)).fold(
                    (Vec::new(), Vec::new(), vec![face], Vec::new()),
                    |(mut front, mut back, mut coplanar_front, mut coplanar_back), mut split| {
                        front.append(&mut split.front);
                        back.append(&mut split.back);
                        coplanar_front.append(&mut split.coplanar_front);
                        coplanar_back.append(&mut split.coplanar_back);

                        (front, back, coplanar_front, coplanar_back)
                    },
                );
            let front = if front.is_empty() {
                None
            } else {
                Some(Box::new(Bsp3::build(front)?))
            };
            let back = if back.is_empty() {
                None
            } else {
                Some(Box::new(Bsp3::build(back)?))
            };
            let mut coplanar_front = Self::join_coplanars(coplanar_front);
            let mut coplanar_back = Self::join_coplanars(coplanar_back);

            coplanar_front.append(&mut coplanar_back);

            Some(Self {
                plane,
                front,
                back,
                faces: coplanar_front,
            })
        })
    }
    fn join_coplanars(polys: Vec<Polygon>) -> Vec<Polygon> {
        if polys.len() > 1 {
            let saved = polys.clone();
            let mut iter = polys.into_iter();
            let mut acc = iter.next();
            for item in iter {
                if let Ok(f) = acc.take().map(|acc| acc.join(item)).transpose() {
                    acc = f
                } else {
                    acc = None;
                    break;
                }
            }
            if let Some(acc) = acc {
                vec![acc]
            } else {
                saved
            }
        } else {
            polys
        }
    }
}
