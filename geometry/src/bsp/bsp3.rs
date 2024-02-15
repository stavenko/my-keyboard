use crate::primitives::{
    plane::{self, Plane, SplitResult},
    polygon::Polygon,
    Face,
};

#[derive(Debug, Clone)]
pub struct Bsp3 {
    plane: Plane,
    pub front: Option<Box<Bsp3>>,
    pub back: Option<Box<Bsp3>>,
    pub coplanar_back: Vec<Polygon>,
    pub coplanar_front: Vec<Polygon>,
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

    pub fn union(self, other: Self) -> Self {
        let (_front1, mut back1) = self.clone().clip_by(&other);
        let (_front2, back2) = other.clone().clip_by(&self);

        back1.merge_tree(back2);
        back1
    }

    pub fn diff(self, other: Self) -> Self {
        let (_front1, mut back1) = self.clone().clip_by(&other);
        let (front2, _back2) = other.clone().clip_by(&self);
        dbg!(front2.clone().invert().into_iter().collect_vec());
        dbg!(back1.clone().into_iter().collect_vec());

        back1.merge_tree(front2.invert());
        back1
    }

    pub(crate) fn clip_by(self, mm: &Self) -> (Self, Self) {
        let Self {
            coplanar_front,
            mut coplanar_back,
            mut front,
            mut back,
            plane,
        } = self;
        let mut segments = coplanar_front;
        segments.append(&mut coplanar_back);
        let mut results = mm.clip_polygons(segments);
        let mut front_tree = Self::new(plane.clone());
        let mut back_tree = Self::new(plane);

        front_tree.coplanar_front = results.front;
        front_tree.coplanar_back = results.coplanar_back;

        back_tree.coplanar_back = results.back;
        back_tree.coplanar_front.append(&mut results.coplanar_front);
        //back_tree.coplanar_back.append(&mut results.coplanar_back);

        if let Some(tree) = front.take() {
            let (fftree, bbtree) = tree.clip_by(mm);
            front_tree.front = Some(Box::new(fftree));
            back_tree.front = Some(Box::new(bbtree));
        }
        if let Some(tree) = back.take() {
            let (fftree, bbtree) = tree.clip_by(mm);
            front_tree.back = Some(Box::new(fftree));
            back_tree.back = Some(Box::new(bbtree));
        }
        (front_tree, back_tree)
    }

    fn new(plane: Plane) -> Self {
        Self {
            plane,
            front: None,
            back: None,
            coplanar_front: Vec::new(),
            coplanar_back: Vec::new(),
        }
    }

    /*
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
    */

    fn clip_polygons(&self, lines: Vec<Polygon>) -> SplitResult {
        let (front, back, coplanar_front, coplanar_back) = lines
            .into_iter()
            .map(|segment| self.plane.split_segment(segment))
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

        let mut split_result = SplitResult::default();

        if let Some(tree) = self.front.as_ref() {
            let result = tree.clip_polygons(front);
            split_result = split_result
                .fronts(result.front)
                .backs(result.back)
                .coplanar_fronts(result.coplanar_front)
                .coplanar_backs(result.coplanar_back);

            let result = tree.clip_polygons(coplanar_front);
            split_result = split_result
                .fronts(result.front)
                .backs(result.back)
                .coplanar_fronts(result.coplanar_front)
                .coplanar_backs(result.coplanar_back);
        } else {
            split_result = split_result.fronts(front).coplanar_fronts(coplanar_front);
        }
        if let Some(tree) = self.back.as_ref() {
            let result = tree.clip_polygons(back);
            split_result = split_result
                .fronts(result.front)
                .backs(result.back)
                .coplanar_fronts(result.coplanar_front)
                .coplanar_backs(result.coplanar_back);
            let result = tree.clip_polygons(coplanar_back);
            split_result = split_result
                .fronts(result.front)
                .backs(result.back)
                .coplanar_fronts(result.coplanar_front)
                .coplanar_backs(result.coplanar_back);
        } else {
            split_result = split_result.backs(back).coplanar_backs(coplanar_back);
        }
        split_result
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

    pub fn build(faces: Vec<Polygon>) -> Option<Self> {
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
