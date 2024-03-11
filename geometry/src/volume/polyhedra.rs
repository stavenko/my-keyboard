use crate::planar::polygon::Polygon;

pub struct Polyhedra {
    pub polygons: Vec<Polygon>,
}

impl Polyhedra {
    pub fn polygons(&self) -> Vec<Polygon> {
        self.polygons.clone()
    }
    pub fn from_polygons(polygons: impl IntoIterator<Item = Polygon>) -> Self {
        let sides = Self::group_by_sides(polygons)
            .filter_map(|s| s.join_polygons_on_side())
            .collect_vec();
    }
}
