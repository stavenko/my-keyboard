use super::index::GeoIndex;

pub trait GeoObject<'a> {
    type Ref: UnRef<'a>;
    type MutRef: UnRef<'a>;

    fn make_ref(&self, index: &'a GeoIndex) -> Self::Ref;
    fn make_mut_ref(&self, index: &'a mut GeoIndex) -> Self::MutRef;
}

pub trait UnRef<'a> {
    type Obj: GeoObject<'a>;

    fn un_ref(self) -> Self::Obj;
}
