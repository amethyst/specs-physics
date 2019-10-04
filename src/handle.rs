use crate::{
    nalgebra::RealField,
    nphysics::object::{Body, BodyPart},
    BodyHandleType,
    BodySetType,
};

use specs::{
    storage::UnprotectedStorage,
    world::Index,
    BitSet,
    Component,
    DenseVecStorage,
    Join,
    ReadStorage,
};

use std::convert::identity;

/// Handle attached to entities representing a single Body
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd, Shrinkwrap)]
#[shrinkwrap(mutable)]
pub struct BodyHandle(pub BodyHandleType);

impl Component for BodyHandle {
    type Storage = DenseVecStorage<Self>;
}

/// Handle attached to entities representing a single BodyPart.
#[derive(Clone, Copy, Debug, Eq, Hash, Ord, PartialEq, PartialOrd)]
pub struct BodyPartHandle {
    pub part_index: usize,
    pub set_handle: BodyHandleType,
}

impl Component for BodyPartHandle {
    type Storage = DenseVecStorage<Self>;
}

/// Extension trait used to join over BodySet's
pub trait EntityHandleExt<'a, N: RealField> {
    fn join(&self, handles: &'a ReadStorage<'a, BodyHandle>) -> BodyJoin<'a, N>;

    fn join_part(&self, handles: &'a ReadStorage<'a, BodyPartHandle>) -> BodyPartJoin<'a, N>;
}

impl<'a, N: RealField> EntityHandleExt<'a, N> for &'a BodySetType<N> {
    fn join(&self, handles: &'a ReadStorage<'a, BodyHandle>) -> BodyJoin<'a, N> {
        BodyJoin(self, handles)
    }

    fn join_part(&self, handles: &'a ReadStorage<'a, BodyPartHandle>) -> BodyPartJoin<'a, N> {
        BodyPartJoin(self, handles)
    }
}

pub trait EntityHandleMutExt<'a, N: RealField> {
    fn join_mut(&mut self, handles: &'a ReadStorage<'a, BodyHandle>) -> MutBodyJoin<'a, N>;
}

impl<'a, N: RealField> EntityHandleMutExt<'a, N> for &'a mut BodySetType<N> {
    fn join_mut(&mut self, handles: &'a ReadStorage<'a, BodyHandle>) -> MutBodyJoin<'a, N> {
        unsafe { MutBodyJoin(&mut *(*self as *mut BodySetType<N>), handles) }
    }
}

pub struct BodyJoin<'a, N: RealField>(&'a BodySetType<N>, &'a ReadStorage<'a, BodyHandle>);

impl<'a, 'e, N: RealField> Join for &'a BodyJoin<'e, N> {
    type Mask = &'a BitSet;
    type Type = Option<&'a dyn Body<N>>;
    type Value = (&'a BodySetType<N>, &'a <BodyHandle as Component>::Storage);

    unsafe fn open(self) -> (Self::Mask, Self::Value) {
        let inner = self.1.open();
        (inner.0, (&self.0, inner.1))
    }

    unsafe fn get(v: &mut Self::Value, i: Index) -> Self::Type {
        let handle = v.1.get(i);
        v.0.get(handle.0)
    }
}

pub struct MutBodyJoin<'a, N: RealField>(&'a mut BodySetType<N>, &'a ReadStorage<'a, BodyHandle>);

impl<'a, 'e, N: RealField> Join for &'a mut MutBodyJoin<'e, N> {
    type Mask = &'a BitSet;
    type Type = Option<&'a mut dyn Body<N>>;
    type Value = (
        &'a mut BodySetType<N>,
        &'a <BodyHandle as Component>::Storage,
    );

    unsafe fn open(self) -> (Self::Mask, Self::Value) {
        let inner = self.1.open();
        (inner.0, (&mut *(self.0 as *mut BodySetType<N>), inner.1))
    }

    unsafe fn get(v: &mut Self::Value, i: Index) -> Self::Type {
        let handle = v.1.get(i);
        (*(v.0 as *mut BodySetType<N>)).get_mut(handle.0)
    }
}

pub struct BodyPartJoin<'a, N: RealField>(&'a BodySetType<N>, &'a ReadStorage<'a, BodyPartHandle>);

impl<'a, 'e, N: RealField> Join for &'a BodyPartJoin<'e, N> {
    type Mask = &'a BitSet;
    type Type = Option<&'a dyn BodyPart<N>>;
    type Value = (
        &'a BodySetType<N>,
        &'a <BodyPartHandle as Component>::Storage,
    );

    unsafe fn open(self) -> (Self::Mask, Self::Value) {
        let inner = self.1.open();
        (inner.0, (&self.0, inner.1))
    }

    unsafe fn get(v: &mut Self::Value, i: Index) -> Self::Type {
        let handle = v.1.get(i);
        v.0.get(handle.set_handle)
            .map(|body| body.part(handle.part_index))
            .and_then(identity)
    }
}
