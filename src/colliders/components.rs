use crate::{nalgebra::RealField, nphysics::object::Collider};
use specs::{Component, DenseVecStorage, Entity, FlaggedStorage};

/// The component type of all physics colliders.
#[derive(Shrinkwrap)]
#[shrinkwrap(mutable)]
pub struct ColliderComponent<N: RealField>(pub Collider<N, Entity>);

impl<N: RealField> Component for ColliderComponent<N> {
    type Storage = FlaggedStorage<Self, DenseVecStorage<Self>>;
}
