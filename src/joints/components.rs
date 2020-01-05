use crate::{nalgebra::RealField, nphysics::joint::JointConstraint};
use specs::{Component, DenseVecStorage, Entity, FlaggedStorage};

/// The component type of all constraint joints.
pub struct JointComponent<N: RealField>(pub Box<dyn JointConstraint<N, Entity>>);

impl<N: RealField> Component for JointComponent<N> {
    type Storage = FlaggedStorage<Self, DenseVecStorage<Self>>;
}
