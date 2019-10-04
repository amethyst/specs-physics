use crate::nphysics::{
    force_generator::DefaultForceGeneratorSet,
    joint::DefaultJointConstraintSet,
    world::{GeometricalWorld, MechanicalWorld},
};

pub(crate) mod body_set;
pub(crate) mod collider_set;

pub use body_set::{BodyComponent, BodyHandleType, BodySet};
pub use collider_set::{ColliderComponent, ColliderHandleType, ColliderSet};

/// This is an alias for the nphysics mechanical world type stored in the specs
/// world. You can fetch this type from the world with
/// `ReadExpect<'_, MechanicalWorldRes<f32>>` or
/// `WriteExpect<'_, MechanicalWorldRes<f32>>`.
pub type MechanicalWorldRes<N> = MechanicalWorld<N, BodyHandleType, ColliderHandleType>;

/// This is an alias for the nphysics geometrical world type stored in the specs
/// world. You can fetch this type from the world with
/// `ReadExpect<'_, MechanicalWorldRes<f32>>` or
/// `WriteExpect<'_, MechanicalWorldRes<f32>>`.
pub type GeometricalWorldRes<N> = GeometricalWorld<N, BodyHandleType, ColliderHandleType>;

// TODO: Can probably make the JointConstraintSet a Storage.
pub type JointConstraintSetRes<N> = DefaultJointConstraintSet<N, BodyHandleType>;

// TODO: Can probably make ForceGeneratorSet a Storage
// Although the usefulness may be somewhat limited?
// Investigating batch dispatch in relation to modifications in nphysics for
// execution of force generators seems a possible path forward.
// Do note, force generators may be executed in *substeps*
pub type ForceGeneratorSetRes<N> = DefaultForceGeneratorSet<N, BodyHandleType>;
