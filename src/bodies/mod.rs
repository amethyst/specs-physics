/*!
Storage, set, and marker types for Bodies, storing the bulk of the state for your simulation.
*/

mod components;
mod marker;
mod set;

pub use components::{BodyComponent, BodyPartHandle};
pub use marker::{
    BodyMarkerStorage, GroundMarker, MultibodyMarker, ReadGroundBodies, ReadMultiBodies,
    ReadRigidBodies, RigidBodyMarker, WriteGroundBodies, WriteMultiBodies, WriteRigidBodies,
};
pub use set::BodySet;
