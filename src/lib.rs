//! nphysics-ecs-dumb
//! Straight forward wrapper around nphysics to allow its usage inside of Amethyst's ECS: Specs

pub extern crate ncollide3d as ncollide;
pub extern crate nphysics3d as nphysics;
#[macro_use]
extern crate derive_builder;
#[macro_use]
extern crate derive_new;
#[macro_use]
extern crate serde;

pub mod bodies;
pub mod systems;
pub mod colliders;

pub use self::bodies::*;
pub use self::forces::*;
pub use self::systems::*;
pub use self::colliders::*;

/// The Physical World containing all physical objects.
pub type PhysicsWorld = self::nphysics::world::World<f32>;
/// Gravity is a type alias for a Vector of dimension 3.
/// It represent a constant acceleration affecting all physical objects in the scene.
pub type Gravity = self::nphysics::math::Vector<f32>;
