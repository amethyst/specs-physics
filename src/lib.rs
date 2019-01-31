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

#[macro_use]
extern crate log;

pub mod bodies;
pub mod colliders;
pub mod systems;
pub mod time_step;

pub use self::bodies::*;
pub use self::colliders::*;
pub use self::systems::*;
pub use self::time_step::*;

/// The Physical World containing all physical objects.
pub type PhysicsWorld = self::nphysics::world::World<f32>;

/// Gravity is a type alias for a Vector of dimension 3.
/// It represents a constant acceleration affecting all physical objects in the scene.
pub type Gravity = self::nphysics::math::Vector<f32>;
