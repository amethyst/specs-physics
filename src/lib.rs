#[macro_use]
extern crate log;
extern crate ncollide3d as ncollide;
extern crate nphysics3d as nphysics;

pub use nalgebra as math;
use nphysics::world::World;

use self::math::Vector3;
pub use self::{
    body::{PhysicsBody, PhysicsBodyBuilder},
    collider::{PhysicsCollider, PhysicsColliderBuilder, Shape},
};

pub mod body;
pub mod collider;
mod systems;

///// The `PhysicsWorld` containing all physical objects.
//pub type PhysicsWorld<N> = World<N>;
//
///// `Gravity` is a type alias for `Vector3<f32>`. It represents a constant
///// acceleration affecting all physical objects in the scene.
//pub type Gravity<N> = Vector3<N>;
