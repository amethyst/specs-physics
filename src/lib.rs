#[macro_use]
extern crate log;
extern crate ncollide3d as ncollide;
extern crate nphysics3d as nphysics;

use std::collections::HashMap;

pub use nalgebra as math;
use nalgebra::RealField;
use nphysics::{
    object::{BodyHandle, ColliderHandle},
    world::World,
};
use specs::world::Index;

pub use self::{
    body::{PhysicsBody, PhysicsBodyBuilder},
    collider::{PhysicsCollider, PhysicsColliderBuilder, Shape},
};
use specs::{Component, DenseVecStorage, Dispatcher, DispatcherBuilder, Entity, FlaggedStorage};
use specs_hierarchy::Parent;

use self::{
    body::Position,
    math::Vector3,
    systems::{
        sync_bodies_to_physics::SyncBodiesToPhysicsSystem,
        sync_colliders_to_physics::SyncCollidersToPhysicsSystem,
    },
};

pub mod body;
pub mod collider;
pub mod systems;

/// The `Physics` `Resource` contains the nphysics `World` and a set of
/// `HashMap`s containing handles to objects of the `World`. These are necessary
/// so we can properly react to removed `Component`s and clean up our `World`
/// accordingly.
pub struct Physics<N: RealField> {
    world: World<N>,

    body_handles: HashMap<Index, BodyHandle>,
    collider_handles: HashMap<Index, ColliderHandle>,
}

impl<N: RealField> Default for Physics<N> {
    fn default() -> Self {
        Self {
            world: World::new(),
            body_handles: HashMap::new(),
            collider_handles: HashMap::new(),
        }
    }
}

/// `Gravity` is a type alias for `Vector3<f32>`. It represents a constant
/// acceleration affecting all physical objects in the scene.
pub type Gravity<N> = Vector3<N>;

/// The `PhysicsParent` `Component` is used to represent a parent/child
/// relationship between physics based `Entity`s.
#[derive(Debug, Clone, Eq, Ord, PartialEq, PartialOrd)]
pub struct PhysicsParent {
    pub entity: Entity,
}

impl Component for PhysicsParent {
    type Storage = FlaggedStorage<Self, DenseVecStorage<Self>>;
}

impl Parent for PhysicsParent {
    fn parent_entity(&self) -> Entity {
        self.entity
    }
}

/// Convenience function for configuring and building a `Dispatcher` with all
/// required physics related `System`s. This also serves as a blueprint on how
/// to properly set up the `System`s and have them depend on each other.
pub fn dispatcher<'a, 'b, N, P>() -> Dispatcher<'a, 'b>
where
    N: RealField,
    P: Component<Storage = FlaggedStorage<P, DenseVecStorage<P>>> + Position<N> + Send + Sync,
{
    let mut dispatcher_builder = DispatcherBuilder::new();

    // add SyncBodiesToPhysicsSystem first since we have to start with bodies;
    // colliders can exist without a body but in most cases have a body parent
    dispatcher_builder.add(
        SyncBodiesToPhysicsSystem::<N, P>::default(),
        "sync_bodies_to_physics_system",
        &[],
    );

    // add SyncCollidersToPhysicsSystem next with SyncBodiesToPhysicsSystem as its
    // dependency
    dispatcher_builder.add(
        SyncCollidersToPhysicsSystem::<N, P>::default(),
        "sync_colliders_to_physics_system",
        &["sync_bodies_to_physics_system"],
    );

    dispatcher_builder.build()
}
