#[macro_use]
extern crate log;
extern crate ncollide3d as ncollide;
extern crate nphysics3d as nphysics;

use std::collections::HashMap;

pub use nalgebra as math;
use nalgebra::{RealField, Scalar};
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
        PhysicsStepperSystem,
        SyncBodiesToPhysicsSystem,
        SyncCollidersToPhysicsSystem,
        SyncGravityToPhysicsSystem,
        SyncPositionsFromPhysicsSystem,
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

/// `Gravity` is a newtype for `Vector3`. It represents a constant
/// acceleration affecting all physical objects in the scene.
pub struct Gravity<N: RealField + Scalar>(Vector3<N>);

impl<N: RealField + Scalar> Default for Gravity<N> {
    fn default() -> Self {
        Self(Vector3::repeat(N::zero()))
    }
}

/// The `TimeStep` is used to set the timestep of the nphysics integration, see
/// `nphysics::world::World::set_timestep(..)`.
pub struct TimeStep<N: RealField>(N);

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
/// required physics related `System`s.
pub fn physics_dispatcher<'a, 'b, N, P>() -> Dispatcher<'a, 'b>
where
    N: RealField,
    P: Component<Storage = FlaggedStorage<P, DenseVecStorage<P>>> + Position<N> + Send + Sync,
{
    let mut dispatcher_builder = DispatcherBuilder::new();
    register_physics_systems::<N, P>(&mut dispatcher_builder);

    dispatcher_builder.build()
}

/// Convenience function for registering all required physics related `System`s
/// to the given `DispatcherBuilder`. This also serves as a blueprint on how
///// to properly set up the `System`s and have them depend on each other.
pub fn register_physics_systems<N, P>(dispatcher_builder: &mut DispatcherBuilder)
where
    N: RealField,
    P: Component<Storage = FlaggedStorage<P, DenseVecStorage<P>>> + Position<N> + Send + Sync,
{
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

    // add SyncGravityToPhysicsSystem; this System can be added at any point in time
    // as it merely handles the gravity value of the nphysics World, thus it has no
    // other dependencies
    dispatcher_builder.add(
        SyncGravityToPhysicsSystem::<N>::default(),
        "sync_gravity_to_physics_system",
        &[],
    );

    // add PhysicsStepperSystem after all other Systems that write data to the
    // nphysics World and has to depend on them; this System is used to progress the
    // nphysics World for all existing objects
    dispatcher_builder.add(
        PhysicsStepperSystem::<N>::default(),
        "physics_stepper_system",
        &[
            "sync_bodies_to_physics_system",
            "sync_colliders_to_physics_system",
            "sync_gravity_to_physics_system",
        ],
    );

    // add SyncPositionsFromPhysicsSystem last as it handles the
    // synchronisation between nphysics World bodies and the Position
    // components; this depends on the PhysicsStepperSystem
    dispatcher_builder.add(
        SyncPositionsFromPhysicsSystem::<N, P>::default(),
        "sync_positions_from_physics_system",
        &["physics_stepper_system"],
    );
}
