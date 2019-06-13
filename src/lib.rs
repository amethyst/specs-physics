#[macro_use]
extern crate log;
extern crate ncollide3d as ncollide;
extern crate nphysics3d as nphysics;

use std::collections::HashMap;

pub use nalgebra as math;
use nphysics::{
    counters::Counters,
    material::MaterialsCoefficientsTable,
    object::{BodyHandle, ColliderHandle},
    solver::IntegrationParameters,
    world::World,
};
pub use shrev;
use specs::world::Index;

pub use self::{
    bodies::{PhysicsBody, PhysicsBodyBuilder},
    colliders::{PhysicsCollider, PhysicsColliderBuilder},
};
use specs::{Component, DenseVecStorage, Dispatcher, DispatcherBuilder, Entity, FlaggedStorage};
use specs_hierarchy::Parent;

use self::{
    bodies::Position,
    math::{RealField, Vector3},
    systems::{
        PhysicsStepperSystem,
        SyncBodiesToPhysicsSystem,
        SyncCollidersToPhysicsSystem,
        SyncParametersToPhysicsSystem,
        SyncPositionsFromPhysicsSystem,
    },
};

pub mod bodies;
pub mod colliders;
pub mod events;
pub mod parameters;
pub mod systems;

/// Resource holding the internal fields where physics computation occurs.
/// Some inspection methods are exposed to allow debugging.
pub struct Physics<N: RealField> {
    /// Core structure where physics computation and synchronization occurs.
    /// Also contains ColliderWorld.
    pub(crate) world: World<N>,

    /// Hashmap of Entities to internal Physics bodies.
    /// Necessary for reacting to removed Components.
    pub(crate) body_handles: HashMap<Index, BodyHandle>,
    /// Hashmap of Entities to internal Collider handles.
    /// Necessary for reacting to removed Components.
    pub(crate) collider_handles: HashMap<Index, ColliderHandle>,
}

// Some non-mutating methods for diagnostics and testing
impl<N: RealField> Physics<N> {
    /// Creates a new instance of the physics structure.
    pub fn new() -> Self {
        Self::default()
    }

    /// Reports the internal value for the timestep.
    /// See also `TimeStep` for setting this value.
    pub fn timestep(&self) -> N {
        self.world.timestep()
    }

    /// Reports the internal value for the gravity.
    /// See also `Gravity` for setting this value.
    pub fn gravity(&self) -> &Vector3<N> {
        self.world.gravity()
    }

    /// Reports the internal value for prediction distance in collision
    /// detection. This cannot change and will normally be `0.002m`
    pub fn prediction(&self) -> N {
        self.world.prediction()
    }

    /// Retrieves the performance statistics for the last simulated timestep.
    /// Profiling is disabled by default.
    /// See also `PhysicsProfilingEnabled` for enabling performance counters.
    pub fn performance_counters(&self) -> &Counters {
        self.world.performance_counters()
    }

    /// Retrieves the internal parameters for integration.
    /// See also `PhysicsIntegrationParameters` for setting these parameters.
    pub fn integration_parameters(&self) -> &IntegrationParameters<N> {
        self.world.integration_parameters()
    }

    /// Retrieves the internal lookup table for friction and restitution
    /// constants. Exposing this for modification is TODO.
    pub fn materials_coefficients_table(&self) -> &MaterialsCoefficientsTable<N> {
        self.world.materials_coefficients_table()
    }
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
    P: Position<N>,
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
    P: Position<N>,
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

    // add SyncParametersToPhysicsSystem; this System can be added at any point in
    // time as it merely synchronizes the simulation parameters of the world,
    // thus it has no other dependencies.
    dispatcher_builder.add(
        SyncParametersToPhysicsSystem::<N>::default(),
        "sync_parameters_to_physics_system",
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
