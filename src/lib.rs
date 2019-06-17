//! ## Usage
//!
//! To use **specs-physics**, add the following dependency to your projects
//! *Cargo.toml*:
//!
//! ```toml
//! [dependencies]
//! specs-physics = "0.2.2"
//! ```
//!
//! **specs-physics** defines a set of [Specs](https://slide-rs.github.io/specs/) `System`s and `Component`s to handle the creation, modification and removal of [nphysics](https://www.nphysics.org/) objects ([RigidBody](https://www.nphysics.org/rigid_body_simulations_with_contacts/#rigid-bodies), [Collider](https://www.nphysics.org/rigid_body_simulations_with_contacts/#colliders)) and the synchronisation of object positions and global gravity between both worlds.
//!
//! ### Generic types
//!
//! All `System`s and `Component`s provided by this crate require between one
//! and two type parameters to function properly. These were explicitly
//! introduced to keep this integration as generic as possible and allow
//! compatibility with as many external crates and game engines as possible.
//!
//! `N: RealField` - [nphysics](https://www.nphysics.org/) is built upon [nalgebra](https://nalgebra.org/) and uses various types and structures from this crate. **specs-physics** builds up on this even further and utilises the same structures, which all work with any type that implements `nalgebra::RealField`. `nalgebra::RealField` is by default implemented for various standard types, such as `f32` and`f64`. `nalgebra` is re-exported under `specs_physics::math`.
//!
//! `P: Position<N>` - a type parameter which implements the `specs_physics::bodies::Position` *trait*, requiring also a `Component` implementation with a `FlaggedStorage`. This `Position` `Component` is used to initially place a [RigidBody](https://www.nphysics.org/rigid_body_simulations_with_contacts/#rigid-bodies) in the [nphysics](https://www.nphysics.org/) world and later used to synchronise the updated translation and rotation of these bodies back into the [Specs](https://slide-rs.github.io/specs/) world.
//!
//! Example for a `Position` `Component`, simply using the "Isometry" type (aka
//! combined translation and rotation structure) directly:
//! ```rust
//! use specs::{Component, DenseVecStorage, FlaggedStorage};
//! use specs_physics::{bodies::Position, math::Isometry3};
//!
//! struct Pos(pub Isometry3<f32>);
//!
//! impl Component for Pos {
//!     type Storage = FlaggedStorage<Self, DenseVecStorage<Self>>;
//! }
//!
//! impl Position<f32> for Pos {
//!     fn isometry(&self) -> &Isometry3<f32> {
//!         &self.0
//!     }
//!
//!     fn isometry_mut(&mut self) -> &mut Isometry3<f32> {
//!         &mut self.0
//!     }
//!
//!     fn set_isometry(&mut self, isometry: &Isometry3<f32>) -> &mut Pos {
//!         self.0 = *isometry;
//!         self
//!     }
//! }
//! ```
//!
//! ### Components
//!
//! ##### PhysicsBody
//!
//! The `specs_physics::PhysicsBody` `Component` is used to define [RigidBody](https://www.nphysics.org/rigid_body_simulations_with_contacts/#rigid-bodies) from the comforts of your [Specs](https://slide-rs.github.io/specs/) world. Changes to the `PhysicsBody` will automatically be synchronised with [nphysics](https://www.nphysics.org/).
//!
//! Example:
//!
//! ```rust
//! use specs_physics::{
//!     bodies::BodyStatus,
//!     math::{Matrix3, Point3, Vector3},
//!     PhysicsBodyBuilder,
//! };
//!
//! let physics_body = PhysicsBodyBuilder::from(BodyStatus::Dynamic)
//!     .gravity_enabled(true)
//!     .velocity(Vector3::new(1.0, 1.0, 1.0))
//!     .angular_inertia(Matrix3::from_diagonal_element(3.0))
//!     .mass(1.3)
//!     .local_center_of_mass(Point3::new(0.0, 0.0, 0.0))
//!     .build();
//! ```
//!
//! ##### PhysicsCollider
//!
//! `specs_physics::PhysicsCollider`s are the counterpart to `PhysicsBody`s. They can exist on their own or as a part of a `PhysicsBody` `PhysicsCollider`s are used to define and create [Colliders](https://www.nphysics.org/rigid_body_simulations_with_contacts/#colliders) in [nphysics](https://www.nphysics.org/).
//!
//! Example:
//!
//! ```rust
//! use specs_physics::{
//!     colliders::{
//!         material::{BasicMaterial, MaterialHandle},
//!         CollisionGroups,
//!         Shape,
//!     },
//!     math::Isometry3,
//!     PhysicsColliderBuilder,
//! };
//!
//! let physics_collider = PhysicsColliderBuilder::from(Shape::Rectangle(10.0, 10.0, 1.0))
//!     .offset_from_parent(Isometry3::identity())
//!     .density(1.2)
//!     .material(MaterialHandle::new(BasicMaterial::default()))
//!     .margin(0.02)
//!     .collision_groups(CollisionGroups::default())
//!     .linear_prediction(0.001)
//!     .angular_prediction(0.0)
//!     .sensor(true)
//!     .build();
//! ```
//!
//! To assign multiple [Colliders](https://www.nphysics.org/rigid_body_simulations_with_contacts/#colliders) the the same body, [Entity hierarchy](https://github.com/bamling/specs-physics/blob/master/examples/hierarchy.rs) can be used. This utilises [specs-hierarchy](https://github.com/rustgd/specs-hierarchy).
//!
//! ### Systems
//!
//! The following `System`s currently exist and should be added to your
//! `Dispatcher` in order:
//!
//! 1. `specs_physics::systems::SyncBodiesToPhysicsSystem` - handles the creation, modification and removal of [RigidBodies](https://www.nphysics.org/rigid_body_simulations_with_contacts/#rigid-bodies) based on the `PhysicsBody` `Component` and an implementation of the `Position` *trait*.
//! 2. `specs_physics::systems::SyncCollidersToPhysicsSystem` - handles the creation, modification and removal of [Colliders](https://www.nphysics.org/rigid_body_simulations_with_contacts/#colliders) based on the `PhysicsCollider` `Component`. This `System` depends on `SyncBodiesToPhysicsSystem` as [Colliders](https://www.nphysics.org/rigid_body_simulations_with_contacts/#colliders) can depend on [RigidBodies](https://www.nphysics.org/rigid_body_simulations_with_contacts/#rigid-bodies).
//! 3. `specs_physics::systems::SyncParametersToPhysicsSystem` - handles the modification of the [nphysics](https://www.nphysics.org/) `World`s parameters.
//! 4. `specs_physics::systems::PhysicsStepperSystem` - handles the progression of the [nphysics](https://www.nphysics.org/) `World` and causes objects to actually move and change their position. This `System` is the backbone for collision detection.
//! 5. `specs_physics::systems::SyncPositionsFromPhysicsSystem` - handles the synchronisation of [RigidBodies](https://www.nphysics.org/rigid_body_simulations_with_contacts/#rigid-bodies) positions back into the [Specs](https://slide-rs.github.io/specs/) `Component`s. This `System` also utilises the `Position` *trait* implementation.
//!
//! An example `Dispatcher` with all required `System`s:
//!
//! ```rust
//! use specs::DispatcherBuilder;
//! use specs_physics::{
//!     bodies::util::SimplePosition,
//!     systems::{
//!         PhysicsStepperSystem,
//!         SyncBodiesToPhysicsSystem,
//!         SyncCollidersToPhysicsSystem,
//!         SyncParametersToPhysicsSystem,
//!         SyncPositionsFromPhysicsSystem,
//!     },
//! };
//!
//! let dispatcher = DispatcherBuilder::new()
//!     .with(
//!         SyncBodiesToPhysicsSystem::<f32, SimplePosition<f32>>::default(),
//!         "sync_bodies_to_physics_system",
//!         &[],
//!     )
//!     .with(
//!         SyncCollidersToPhysicsSystem::<f32, SimplePosition<f32>>::default(),
//!         "sync_colliders_to_physics_system",
//!         &["sync_bodies_to_physics_system"],
//!     )
//!     .with(
//!         SyncParametersToPhysicsSystem::<f32>::default(),
//!         "sync_gravity_to_physics_system",
//!         &[],
//!     )
//!     .with(
//!         PhysicsStepperSystem::<f32>::default(),
//!         "physics_stepper_system",
//!         &[
//!             "sync_bodies_to_physics_system",
//!             "sync_colliders_to_physics_system",
//!             "sync_gravity_to_physics_system",
//!         ],
//!     )
//!     .with(
//!         SyncPositionsFromPhysicsSystem::<f32, SimplePosition<f32>>::default(),
//!         "sync_positions_from_physics_system",
//!         &["physics_stepper_system"],
//!     )
//!     .build();
//! ```
//!
//! Alternatively to building your own `Dispatcher`, you can always fall back on
//! the convenience function `specs_physics::physics_dispatcher()`, which
//! returns a configured *default* `Dispatcher` for you or
//! `specs_physics::register_physics_systems()` which takes a
//! `DispatcherBuilder` as an argument and registers the required `System`s for
//! you .

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
///
/// # Examples
/// ```
/// use specs_physics::bodies::util::SimplePosition;
/// let dispatcher = specs_physics::physics_dispatcher::<f32, SimplePosition<f32>>();
/// ```
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
            "sync_parameters_to_physics_system",
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
