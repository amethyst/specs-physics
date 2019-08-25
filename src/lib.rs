//! ## Usage
//!
//! To use **specs-physics**, add the following dependency to your projects
//! *Cargo.toml*:
//!
//! ```toml
//! [dependencies]
//! specs-physics = "0.3.0"
//! ```
//!
//! **specs-physics** defines a set of [Specs] `System`s and `Component`s to
//! handle the creation, modification and removal of [nphysics] objects
//! ([RigidBody], [Collider]) and the synchronisation of object positions
//! and global gravity between both worlds.
//!
//! ### Generic types
//!
//! All `System`s and `Component`s provided by this crate require between one
//! and two type parameters to function properly. These were explicitly
//! introduced to keep this integration as generic as possible and allow
//! compatibility with as many external crates and game engines as possible.
//!
//! #### `N: RealField`
//!
//! [nphysics] is built upon [nalgebra] and uses various types and
//! structures from this crate. **specs-physics** builds up on this even further
//! and utilises the same structures, which all work with any type that
//! implements `nalgebra::RealField`. `nalgebra::RealField` is by default
//! implemented for various standard types, such as `f32` and`f64`. `nalgebra`
//! is re-exported under `specs_physics::nalgebra`.
//!
//! #### `P: Position<N>`
//!
//! a type parameter which implements the `specs_physics::bodies::Position`
//! *trait*, requiring also a `Component` implementation with a
//! `FlaggedStorage`. This `Position` `Component` is used to initially place a
//! [RigidBody] in the [nphysics] world and later used to synchronise the
//! updated translation and rotation of these bodies back into the [Specs]
//! world.
//!
//! Example for a `Position` `Component`, simply using the "Isometry" type (aka
//! combined translation and rotation structure) directly:
//!
//! ```rust,ignore
//! use specs::{Component, DenseVecStorage, FlaggedStorage};
//! use specs_physics::{bodies::Position, nalgebra::Isometry3};
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
//! }
//! ```
//!
//! If you're using [Amethyst], you can enable the "amethyst" feature for this
//! crate which provides a `Position<Float>` impl for `Transform`.
//!
//! ```toml
//! [dependencies]
//! specs-physics = { version = "0.3", features = ["amethyst"] }
//! ```
//!
//! ### Components
//!
//! ##### PhysicsBody
//!
//! The `specs_physics::PhysicsBody` `Component` is used to define [RigidBody]
//! from the comforts of your [Specs] world. Changes to the `PhysicsBody` will
//! automatically be synchronised with [nphysics].
//!
//! Example:
//!
//! ```rust,ignore
//! use specs_physics::{
//!     nalgebra::{Matrix3, Point3},
//!     nphysics::{algebra::Velocity3, object::BodyStatus},
//!     PhysicsBodyBuilder,
//! };
//!
//! let physics_body = PhysicsBodyBuilder::from(BodyStatus::Dynamic)
//!     .gravity_enabled(true)
//!     .velocity(Velocity3::linear(1.0, 1.0, 1.0))
//!     .angular_inertia(Matrix3::from_diagonal_element(3.0))
//!     .mass(1.3)
//!     .local_center_of_mass(Point3::new(0.0, 0.0, 0.0))
//!     .build();
//! ```
//!
//! ##### PhysicsCollider
//!
//! `specs_physics::PhysicsCollider`s are the counterpart to `PhysicsBody`s.
//! They can exist on their own or as a part of a `PhysicsBody`
//! `PhysicsCollider`s are used to define and create [Collider]'s in
//! [nphysics].
//!
//! Example:
//!
//! ```rust,ignore
//! use specs_physics::{
//!     colliders::Shape,
//!     nalgebra::{Isometry3, Vector3},
//!     ncollide::world::CollisionGroups,
//!     nphysics::material::{BasicMaterial, MaterialHandle},
//!     PhysicsColliderBuilder,
//! };
//!
//! let physics_collider = PhysicsColliderBuilder::from(
//!         Shape::Cuboid{ half_extents: Vector3::new(10.0, 10.0, 1.0) })
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
//! To assign multiple [Collider]'s the the same body, [Entity hierarchy]
//! can be used. This utilises [specs-hierarchy].
//!
//! ### Systems
//!
//! The following `System`s currently exist and should be added to your
//! `Dispatcher` in order:
//!
//! 1. `specs_physics::systems::SyncBodiesToPhysicsSystem` - handles the
//! creation, modification and removal of [RigidBody]'s based on the
//! `PhysicsBody` `Component` and an implementation of the `Position`
//! *trait*.
//!
//! 2. `specs_physics::systems::SyncCollidersToPhysicsSystem` - handles
//! the creation, modification and removal of [Collider]'s based on the
//! `PhysicsCollider` `Component`. This `System` depends on
//! `SyncBodiesToPhysicsSystem` as [Collider] can depend on [RigidBody].
//!
//! 3. `specs_physics::systems::SyncParametersToPhysicsSystem` - handles the
//! modification of the [nphysics] `World`s parameters.
//!
//! 4. `specs_physics::systems::PhysicsStepperSystem` - handles the progression
//! of the [nphysics] `World` and causes objects to actually move and
//! change their position. This `System` is the backbone for collision
//! detection.
//!
//! 5. `specs_physics::systems::SyncBodiesFromPhysicsSystem` -
//! handles the synchronisation of [RigidBody] positions and dynamics back
//! into the [Specs] `Component`s. This `System` also utilises the
//! `Position` *trait* implementation.
//!
//! An example `Dispatcher` with all required `System`s:
//!
//! ```rust,no_run
//! use specs::DispatcherBuilder;
//! use specs_physics::{
//!     systems::{
//!         PhysicsStepperSystem,
//!         SyncBodiesFromPhysicsSystem,
//!         SyncBodiesToPhysicsSystem,
//!         SyncCollidersToPhysicsSystem,
//!         SyncParametersToPhysicsSystem,
//!     },
//!     SimplePosition,
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
//!         SyncBodiesFromPhysicsSystem::<f32, SimplePosition<f32>>::default(),
//!         "sync_bodies_from_physics_system",
//!         &["physics_stepper_system"],
//!     )
//!     .build();
//! ```
//!
//! If you're using [Amethyst] Transforms directly, you'd pass the generic
//! arguments like so:
//!
//! ```rust,ignore
//! use amethyst::core::{Float, Transform};
//! use specs_physics::systems::SyncBodiesToPhysicsSystem;
//! SyncBodiesToPhysicsSystem::<Float, Transform>::default();
//! ```
//!
//! Alternatively to building your own `Dispatcher`, you can always fall back on
//! the convenience function `specs_physics::physics_dispatcher()`, which
//! returns a configured *default* `Dispatcher` for you or
//! `specs_physics::register_physics_systems()` which takes a
//! `DispatcherBuilder` as an argument and registers the required `System`s for
//! you.
//!
//! [Specs]: https://slide-rs.github.io/specs/
//! [nphysics]: https://www.nphysics.org/
//! [nalgebra]: https://nalgebra.org/
//! [RigidBody]: https://www.nphysics.org/rigid_body_simulations_with_contacts/#rigid-bodies
//! [Collider]: https://www.nphysics.org/rigid_body_simulations_with_contacts/#colliders
//! [Amethyst]: https://amethyst.rs/
//! [Entity hierarchy]: https://github.com/bamling/specs-physics/blob/master/examples/hierarchy.rs
//! [specs-hierarchy]: https://github.com/rustgd/specs-hierarchy

#[macro_use]
extern crate log;

pub use nalgebra;
#[cfg(feature = "physics3d")]
pub extern crate ncollide3d as ncollide;
#[cfg(feature = "physics3d")]
pub extern crate nphysics3d as nphysics;
#[cfg(feature = "physics2d")]
pub extern crate ncollide2d as ncollide;
#[cfg(feature = "physics2d")]
pub extern crate nphysics2d as nphysics;
pub use shrev;

use std::collections::HashMap;

use specs::{
    world::Index, Component, DenseVecStorage, Dispatcher, DispatcherBuilder, Entity, FlaggedStorage,
};
use specs_hierarchy::Parent;

pub use self::{
    bodies::{PhysicsBody, PhysicsBodyBuilder},
    positon::{Position, SimplePosition},
    colliders::{PhysicsCollider, PhysicsColliderBuilder},
};

use nphysics::{
    counters::Counters,
    material::MaterialsCoefficientsTable,
    object::{BodyHandle, ColliderHandle},
    solver::IntegrationParameters,
    world::World,
};

use self::{
    nalgebra::RealField,
    systems::{
        PhysicsStepperSystem, SyncBodiesFromPhysicsSystem, SyncBodiesToPhysicsSystem,
        SyncCollidersToPhysicsSystem, SyncParametersToPhysicsSystem,
    },
};

#[cfg(feature = "physics3d")]
use nalgebra::Vector3 as Vector;

#[cfg(feature = "physics2d")]
use nalgebra::Vector2 as Vector;

pub mod bodies;
pub mod colliders;
pub mod events;
pub mod parameters;
pub mod systems;
pub mod positon;

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
    pub fn gravity(&self) -> &Vector<N> {
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
/// ```rust
/// use specs_physics::SimplePosition;
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

    // add SyncBodiesFromPhysicsSystem last as it handles the
    // synchronisation between nphysics World bodies and the Position
    // components; this depends on the PhysicsStepperSystem
    dispatcher_builder.add(
        SyncBodiesFromPhysicsSystem::<N, P>::default(),
        "sync_bodies_from_physics_system",
        &["physics_stepper_system"],
    );
}
