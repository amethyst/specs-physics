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
//! SyncBodiesToPhysicsSystem::<f32, Transform>::default();
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

#[macro_use]
extern crate shrinkwraprs;

pub use nalgebra;

#[cfg(feature = "dim3")]
pub use ncollide3d as ncollide;
#[cfg(feature = "dim3")]
pub use nphysics3d as nphysics;

#[cfg(feature = "dim2")]
pub use ncollide2d as ncollide;
#[cfg(feature = "dim2")]
pub use nphysics2d as nphysics;

pub mod builder;
pub mod events;
pub mod handle;
pub mod position;
pub mod systems;

pub use self::position::{Position, SimplePosition};

use crate::nphysics::{
    force_generator::DefaultForceGeneratorSet,
    joint::DefaultJointConstraintSet,
    object::{DefaultBodyHandle, DefaultBodySet, DefaultColliderHandle, DefaultColliderSet},
    world::{DefaultGeometricalWorld, DefaultMechanicalWorld},
};

pub type BodyHandleType = DefaultBodyHandle;
pub type ColliderHandleType = DefaultColliderHandle;

pub type MechanicalWorldRes<N> = DefaultMechanicalWorld<N>;
pub type GeometricalWorldRes<N> = DefaultGeometricalWorld<N>;
pub type BodySetRes<N> = DefaultBodySet<N>;
pub type ColliderSetRes<N> = DefaultColliderSet<N>;
pub type JointConstraintSetRes<N> = DefaultJointConstraintSet<N>;
pub type ForceGeneratorSetRes<N> = DefaultForceGeneratorSet<N>;
