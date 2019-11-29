/*!
**For when you want some [nphysics] in your [Specs]!** *Somewhat better than sliced bread!*

Remember all those "Default" types in the [nphysics tutorial], like `DefaultBodySet` and
`DefaultColliderSet`? **specs-physics** provides [ECS Component]-based implementations for
*nphysics* data sets, as well as faculties for synchronizing pose data to your position type of
choice and stepping functionality for stepping your simulations to a real good beat.

[Specs]: https://github.com/amethyst/specs
[ECS Component]: https://amethyst.github.io/specs/01_intro.html#whats-an-ecs
[nphysics]: https://www.nphysics.org/
[nphysics tutorial]: https://www.nphysics.org/rigid_body_simulations_with_contacts/#basic-setup

# Usage

To use **specs-physics** with [nphysics3d], add the following dependency to your project's
*[Cargo.toml]*:

```toml
[dependencies]
specs-physics = { version = "0.4.0", features = ["dim3"] }
```

For [nphysics2d], replace `dim3` with `dim2`. You **must** enable one of these two features, and
you can *only* enable one of them! Note that these docs are built by default with the 3D feature,
but content is the same with the exception of the `nphysics3d` and `ncollide3d` re-exports below.
Also available is an `amethyst` feature, which adds synchronization support for [Amethyst] through
`amethyst_core`'s [`Transform`] type as well as a [`SystemBundle`] trait impl for [`PhysicsBundle`].

And then for a basic working example, the following code goes in your `main.rs`, aspects of which
are explained in the sections below.

```
use specs::{DispatcherBuilder, Join, System, World, WorldExt};
use specs_physics::{
    bodies::WriteRigidBodies,
    nphysics::{
        math::{Force, ForceType, Vector},
        object::Body
    },
    PhysicsBundle,
    SimplePosition
};

fn main() {
    // Initialize world and dispatcher
    let mut world = World::new();
    let mut dispatcher_builder = DispatcherBuilder::new()
        .with(MyPhysicsSystem, "my_physics_system", &[]);

    // Create a new physics bundle
    PhysicsBundle::<f32, SimplePosition<f32>>::new(Vector::y() * -9.81, &["my_physics_system"])
        .register(&mut world, &mut dispatcher_builder);

    let mut dispatcher = dispatcher_builder.build();
    dispatcher.setup(&mut world);

    // In your application loop
    dispatcher.dispatch(&world);
}

struct MyPhysicsSystem;

impl<'s> System<'s> for MyPhysicsSystem {
    type SystemData = WriteRigidBodies<'s, f32>;

    fn run(&mut self, mut rigid_bodies: Self::SystemData) {
        let force = Force::<f32>::linear(Vector::x());
        for (body,) in (&mut rigid_bodies,).join() {
            body.apply_force(0, &force, ForceType::AccelerationChange, true);
        }
    }
}
```

[Cargo.toml]: https://doc.rust-lang.org/cargo/reference/specifying-dependencies.html
[nphysics2d]: https://www.nphysics.org/rustdoc/nphysics2d/index.html
[nphysics3d]: https://www.nphysics.org/rustdoc/nphysics3d/index.html
[Amethyst]: https://amethyst.rs/
[`Transform`]: https://docs.amethyst.rs/stable/amethyst_core/transform/components/struct.Transform.html
[`SystemBundle`]: https://docs.amethyst.rs/stable/amethyst_core/bundle/trait.SystemBundle.html

# Universal Generic Parameters

**specs-physics** relies on the following generic parameters for configuration, each of which should
only be given a single value throughout your codebase. Failing to use the same values for these
parameters will cause you to reference incorrect instances of your desired types!

## `N: RealField`

The `N`: [`RealField`] parameter determines the precision of the numeric type used for simulation.
Valid values for `N` are [`f32`] and [`f64`], although if you're using Amethyst only `f32` is
*currently* implemented for interoperability with [`Transform`]. Be careful about type inference in
the Rust language itself, because `f64` may be inferred for floating point numbers where a type
isn't stated and `f32` can't be inferred, so if you're using `f32` play it safe and always be
explicit with your `N` parameters.

## `P: Pose`

The second common parameter is the `P`: [`Pose`] parameter which determines the **Specs** Component
that physics positions are synchronized to. The mechanism for synchronization is explained in
further detail on the documentation for the [`Pose`] trait. Your `Pose` type should implement
`Pose<N>` for the same `N` that you've chosen. [`SimplePosition`] is provided as an easy utility
transform type which implements `Pose`, although for Amethyst, you may just pass [`Transform`] to
this parameter.

[`RealField`]: https://nalgebra.org/rustdoc/nalgebra/trait.RealField.html

# PhysicsBundle and its Systems

Besides providing nice storage types for *nphysics*, **specs-physics** takes on two major
responsibilities.

First is the stepping of the simulation, which is done simply once per run in
[`PhysicsStepperSystem`] and an advanced fixed stepper is provided in [`PhysicsBatchSystem`].
Second is the synchronization to the [`Pose`] type mentioned above, which is performed in the
[`PhysicsPoseSystem`].

The single step and pose synchronization systems are set up along with the Resources they depend on
by [`PhysicsBundle`], which is the best way to initiate your **specs-physics** simulation. You can
check out the documentation on that type's page for more details, and you can find examples for
different cases in the [examples directory].

[`PhysicsStepperSystem`]: systems/struct.PhysicsStepperSystem.html
[`PhysicsBatchSystem`]: systems/struct.PhysicsBatchSystem.html
[`PhysicsPoseSystem`]: systems/struct.PhysicsPoseSystem.html
[examples directory]: https://github.com/amethyst/specs-physics/tree/master/examples

# Interacting with the storages

So while in vanilla *nphysics* you may access your bodies, colliders, or constraint joints via
`DefaultBodySet`, `DefaultColliderSet`, or `DefaultJointConstraintSet`, this data is instead held in
[`BodyComponent`], [`ColliderComponent`], and [`JointComponent`] storages indexed with [`Entity`]
when using **specs-physics**. Sugar for inserting a new entity with a Body or Collider is provided
in [`EntityBuilderExt`] which also adds a marker component for some Body types to that body's
Entity, so that you may join over only the bodies of a specific type with automatic downcasting via
[`ReadRigidBodies`], [`WriteRigidBodies`], and [`ReadGroundBodies`], etc.

[`BodyComponent`]: bodies/struct.BodyComponent.html
[`ColliderComponent`]: colliders/struct.ColliderComponent.html
[`JointComponent`]: joints/struct.JointComponent.html
[`Entity`]: https://docs.rs/specs/latest/specs/struct.Entity.html
[`ReadRigidBodies`]: bodies/type.ReadRigidBodies.html
[`WriteRigidBodies`]: bodies/type.WriteRigidBodies.html
[`ReadGroundBodies`]: bodies/type.ReadGroundBodies.html

# Getting help

If you find any bugs or would like to contribut a pull request, visit our [Github repository].

If you'd like to ask for help, feel free to visit the `#physics` channel on our [Discord server]!

[Github repository]: https://github.com/amethyst/specs-physics
[Discord server]: https://discord.gg/amethyst
*/

#[cfg(any(
    not(any(feature = "dim2", feature = "dim3")),
    all(feature = "dim2", feature = "dim3")
))]
compile_error!(
    r#"Either the feature "dim3" or the feature "dim2" must be enabled.
You cannot enable both, and you cannot enable neither."#
);

#[macro_use]
extern crate log;

#[macro_use]
extern crate shrinkwraprs;

pub mod bodies;
pub mod colliders;
pub mod joints;
pub mod stepper;
pub mod systems;

mod builder;
mod bundle;
mod pose;
mod world;

pub use self::{
    builder::EntityBuilderExt,
    bundle::PhysicsBundle,
    pose::{Pose, SimplePosition},
    world::{ForceGeneratorSetRes, GeometricalWorldRes, MechanicalWorldRes},
};

pub use nalgebra;

#[cfg(feature = "dim3")]
pub use ncollide3d as ncollide;
#[cfg(feature = "dim3")]
pub use nphysics3d as nphysics;

#[cfg(feature = "dim2")]
pub use ncollide2d as ncollide;
#[cfg(feature = "dim2")]
pub use nphysics2d as nphysics;

// These are separated so they appear in a relevant order.
#[doc(no_inline)]
pub use bodies::ReadRigidBodies;

#[doc(no_inline)]
pub use bodies::WriteRigidBodies;

#[doc(no_inline)]
pub use bodies::BodyComponent;

#[doc(no_inline)]
pub use colliders::ColliderComponent;
