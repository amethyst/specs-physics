# specs-physics

[![Build Status][bi]][bl] [![Crates.io][ci]][cl] ![MIT/Apache][li] [![Docs.rs][di]][dl]

[bi]: https://travis-ci.com/bamling/specs-physics.svg?branch=master
[bl]: https://travis-ci.com/bamling/specs-physics

[ci]: https://img.shields.io/crates/v/specs-physics.svg
[cl]: https://crates.io/crates/specs-physics/

[li]: https://img.shields.io/crates/l/specs-physics.svg

[di]: https://docs.rs/specs-physics/badge.svg
[dl]: https://docs.rs/specs-physics/

**specs-physics** aims to be an easily usable and extendable [nphysics](https://www.nphysics.org/) physics engine integration for applications and games that utilise the [Specs ECS](https://slide-rs.github.io/specs/).

The dream is to *simply* create `Entity`s with a set of configurable `Component`s and have most of your physics covered, be it collision/proximity detection, velocity and acceleration or gravity.

## Usage

To use **specs-physics**, add the following dependency to your projects *Cargo.toml*:

```toml
[dependencies]
specs-physics = "0.2.2"
```

**specs-physics** defines a set of [Specs](https://slide-rs.github.io/specs/) `System`s and `Component`s to handle the creation, modification and removal of [nphysics](https://www.nphysics.org/) objects ([RigidBody](https://www.nphysics.org/rigid_body_simulations_with_contacts/#rigid-bodies), [Collider](https://www.nphysics.org/rigid_body_simulations_with_contacts/#colliders)) and the synchronisation of object positions and global gravity between both worlds.

### Generic types

All `System`s and `Component`s provided by this crate require between one and two type parameters to function properly. These were explicitly introduced to keep this integration as generic as possible and allow compatibility with as many external crates and game engines as possible.

`N: RealField` - [nphysics](https://www.nphysics.org/) is built upon [nalgebra](https://nalgebra.org/) and uses various types and structures from this crate. **specs-physics** builds up on this even further and utilises the same structures, which all work with any type that implements `nalgebra::RealField`. `nalgebra::RealField` is by default implemented for various standard types, such as `f32` and`f64`. `nalgebra` is re-exported under `specs_physics::math`.

`P: Position<N>` - a type parameter which implements the `specs_physics::bodies::Position` *trait*, requiring also a `Component` implementation with a `FlaggedStorage`. This `Position` `Component` is used to initially place a [RigidBody](https://www.nphysics.org/rigid_body_simulations_with_contacts/#rigid-bodies) in the [nphysics](https://www.nphysics.org/) world and later used to synchronise the updated translation and rotation of these bodies back into the [Specs](https://slide-rs.github.io/specs/) world.

Example for a `Position` `Component`, simply using the "Isometry" type (aka combined translation and rotation structure) directly:
```rust
use specs::{Component, DenseVecStorage, FlaggedStorage};
use specs_physics::{
    bodies::Position,
    math::Isometry3,
};

struct Position(Isometry3<f32>);

impl Component for Position {
    type Storage = FlaggedStorage<Self, DenseVecStorage<Self>>;
}

impl Position<f32> for Position {
    fn as_isometry(&self) -> Isometry3<f32> {
        self.0
    }

    fn set_isometry(&mut self, isometry: &Isometry3<f32>) {
        self.0 = isometry;
    }
}
```

### Components

##### PhysicsBody

The `specs_physics::PhysicsBody` `Component` is used to define [RigidBody](https://www.nphysics.org/rigid_body_simulations_with_contacts/#rigid-bodies) from the comforts of your [Specs](https://slide-rs.github.io/specs/) world. Changes to the `PhysicsBody` will automatically be synchronised with [nphysics](https://www.nphysics.org/).

Example:

```rust
use specs_physics::{
    bodies::BodyStatus,
    math::{Matrix3, Point3, Vector3},
    PhysicsBodyBuilder,
};

let physics_body = PhysicsBodyBuilder::from(BodyStatus::Dynamic)
    .gravity_enabled(true)
    .velocity(Vector3::new(1.0, 1.0, 1.0))
    .angular_inertia(Matrix3::from_diagonal_element(3.0))
    .mass(1.3)
    .local_center_of_mass(Point3::new(0.0, 0.0, 0.0))
    .build();
```

##### PhysicsCollider

`specs_physics::PhysicsCollider`s are the counterpart to `PhysicsBody`s. They can exist on their own or as a part of a `PhysicsBody` `PhysicsCollider`s are used to define and create [Colliders](https://www.nphysics.org/rigid_body_simulations_with_contacts/#colliders) in [nphysics](https://www.nphysics.org/).

Example:

```rust
use specs_physics::{
    colliders::{
        material::{BasicMaterial, MaterialHandle},
        CollisionGroups,
    },
    math::Isometry3,
    PhysicsColliderBuilder,
    Shape,
};

let physics_collider = PhysicsColliderBuilder::from(Shape::Rectangle(10.0, 10.0, 1.0))
    .offset_from_parent(Isometry3::identity())
    .density(1.2)
    .material(MaterialHandle::new(BasicMaterial::default()))
    .margin(0.02)
    .collision_groups(CollisionGroups::default())
    .linear_prediction(0.001)
    .angular_prediction(0.0)
    .sensor(true)
    .build();
``` 

To assign multiple [Colliders](https://www.nphysics.org/rigid_body_simulations_with_contacts/#colliders) the the same body, [Entity hierarchy](https://github.com/bamling/specs-physics/blob/master/examples/hierarchy.rs) can be used. This utilises [specs-hierarchy](https://github.com/rustgd/specs-hierarchy).

### Systems

The following `System`s currently exist and should be added to your `Dispatcher` in order:

1. `specs_physics::systems::SyncBodiesToPhysicsSystem` - handles the creation, modification and removal of [RigidBodies](https://www.nphysics.org/rigid_body_simulations_with_contacts/#rigid-bodies) based on the `PhysicsBody` `Component` and an implementation of the `Position` *trait*.
2. `specs_physics::systems::SyncCollidersToPhysicsSystem` - handles the creation, modification and removal of [Colliders](https://www.nphysics.org/rigid_body_simulations_with_contacts/#colliders) based on the `PhysicsCollider` `Component`. This `System` depends on `SyncBodiesToPhysicsSystem` as [Colliders](https://www.nphysics.org/rigid_body_simulations_with_contacts/#colliders) can depend on [RigidBodies](https://www.nphysics.org/rigid_body_simulations_with_contacts/#rigid-bodies).
4. `specs_physics::systems::PhysicsStepperSystem` - handles the progression of the [nphysics](https://www.nphysics.org/) `World` and causes objects to actually move and change their position. This `System` is the backbone for collision detection.
5. `specs_physics::systems::SyncPositionsFromPhysicsSystem` - handles the synchronisation of [RigidBodies](https://www.nphysics.org/rigid_body_simulations_with_contacts/#rigid-bodies) positions back into the [Specs](https://slide-rs.github.io/specs/) `Component`s. This `System` also utilises the `Position` *trait* implementation.

An example `Dispatcher` with all required `System`s:

```rust
let dispatcher = DispatcherBuilder::new()
    .with(
        SyncBodiesToPhysicsSystem::<f32, MyPosition>::default(),
        "sync_bodies_to_physics_system",
        &[],
    )
    .with(
        SyncCollidersToPhysicsSystem::<f32, MyPosition>::default(),
        "sync_colliders_to_physics_system",
        &["sync_bodies_to_physics_system"],
    )
    .with(
        PhysicsStepperSystem::<f32>::default(),
        "physics_stepper_system",
        &[
            "sync_bodies_to_physics_system",
            "sync_colliders_to_physics_system",
        ],
    )
    .with(
        SyncPositionsFromPhysicsSystem::<f32, MyPosition>::default(),
        "sync_positions_from_physics_system",
        &["physics_stepper_system"],
    )
    .build();
```

Alternatively to building your own `Dispatcher`, you can always fall back on the convenience function `specs_physics::physics_dispatcher()`, which returns a configured *default* `Dispatcher` for you or `specs_physics::register_physics_systems()` which takes a `DispatcherBuilder` as an argument and registers the required `System`s for you .

### Examples

Full examples can be found under [src/examples](https://github.com/bamling/specs-physics/tree/master/examples). If anything is missing or unclear, feel free to open an issue or give me a poke!

## Contributing 

I'd appreciate any kind of contribution to this project, be it feature requests, bugs/issues, pull requests, documentation, tests or examples! 

Please just try to format any code changes according to the [rustfmt.toml](https://github.com/bamling/specs-physics/blob/master/rustfmt.toml) rules. They're not exactly set in stone and I'm open for suggestions, but let's try to keep things tidy!

## Current Roadmap

Full *TODO* sheet can be found in [this nphysics issue][todo]

- [x] RigidBody Components
- [x] Collider Components
- [x] Proximity and Contact EventChannels
- [ ] External force property
- [x] `log` based logging
- [ ] Handling Body Activation & Sleeping
- [ ] Multibody-based Component Joints
- [ ] Force generator inversion of control
- [ ] Time scale and simulation pausing

Investigating:

- [ ] Proximity & Curve-based external force utility
- [ ] Constraint-based Joints
- [ ] Kinematics

## License

Distributed under the MIT License. See [LICENSE](https://github.com/bamling/specs-physics/blob/master/LICENSE) for more information.

## Acknowledgments

This project is heavily inspired by [nphysics-ecs-dumb](https://github.com/distransient/nphysics-ecs-dumb); they did most of the heavy lifting, I'm just building up on what they have started!

**Special thanks to:**
- [distransient](https://github.com/distransient)
- [jojolepro](https://github.com/jojolepro)