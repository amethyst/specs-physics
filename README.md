# specs-physics

[![Build Status][bi]][bl]

[bi]: https://travis-ci.com/bamling/specs-physics.svg?branch=master
[bl]: https://travis-ci.com/bamling/specs-physics

**specs-physics** aims to be an easily usable and extendable [nphysics](https://www.nphysics.org/) physics engine integration for applications and games that utilise the [Specs ECS](https://slide-rs.github.io/specs/).

The dream is to *simply* create `Entity`s with a set of configurable `Component`s and have most of your physics covered, be it collision/proximity detection, velocity and acceleration or gravity.

## Usage

```toml
[dependencies]
specs-physics = "0.0.1"
```

### Example

```rust
extern crate log;
extern crate simple_logger;

use specs::{world::Builder, Component, DenseVecStorage, FlaggedStorage, World};
use specs_physics::{
    body::{BodyStatus, Position},
    dispatcher,
    PhysicsBodyBuilder,
    PhysicsColliderBuilder,
    Shape,
};

/// `Pos` struct for synchronisation of the position between the ECS and
/// nphysics; this has to implement both `Component` and `Position`
struct Pos {
    x: f32,
    y: f32,
    z: f32,
}

impl Component for Pos {
    type Storage = FlaggedStorage<Self, DenseVecStorage<Self>>;
}

impl Position<f32> for Pos {
    fn position(&self) -> (f32, f32, f32) {
        (self.x, self.y, self.z)
    }

    fn set_position(&mut self, x: f32, y: f32, z: f32) {
        self.x = x;
        self.y = y;
        self.z = z;
    }
}

fn main() {
    // initialise the logger for system logs
    simple_logger::init().unwrap();

    // initialise the Specs world; this will contain our Resources and Entities
    let mut world = World::new();

    // create the dispatcher containing all relevant Systems; alternatively to using
    // the convenience function you can add all required Systems by hand
    let mut dispatcher = dispatcher::<f32, Pos>();
    dispatcher.setup(&mut world.res);

    // create an Entity containing the required Components
    world
        .create_entity()
        .with(Pos {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        })
        .with(PhysicsBodyBuilder::<f32>::from(BodyStatus::Dynamic).build())
        .with(PhysicsColliderBuilder::<f32>::from(Shape::Rectangle(1.0, 1.0, 1.0)).build())
        .build();

    // execute the dispatcher
    dispatcher.dispatch(&world.res);
}
```

## Contributing 

I'd appreciate any kind of contribution to this project, be it feature requests, bugs/issues, pull requests, documentation, tests or examples! 

Please just try to format any code changes according to the [rustfmt.toml](https://github.com/bamling/specs-physics/blob/master/rustfmt.toml) rules. They're not exactly set in stone and I'm open for suggestions, but let's try to keep things tidy!

## License

Distributed under the MIT License. See [LICENSE](https://github.com/bamling/specs-physics/blob/master/LICENSE) for more information.

## Acknowledgments

This project is heavily inspired by [nphysics-ecs-dumb](https://github.com/distransient/nphysics-ecs-dumb); they did most of the heavy lifting, I'm just building up on what they have started!

**Special thanks to:**
- [distransient](https://github.com/distransient)
- [jojolepro](https://github.com/jojolepro)