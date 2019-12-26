#[macro_use]
extern crate log;
extern crate simple_logger;

use specs::{Builder, World, WorldExt};
use specs_physics::{
    colliders::Shape,
    nalgebra::{Isometry3, Vector3},
    nphysics::{algebra::Velocity3, object::BodyStatus},
    physics_dispatcher,
    PhysicsBodyBuilder,
    PhysicsColliderBuilder,
    SimplePosition,
};

fn main() {
    // initialise the logger for system logs
    simple_logger::init().unwrap();

    // initialise the Specs world; this will contain our Resources and Entities
    let mut world = World::new();

    // create the dispatcher containing all relevant Systems; alternatively to using
    // the convenience function you can add all required Systems by hand
    let mut dispatcher = physics_dispatcher::<f32, SimplePosition<f32>>();
    dispatcher.setup(&mut world);

    // create an Entity with a dynamic PhysicsBody component and a velocity
    let entity = world
        .create_entity()
        .with(SimplePosition::<f32>(Isometry3::<f32>::translation(
            1.0, 1.0, 1.0,
        )))
        .with(
            PhysicsBodyBuilder::<f32>::from(BodyStatus::Dynamic)
                .velocity(Velocity3::linear(1.0, 0.0, 0.0))
                .build(),
        )
        .with(
            PhysicsColliderBuilder::<f32>::from(Shape::Cuboid {
                half_extents: Vector3::new(2.0, 2.0, 1.0),
            })
            .build(),
        )
        .build();

    // create an Entity with a static PhysicsBody component right next to the first
    // one
    world
        .create_entity()
        .with(SimplePosition::<f32>(Isometry3::<f32>::translation(
            3.0, 1.0, 1.0,
        )))
        .with(PhysicsBodyBuilder::<f32>::from(BodyStatus::Static).build())
        .with(
            PhysicsColliderBuilder::<f32>::from(Shape::Cuboid {
                half_extents: Vector3::new(2.0, 2.0, 1.0),
            })
            .build(),
        )
        .build();

    // execute the dispatcher
    dispatcher.dispatch(&world);

    // fetch the translation component for the Entity with the dynamic body; the
    // position should still be approx the same
    let pos_storage = world.read_storage::<SimplePosition<f32>>();
    let pos = pos_storage.get(entity).unwrap();

    info!("updated position: {}", pos.0.translation);
}
