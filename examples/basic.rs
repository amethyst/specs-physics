extern crate log;
extern crate simple_logger;

use specs::{Builder, World, WorldExt};
use specs_physics::{
    colliders::Shape,
    nalgebra::{Isometry3, Vector3},
    nphysics::object::BodyStatus,
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

    // create an Entity containing the required Components
    world
        .create_entity()
        .with(SimplePosition::<f32>(Isometry3::<f32>::translation(
            1.0, 1.0, 1.0,
        )))
        .with(PhysicsBodyBuilder::<f32>::from(BodyStatus::Dynamic).build())
        .with(
            PhysicsColliderBuilder::<f32>::from(Shape::Cuboid {
                half_extents: Vector3::new(1.0, 1.0, 1.0),
            })
            .build(),
        )
        .build();

    // execute the dispatcher
    dispatcher.dispatch(&world);
}
