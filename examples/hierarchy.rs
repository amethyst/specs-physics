extern crate log;
extern crate simple_logger;

use specs::{world::World, Builder, WorldExt};
use specs_physics::{
    colliders::Shape,
    nalgebra::{Isometry3, Vector3},
    nphysics::object::BodyStatus,
    physics_dispatcher,
    PhysicsBodyBuilder,
    PhysicsColliderBuilder,
    PhysicsParent,
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

    // create an Entity containing the required Components; this Entity will be the
    // parent
    let parent = world
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

    // create the child Entity; if this Entity has its own PhysicsBody it'll more or
    // less be its own object in the nphysics World, however if it's just a
    // PhysicsCollider the parent/child hierarchy will actually take effect and the
    // collider will be attached to the parent
    let _child = world
        .create_entity()
        .with(SimplePosition::<f32>(Isometry3::<f32>::translation(
            1.0, 1.0, 1.0,
        )))
        .with(
            PhysicsColliderBuilder::<f32>::from(Shape::Cuboid {
                half_extents: Vector3::new(1.0, 1.0, 1.0),
            })
            .sensor(true)
            .build(),
        )
        .with(PhysicsParent { entity: parent })
        .build();

    // execute the dispatcher
    dispatcher.dispatch(&world);
}
