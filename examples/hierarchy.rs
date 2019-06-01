extern crate log;
extern crate simple_logger;

use specs::{world::Builder, Component, DenseVecStorage, FlaggedStorage, World};
use specs_physics::{
    body::{BodyStatus, Position},
    physics_dispatcher,
    PhysicsBodyBuilder,
    PhysicsColliderBuilder,
    PhysicsParent,
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
    let mut dispatcher = physics_dispatcher::<f32, Pos>();
    dispatcher.setup(&mut world.res);

    // create an Entity containing the required Components; this Entity will be the
    // parent
    let parent = world
        .create_entity()
        .with(Pos {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        })
        .with(PhysicsBodyBuilder::<f32>::from(BodyStatus::Dynamic).build())
        .with(PhysicsColliderBuilder::<f32>::from(Shape::Rectangle(1.0, 1.0, 1.0)).build())
        .build();

    // create the child Entity; if this Entity has its own PhysicsBody it'll more or
    // less be its own object in the nphysics World, however if it's just a
    // PhysicsCollider the parent/child hierarchy will actually take effect and the
    // collider will be attached to the parent
    let _child = world
        .create_entity()
        .with(Pos {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        })
        .with(
            PhysicsColliderBuilder::<f32>::from(Shape::Rectangle(1.0, 1.0, 1.0))
                .sensor(true)
                .build(),
        )
        .with(PhysicsParent { entity: parent })
        .build();

    // execute the dispatcher
    dispatcher.dispatch(&world.res);
}
