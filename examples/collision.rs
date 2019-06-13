#[macro_use]
extern crate log;
extern crate simple_logger;

use specs::{world::Builder, Component, DenseVecStorage, FlaggedStorage, World};
use specs_physics::{
    bodies::{BodyStatus, Position},
    colliders::Shape,
    math::{Isometry3, Vector3},
    physics_dispatcher,
    PhysicsBodyBuilder,
    PhysicsColliderBuilder,
};

/// `SimpleTranslation` struct for synchronisation of the position between the
/// ECS and nphysics; this has to implement both `Component` and `Position`
struct SimpleTranslation {
    x: f32,
    y: f32,
    z: f32,
}

impl Component for SimpleTranslation {
    type Storage = FlaggedStorage<Self, DenseVecStorage<Self>>;
}

impl Position<f32> for SimpleTranslation {
    fn as_isometry(&self) -> Isometry3<f32> {
        Isometry3::translation(self.x, self.y, self.z)
    }

    fn set_isometry(&mut self, isometry: &Isometry3<f32>) {
        let translation = isometry.translation.vector;
        self.x = translation.x;
        self.y = translation.y;
        self.z = translation.z;
    }
}

fn main() {
    // initialise the logger for system logs
    simple_logger::init().unwrap();

    // initialise the Specs world; this will contain our Resources and Entities
    let mut world = World::new();

    // create the dispatcher containing all relevant Systems; alternatively to using
    // the convenience function you can add all required Systems by hand
    let mut dispatcher = physics_dispatcher::<f32, SimpleTranslation>();
    dispatcher.setup(&mut world.res);

    // create an Entity with a dynamic PhysicsBody component and a velocity
    let entity = world
        .create_entity()
        .with(SimpleTranslation {
            x: 1.0,
            y: 1.0,
            z: 1.0,
        })
        .with(
            PhysicsBodyBuilder::<f32>::from(BodyStatus::Dynamic)
                .velocity(Vector3::new(1.0, 0.0, 0.0))
                .build(),
        )
        .with(PhysicsColliderBuilder::<f32>::from(Shape::Rectangle(2.0, 2.0, 1.0)).build())
        .build();

    // create an Entity with a static PhysicsBody component right next to the first
    // one
    world
        .create_entity()
        .with(SimpleTranslation {
            x: 3.0,
            y: 1.0,
            z: 1.0,
        })
        .with(PhysicsBodyBuilder::<f32>::from(BodyStatus::Static).build())
        .with(PhysicsColliderBuilder::<f32>::from(Shape::Rectangle(2.0, 2.0, 1.0)).build())
        .build();

    // execute the dispatcher
    dispatcher.dispatch(&world.res);

    // fetch the translation component for the Entity with the dynamic body; the
    // position should still be approx the same
    let pos_storage = world.read_storage::<SimpleTranslation>();
    let pos = pos_storage.get(entity).unwrap();

    info!("updated position: x={}, y={}, z={},", pos.x, pos.y, pos.z);
}
