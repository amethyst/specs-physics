use specs::{
    Builder, DispatcherBuilder, Join, ReadExpect, ReadStorage, System, World, WorldExt,
    WriteStorage,
};
use specs_physics::{
    ncollide::shape::{Ball, ShapeHandle},
    nphysics::object::{ColliderDesc, RigidBody, RigidBodyDesc},
    systems::{PhysicsBatchSystem, PhysicsBundle},
    world::MechanicalWorldRes,
    BodyComponent, EntityBuilderExt, SimplePosition,
};

fn main() {
    // Just as usual we initialize our World
    let mut world = World::new();

    // And create a dispatcher with systems our stepper depends on
    let mut physics_builder =
        DispatcherBuilder::new().with(MyPhysicsSystem, "my_physics_system", &[]);

    // However, when constructing our PhysicsBundle,
    // we must be sure to construct our bundle with a value for its stepper
    // Refer to the PhysicsBundle documentation to see its various constructor
    // options
    PhysicsBundle::<f32, SimplePosition<f32>>::stepper(60)
        .with_deps(&["my_physics_system"])
        .register(&mut world, &mut physics_builder);

    // And then create a second DispatcherBuilder, which will be our "per-frame"
    // dispatcher
    let mut dispatcher = DispatcherBuilder::new()
        // We add all of our fixed-step dispatching as a batch,
        // to be executed N times a frame to keep stepping in time
        .with_batch::<PhysicsBatchSystem<f32>>(physics_builder, "physics_bundle", &[])
        // And at the end of a frame, we render
        .with(MyRenderingSystem, "rendering_system", &["physics_bundle"])
        .build();
    dispatcher.setup(&mut world);

    // Everything else is just as normal.
    let collider = ColliderDesc::new(ShapeHandle::new(Ball::new(1.0)));
    world
        .create_entity()
        .with(SimplePosition::<f32>::default())
        .with_body::<f32, _>(RigidBodyDesc::new().build())
        .with_collider::<f32>(&collider)
        .build();

    // This executes our top level frame dispatcher,
    // which may call our fixed step dispatcher multiple times per frame.
    dispatcher.dispatch(&world);
}

struct MyPhysicsSystem;
impl<'s> System<'s> for MyPhysicsSystem {
    type SystemData = WriteStorage<'s, BodyComponent<f32>>;

    fn run(&mut self, mut bodies: Self::SystemData) {
        for (body,) in (&mut bodies,).join() {
            if let Some(_rigid_body) = body.downcast_mut::<RigidBody<f32>>() {
                // Operate on our bodies.
            }
        }
    }
}

struct MyRenderingSystem;
impl<'s> System<'s> for MyRenderingSystem {
    type SystemData = (
        ReadStorage<'s, SimplePosition<f32>>,
        ReadExpect<'s, MechanicalWorldRes<f32>>,
    );

    fn run(&mut self, (positions, mechanical_world): Self::SystemData) {
        for pos in (&positions,).join() {
            println!(
                "Body Position (X: {:?}, Y: {:?}, Z: {:?}) @ {:?}s",
                pos.0.translation.vector.x,
                pos.0.translation.vector.y,
                pos.0.translation.vector.z,
                mechanical_world.integration_parameters.t,
            );
        }
    }
}
