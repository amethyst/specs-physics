use crate::bodies::DynamicBody;
use crate::World;
use amethyst::core::{GlobalTransform, Transform};
use amethyst::ecs::{Join, ReadStorage, System, Write, WriteExpect, WriteStorage};
use amethyst::shrev::EventChannel;
use nalgebra::Vector3;
use ncollide3d::events::ContactEvent;
use nphysics3d::object::Body;

#[derive(Default)]
pub struct SyncBodiesFromPhysicsSystem;

impl SyncBodiesFromPhysicsSystem {
    pub fn new() -> Self {
        Default::default()
    }
}

impl<'a> System<'a> for SyncBodiesFromPhysicsSystem {
    type SystemData = (
        WriteExpect<'a, World>,
        Write<'a, EventChannel<ContactEvent>>,
        WriteStorage<'a, GlobalTransform>,
        WriteStorage<'a, DynamicBody>,
        ReadStorage<'a, Transform>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            mut physical_world,
            _contact_events,
            mut global_transforms,
            mut physics_bodies,
            local_transforms,
        ) = data;

        // Apply the updated values of the simulated world to our Components
        for (mut global_transform, mut body, local_transform) in (
            &mut global_transforms,
            &mut physics_bodies,
            local_transforms.maybe(),
        )
            .join()
        {
            let updated_body = physical_world.body(body.handle().unwrap());

            if updated_body.is_ground() || !updated_body.is_active() || updated_body.is_static() {
                continue;
            }

            match (body, updated_body) {
                (
                    DynamicBody::RigidBody(ref mut rigid_body),
                    Body::RigidBody(ref updated_rigid_body),
                ) => {
                    println!(
                        "super power: change mehhh! new pos: {:?}",
                        updated_rigid_body.position()
                    );

                    // TODO: Might get rid of the scale!!!
                    global_transform.0 = updated_rigid_body
                        .position()
                        .to_homogeneous()
                        .prepend_nonuniform_scaling(
                            local_transform
                                .map(|tr| tr.scale())
                                .unwrap_or(&Vector3::new(1.0, 1.0, 1.0)),
                        );

                    rigid_body.velocity = *updated_rigid_body.velocity();
                    let inertia = updated_rigid_body.inertia();
                    rigid_body.mass = inertia.linear;
                    rigid_body.angular_mass = inertia.angular;
                    rigid_body.center_of_mass = updated_rigid_body.center_of_mass();
                }
                (DynamicBody::Multibody(_multibody), Body::Multibody(_updated_multibody)) => {
                    // match updated_multibody.links().next() {
                    //    Some(link) => link.position(),
                    //    None => continue,
                    // };
                }
                _ => println!("Unexpected dynamics body pair."),
            };
        }

        // TODO: reader id from other system?
        // Now that we changed them all, let's remove all those pesky events that were generated.
        // global_transforms
        //     .channel()
        //     .read(&mut self.transforms_reader_id.as_mut().unwrap())
        //     .for_each(|_| ());
        // physics_bodies
        //     .channel()
        //     .read(&mut self.physics_bodies_reader_id.as_mut().unwrap())
        //     .for_each(|_| ());
    }
}
