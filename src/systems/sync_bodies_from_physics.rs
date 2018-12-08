use nphysics3d::object::ColliderHandle;
use crate::colliders::Collider;
use crate::bodies::DynamicBody;
use crate::PhysicsWorld;
use amethyst::core::{GlobalTransform, Transform};
use amethyst::ecs::{Join, ReadStorage, System, Write, ReadExpect, WriteStorage, Entity, Entities};
use amethyst::ecs::world::EntitiesRes;
use amethyst::shrev::EventChannel;
use nalgebra::Vector3;
use ncollide3d::events::{ContactEvent, ProximityEvent};
use nphysics3d::object::Body;

// Might want to replace by better types.
pub type EntityContactEvent = (Entity, Entity, ContactEvent);
pub type EntityProximityEvent = (Entity, Entity, ProximityEvent);

#[derive(Default)]
pub struct SyncBodiesFromPhysicsSystem;

impl SyncBodiesFromPhysicsSystem {
    pub fn new() -> Self {
        Default::default()
    }
}

impl<'a> System<'a> for SyncBodiesFromPhysicsSystem {
    type SystemData = (
        Entities<'a>,
        ReadExpect<'a, PhysicsWorld>,
        Write<'a, EventChannel<EntityContactEvent>>,
        Write<'a, EventChannel<EntityProximityEvent>>,
        WriteStorage<'a, GlobalTransform>,
        WriteStorage<'a, DynamicBody>,
        ReadStorage<'a, Transform>,
        ReadStorage<'a, Collider>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            entities,
            physical_world,
            mut contact_events,
            mut proximity_events,
            mut global_transforms,
            mut physics_bodies,
            local_transforms,
            colliders,
        ) = data;

        // Apply the updated values of the simulated world to our Components
        #[allow(unused_mut)]
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

        let collision_world = physical_world.collision_world();

        contact_events.iter_write(collision_world.contact_events().iter().cloned().map(|ev| {
            let (handle1, handle2) = match ev {
                ContactEvent::Started(h1, h2) => (h1, h2),
                ContactEvent::Stopped(h1, h2) => (h1, h2),
            };

            let e1 = entity_from_handle(&entities, &colliders, &handle1).expect("Failed to find entity for collider.");
            let e2 = entity_from_handle(&entities, &colliders, &handle2).expect("Failed to find entity for collider.");
            (e1, e2, ev)
        }));

        proximity_events.iter_write(collision_world.proximity_events().iter().cloned().map(|ev| {
            let e1 = entity_from_handle(&entities, &colliders, &ev.collider1).expect("Failed to find entity for collider.");
            let e2 = entity_from_handle(&entities, &colliders, &ev.collider2).expect("Failed to find entity for collider.");
            (e1, e2, ev)
        }));

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

pub fn entity_from_handle(entities: &EntitiesRes, colliders: &ReadStorage<Collider>, handle: &ColliderHandle) -> Option<Entity> {
    (&*entities, colliders).join().find(|(_, c)| c.handle.expect("Collider has no handle and wasn't removed.") == *handle).map(|(e, _)| e)
}
