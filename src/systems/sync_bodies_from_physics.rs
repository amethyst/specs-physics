use crate::bodies::DynamicBody;
use crate::colliders::Collider;
use crate::PhysicsWorld;
use amethyst::core::{GlobalTransform, Transform};
use amethyst::ecs::world::EntitiesRes;
use amethyst::ecs::{Entities, Entity, Join, ReadExpect, ReadStorage, System, Write, WriteStorage};
use amethyst::shrev::EventChannel;
use nalgebra::Vector3;
use ncollide3d::events::{ContactEvent, ProximityEvent};
use nphysics3d::object::ColliderHandle;

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
        WriteStorage<'a, Transform>,
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
            mut local_transforms,
            colliders,
        ) = data;

        trace!("Synchronizing bodies from physical world.");

        // Apply the updated values of the simulated world to our Components
        #[allow(unused_mut)]
        for (mut global_transform, mut body, mut local_transform) in (
            &mut global_transforms,
            &mut physics_bodies,
            (&mut local_transforms).maybe(),
        )
            .join()
        {
            if let Some(updated_body_handle) = body.handle() {
                if let Some(updated_body) = physical_world.rigid_body(updated_body_handle) {
                    if !updated_body.is_active() || updated_body.is_static() {
                        trace!(
                            "Skipping synchronizing data from non-dynamic body: {:?}",
                            updated_body.handle()
                        );
                        continue;
                    }

                    trace!(
                        "Synchronizing RigidBody from handle: {:?}",
                        updated_body.handle()
                    );

                    trace!(
                        "Synchronized RigidBody's updated position: {:?}",
                        updated_body.position()
                    );

                    global_transform.0 = updated_body
                        .position()
                        .to_homogeneous()
                        .prepend_nonuniform_scaling(
                            &local_transform
                                .as_ref()
                                .map(|tr| *tr.scale())
                                .unwrap_or_else(|| Vector3::new(1.0, 1.0, 1.0)),
                        );

                    if let Some(ref mut local_transform) = local_transform {
                        *local_transform.isometry_mut() = updated_body.position();
                    }

                    trace!(
                        "Synchronized RigidBody's updated velocity: {:?}",
                        updated_body.velocity()
                    );
                    body.velocity = *updated_body.velocity();

                    trace!(
                        "Synchronized RigidBody's updated inertia: {:?}",
                        updated_body.inertia()
                    );
                    let inertia = updated_body.inertia();
                    body.mass = inertia.linear;
                    body.angular_mass = inertia.angular;

                    trace!(
                        "Synchronized RigidBody's updated center of mass: {:?}",
                        updated_body.center_of_mass()
                    );
                    body.center_of_mass = updated_body.center_of_mass();
                } else {
                    error!("Found body without pair in physics world!");
                }
            } else {
                error!("Found body without handle!");
            }
        }

        trace!("Iterating collision events.");

        let collision_world = physical_world.collision_world();

        contact_events.iter_write(collision_world.contact_events().iter().cloned().map(|ev| {
            trace!("Emitting contact event: {:?}", ev);

            let (handle1, handle2) = match ev {
                ContactEvent::Started(h1, h2) => (h1, h2),
                ContactEvent::Stopped(h1, h2) => (h1, h2),
            };

            let e1 = entity_from_handle(&entities, &colliders, handle1)
                .expect("Failed to find entity for collider.");
            let e2 = entity_from_handle(&entities, &colliders, handle2)
                .expect("Failed to find entity for collider.");
            (e1, e2, ev)
        }));

        proximity_events.iter_write(
            collision_world
                .proximity_events()
                .iter()
                .cloned()
                .map(|ev| {
                    trace!("Emitting proximity event: {:?}", ev);

                    let e1 = entity_from_handle(&entities, &colliders, ev.collider1)
                        .expect("Failed to find entity for collider.");
                    let e2 = entity_from_handle(&entities, &colliders, ev.collider2)
                        .expect("Failed to find entity for collider.");
                    (e1, e2, ev)
                }),
        );
    }
}

pub fn entity_from_handle(
    entities: &EntitiesRes,
    colliders: &ReadStorage<Collider>,
    handle: ColliderHandle,
) -> Option<Entity> {
    (&*entities, colliders)
        .join()
        .find(|(_, c)| {
            c.handle
                .expect("Collider has no handle and wasn't removed.")
                == handle
        })
        .map(|(e, _)| e)
}
