use crate::bodies::DynamicBody;
use crate::Collider;
use crate::PhysicsWorld;
use amethyst::core::GlobalTransform;
use amethyst::ecs::storage::{ComponentEvent, GenericReadStorage, MaskedStorage};
use amethyst::ecs::{
    BitSet, Component, Entities, Join, ReadStorage, ReaderId, Resources, Storage, System,
    SystemData, Tracked, WriteExpect, WriteStorage,
};
use core::ops::Deref;
use nphysics::object::BodyHandle;

#[derive(Default, new)]
pub struct SyncCollidersToPhysicsSystem {
    #[new(default)]
    colliders_reader_id: Option<ReaderId<ComponentEvent>>,
}

impl<'a> System<'a> for SyncCollidersToPhysicsSystem {
    type SystemData = (
        WriteExpect<'a, PhysicsWorld>,
        Entities<'a>,
        ReadStorage<'a, GlobalTransform>,
        ReadStorage<'a, DynamicBody>,
        WriteStorage<'a, Collider>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (mut physical_world, entities, transforms, rigid_bodies, mut colliders) = data;
        // TODO: Check for inserted/removed rigid_bodies. parent sync https://nphysics.org/rustdoc/nphysics3d/world/struct.World.html?search=#method.add_collider
        let mut inserted_colliders = BitSet::new();
        let mut modified_colliders = BitSet::new();

        iterate_events(
            &transforms,
            self.colliders_reader_id.as_mut().unwrap(),
            &mut inserted_colliders,
            &mut modified_colliders,
            &mut physical_world,
            &entities,
            &colliders,
        );

        for (entity, mut collider, id) in (
            &entities,
            &mut colliders,
            &inserted_colliders | &modified_colliders,
        )
            .join()
        {
            if inserted_colliders.contains(id) {
                trace!("Detected inserted collider with id {}", id);

                // Just inserted. Remove old one and insert new.
                if let Some(handle) = collider.handle {
                    if physical_world.collider(handle).is_some() {
                        trace!("Removing collider marked as inserted that already exists with handle: {:?}", handle);

                        physical_world.remove_colliders(&[handle]);
                    }
                }

                let parent = if let Some(rb) = rigid_bodies.get(entity) {
                    trace!("Attaching inserted collider to rigid body: {}", entity);

                    rb.handle().expect(
                        "You should normally have a body handle at this point. This is a bug.",
                    )
                } else {
                    trace!("Attaching inserted collider to ground.");

                    BodyHandle::ground()
                };

                collider.handle = Some(physical_world.add_collider(
                    collider.margin,
                    collider.shape.clone(),
                    parent,
                    collider.offset_from_parent,
                    collider.physics_material.clone(),
                ));

                trace!("Inserted collider to world with values: {}", collider);

                let prediction = physical_world.prediction();
                let angular_prediction = physical_world.angular_prediction();

                let collision_world = physical_world.collision_world_mut();

                let collider_object = collision_world
                    .collision_object_mut(collider.handle.unwrap())
                    .unwrap();

                let collider_handle = collider_object.handle().clone();

                collision_world.set_collision_group(collider_handle, collider.collision_group);

                collider_object.set_query_type(collider.query_type.to_geometric_query_type(
                    collider.margin,
                    prediction,
                    angular_prediction,
                ));
            } else if modified_colliders.contains(id) || modified_colliders.contains(id) {
                println!("Detected changed collider with id {:?}", id);

                let prediction = physical_world.prediction();
                let angular_prediction = physical_world.angular_prediction();

                let collision_world = physical_world.collision_world_mut();
                let collider_handle = collision_world
                    .collision_object(collider.handle.unwrap())
                    .unwrap()
                    .handle()
                    .clone();

                collision_world.set_collision_group(collider_handle, collider.collision_group);
                collision_world.set_shape(collider_handle, collider.shape.clone());

                let collider_object = collision_world
                    .collision_object_mut(collider.handle.unwrap())
                    .unwrap();
                //collider_handle.set_shape(collider_handle.shape);
                collider_object.set_position(collider.offset_from_parent.clone());
                collider_object.set_query_type(collider.query_type.to_geometric_query_type(
                    collider.margin,
                    prediction,
                    angular_prediction,
                ));
                collider_object
                    .data_mut()
                    .set_material(collider.physics_material.clone());
            }
        }
    }

    fn setup(&mut self, res: &mut Resources) {
        Self::SystemData::setup(res);

        let mut collider_storage: WriteStorage<Collider> = SystemData::fetch(&res);
        self.colliders_reader_id = Some(collider_storage.register_reader());
    }
}

fn iterate_events<'a, T, D, S>(
    tracked_storage: &Storage<T, D>,
    reader: &mut ReaderId<ComponentEvent>,
    inserted: &mut BitSet,
    modified: &mut BitSet,
    world: &mut PhysicsWorld,
    entities: &Entities,
    colliders: &S,
) where
    T: Component,
    T::Storage: Tracked,
    D: Deref<Target = MaskedStorage<T>>,
    S: GenericReadStorage<Component = Collider>,
{
    let events = tracked_storage.channel().read(reader);

    for event in events {
        match event {
            ComponentEvent::Modified(id) => {
                modified.add(*id);
            }
            ComponentEvent::Inserted(id) => {
                inserted.add(*id);
            }
            ComponentEvent::Removed(id) => {
                match colliders.get(entities.entity(*id)) {
                    Some(collider) => {
                        match collider.handle {
                            Some(handle) => {
                                trace!("Removing collider with id: {}", id);

                                world.remove_colliders(&[handle]);
                            }
                            None => {
                                error!("Missing handle in collider: {}", id);
                            }
                        };
                    }
                    None => {
                        error!("Missing collider with id: {}", id);
                    }
                };
            }
        };
    }
}
