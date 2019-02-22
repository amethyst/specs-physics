use crate::bodies::DynamicBody;
use crate::Collider;
use crate::PhysicsWorld;
use amethyst::core::Transform;
use amethyst::ecs::storage::{ComponentEvent, GenericReadStorage, MaskedStorage};
use amethyst::ecs::{
    BitSet, Component, Entities, Join, ReadStorage, ReaderId, Resources, Storage, System,
    SystemData, Tracked, WriteExpect, WriteStorage,
};
use core::ops::Deref;
use nphysics::material::MaterialHandle;
use nphysics::object::{BodyHandle, BodyPartHandle, ColliderDesc};

#[derive(Default, new)]
pub struct SyncCollidersToPhysicsSystem {
    #[new(default)]
    colliders_reader_id: Option<ReaderId<ComponentEvent>>,
}

impl<'a> System<'a> for SyncCollidersToPhysicsSystem {
    type SystemData = (
        WriteExpect<'a, PhysicsWorld>,
        Entities<'a>,
        ReadStorage<'a, Transform>,
        ReadStorage<'a, DynamicBody>,
        WriteStorage<'a, Collider>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (mut physical_world, entities, transforms, rigid_bodies, mut colliders) = data;
        // TODO: Check for inserted/removed rigid_bodies. parent sync https://nphysics.org/rustdoc/nphysics3d/world/struct.World.html?search=#method.add_collider
        let mut inserted_colliders = BitSet::new();
        let mut modified_colliders = BitSet::new();

        iterate_events(
            &colliders,
            self.colliders_reader_id.as_mut().unwrap(),
            &mut inserted_colliders,
            &mut modified_colliders,
            &mut physical_world,
            &entities,
            &colliders,
        );

        for (entity, mut collider, id, tr) in (
            &entities,
            &mut colliders,
            &inserted_colliders | &modified_colliders,
            &transforms,
        )
            .join()
        {
            if inserted_colliders.contains(id) {
                trace!("Detected inserted collider with id {:?}", id);
                // Just inserted. Remove old one and insert new.
                if collider.handle.is_some()
                    && physical_world.collider(collider.handle.unwrap()).is_some()
                {
                    physical_world.remove_colliders(&[collider.handle.unwrap()]);
                }

                let parent = if let Some(rb) = rigid_bodies.get(entity) {
                    trace!("Attaching inserted collider to rigid body: {:?}", entity);

                    rb.handle.expect(
                        "You should normally have a body handle at this point. This is a bug.",
                    )
                } else {
                    BodyHandle::ground()
                };
                let position = if parent.is_ground() {
                    tr.isometry() * collider.offset_from_parent
                } else {
                    collider.offset_from_parent
                };

                //trace!("Inserted collider to world with values: {:?}", collider);

                let prediction = physical_world.prediction();
                let angular_prediction = 0.09;

                let parent_part_handle = physical_world
                    .rigid_body(parent)
                    .map(|body| body.part_handle())
                    .unwrap_or_else(BodyPartHandle::ground);

                collider.handle = Some(
                    ColliderDesc::new(collider.shape.clone())
                        .user_data(entity)
                        .margin(collider.margin)
                        .position(position)
                        .material(MaterialHandle::new(collider.physics_material))
                        .build_with_parent(parent_part_handle, &mut physical_world)
                        .unwrap()
                        .handle(),
                );

                let collision_world = physical_world.collider_world_mut();

                collision_world.as_collider_world_mut().set_query_type(
                    collider.handle.unwrap(),
                    collider.query_type.to_geometric_query_type(
                        collider.margin,
                        prediction,
                        angular_prediction,
                    ),
                );

                let collider_object = collision_world
                    .collider_mut(collider.handle.unwrap())
                    .unwrap();

                let collider_handle = collider_object.handle();

                collision_world.set_collision_groups(collider_handle, collider.collision_group);
            } else if modified_colliders.contains(id) || modified_colliders.contains(id) {
                trace!("Detected changed collider with id {:?}", id);

                let prediction = physical_world.prediction();
                let angular_prediction = 0.09;

                let collision_world = physical_world.collider_world_mut();
                let collider_handle = collision_world
                    .collider(collider.handle.unwrap())
                    .unwrap()
                    .handle();

                collision_world.set_collision_groups(collider_handle, collider.collision_group);
                collision_world
                    .as_collider_world_mut()
                    .set_shape(collider_handle, collider.shape.clone());

                let collider_object = collision_world
                    .collider_mut(collider.handle.unwrap())
                    .unwrap();

                let parent = if let Some(rb) = rigid_bodies.get(entity) {
                    trace!("Updating collider to rigid body: {:?}", entity);

                    rb.handle().expect(
                        "You should normally have a body handle at this point. This is a bug.",
                    )
                } else {
                    trace!("Updating collider to ground.");

                    BodyHandle::ground()
                };

                let position = if parent.is_ground() {
                    tr.isometry() * collider.offset_from_parent
                } else {
                    collider.offset_from_parent
                };

                collider_object.set_position(position);

                collision_world.as_collider_world_mut().set_query_type(
                    collider.handle.unwrap(),
                    collider.query_type.to_geometric_query_type(
                        collider.margin,
                        prediction,
                        angular_prediction,
                    ),
                );

                // TODO: Material changes & more complex mats than BasicMaterial
                /*collider_object
                .user_data_mut()
                .set_material(collider.physics_material.clone());*/
            }
        }

        colliders
            .channel()
            .read(&mut self.colliders_reader_id.as_mut().unwrap())
            .for_each(|_| ());
    }

    fn setup(&mut self, res: &mut Resources) {
        Self::SystemData::setup(res);

        let mut collider_storage: WriteStorage<Collider> = SystemData::fetch(&res);
        self.colliders_reader_id = Some(collider_storage.register_reader());
    }
}

fn iterate_events<T, D, S>(
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
