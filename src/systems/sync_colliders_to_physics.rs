use std::marker::PhantomData;

use specs::{
    storage::ComponentEvent,
    world::Index,
    Join,
    ReadStorage,
    ReaderId,
    System,
    SystemData,
    World,
    WriteExpect,
    WriteStorage,
};

use crate::{
    bodies::Position,
    colliders::PhysicsCollider,
    nalgebra::RealField,
    nphysics::object::{BodyPartHandle, ColliderDesc},
    Physics,
    PhysicsParent,
};

use super::iterate_component_events;

/// The `SyncCollidersToPhysicsSystem` handles the synchronisation of
/// `PhysicsCollider` `Component`s into the physics `World`.
pub struct SyncCollidersToPhysicsSystem<N, P> {
    positions_reader_id: Option<ReaderId<ComponentEvent>>,
    physics_colliders_reader_id: Option<ReaderId<ComponentEvent>>,

    n_marker: PhantomData<N>,
    p_marker: PhantomData<P>,
}

impl<'s, N, P> System<'s> for SyncCollidersToPhysicsSystem<N, P>
where
    N: RealField,
    P: Position<N>,
{
    type SystemData = (
        ReadStorage<'s, P>,
        ReadStorage<'s, PhysicsParent>,
        WriteExpect<'s, Physics<N>>,
        WriteStorage<'s, PhysicsCollider<N>>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (positions, parent_entities, mut physics, mut physics_colliders) = data;

        // collect all ComponentEvents for the Position storage
        let (inserted_positions, ..) =
            iterate_component_events(&positions, self.positions_reader_id.as_mut().unwrap());

        // collect all ComponentEvents for the PhysicsCollider storage
        let (inserted_physics_colliders, modified_physics_colliders, removed_physics_colliders) =
            iterate_component_events(
                &physics_colliders,
                self.physics_colliders_reader_id.as_mut().unwrap(),
            );

        // iterate over PhysicsCollider and Position components with an id/Index that
        // exists in either of the collected ComponentEvent BitSets
        for (position, parent_entity, mut physics_collider, id) in (
            &positions,
            parent_entities.maybe(),
            &mut physics_colliders.restrict_mut(),
            &inserted_positions
                | &inserted_physics_colliders
                | &modified_physics_colliders
                | &removed_physics_colliders,
        )
            .join()
        {
            // handle inserted events
            if inserted_positions.contains(id) || inserted_physics_colliders.contains(id) {
                debug!("Inserted PhysicsCollider with id: {}", id);
                add_collider::<N, P>(
                    id,
                    parent_entity,
                    &position,
                    &mut physics,
                    physics_collider.get_mut_unchecked(),
                );
            }

            // handle modified events
            if modified_physics_colliders.contains(id) {
                debug!("Modified PhysicsCollider with id: {}", id);
                update_collider::<N, P>(id, &mut physics, physics_collider.get_unchecked());
            }

            // handle removed events
            if removed_physics_colliders.contains(id) {
                debug!("Removed PhysicsCollider with id: {}", id);
                remove_collider::<N, P>(id, &mut physics);
            }
        }

        // Drain update triggers caused by inserts
        let event_iter = physics_colliders
            .channel()
            .read(self.physics_colliders_reader_id.as_mut().unwrap());
        for _ in event_iter {}
    }

    fn setup(&mut self, res: &mut World) {
        info!("SyncCollidersToPhysicsSystem.setup");
        Self::SystemData::setup(res);

        // initialise required resources
        res.entry::<Physics<N>>().or_insert_with(Physics::default);

        // register reader id for the Position storage
        let mut position_storage: WriteStorage<P> = SystemData::fetch(&res);
        self.positions_reader_id = Some(position_storage.register_reader());

        // register reader id for the PhysicsBody storage
        let mut physics_collider_storage: WriteStorage<PhysicsCollider<N>> =
            SystemData::fetch(&res);
        self.physics_colliders_reader_id = Some(physics_collider_storage.register_reader());
    }
}

impl<N, P> Default for SyncCollidersToPhysicsSystem<N, P>
where
    N: RealField,
    P: Position<N>,
{
    fn default() -> Self {
        Self {
            positions_reader_id: None,
            physics_colliders_reader_id: None,
            n_marker: PhantomData,
            p_marker: PhantomData,
        }
    }
}

fn add_collider<N, P>(
    id: Index,
    parent_entity: Option<&PhysicsParent>,
    position: &P,
    physics: &mut Physics<N>,
    physics_collider: &mut PhysicsCollider<N>,
) where
    N: RealField,
    P: Position<N>,
{
    // remove already existing colliders for this inserted event
    if let Some(handle) = physics.collider_handles.remove(&id) {
        warn!("Removing orphaned collider handle: {:?}", handle);
        physics.world.remove_colliders(&[handle]);
    }

    // attempt to find an existing RigidBody for this Index; if one exists we'll
    // fetch its BodyPartHandle and use it as the Colliders parent in the
    // nphysics World
    let parent_part_handle = match physics.body_handles.get(&id) {
        Some(parent_handle) => physics
            .world
            .rigid_body(*parent_handle)
            .map_or(BodyPartHandle::ground(), |body| body.part_handle()),
        None => {
            // if BodyHandle was found for the current Entity/Index, check for a potential
            // parent Entity and repeat the first step
            if let Some(parent_entity) = parent_entity {
                match physics.body_handles.get(&parent_entity.entity.id()) {
                    Some(parent_handle) => physics
                        .world
                        .rigid_body(*parent_handle)
                        .map_or(BodyPartHandle::ground(), |body| body.part_handle()),
                    None => {
                        // ultimately default to BodyPartHandle::ground()
                        BodyPartHandle::ground()
                    }
                }
            } else {
                // no parent Entity exists, default to BodyPartHandle::ground()
                BodyPartHandle::ground()
            }
        }
    };

    // translation based on parent handle; if we did not have a valid parent and
    // ended up defaulting to BodyPartHandle::ground(), we'll need to take the
    // Position into consideration
    let translation = if parent_part_handle.is_ground() {
        // let scale = 1.0; may be added later
        let iso = &mut position.isometry().clone();
        iso.translation.vector +=
            iso.rotation * physics_collider.offset_from_parent.translation.vector; //.component_mul(scale);
        iso.rotation *= physics_collider.offset_from_parent.rotation;
        *iso
    } else {
        physics_collider.offset_from_parent
    };

    // create the actual Collider in the nphysics World and fetch its handle
    let handle = ColliderDesc::new(physics_collider.shape_handle())
        .position(translation)
        .density(physics_collider.density)
        .material(physics_collider.material.clone())
        .margin(physics_collider.margin)
        .collision_groups(physics_collider.collision_groups)
        .linear_prediction(physics_collider.linear_prediction)
        .angular_prediction(physics_collider.angular_prediction)
        .sensor(physics_collider.sensor)
        .user_data(id)
        .build_with_parent(parent_part_handle, &mut physics.world)
        .unwrap()
        .handle();

    physics_collider.handle = Some(handle);
    physics.collider_handles.insert(id, handle);

    info!(
        "Inserted collider to world with values: {:?}",
        physics_collider
    );
}

fn update_collider<N, P>(id: Index, physics: &mut Physics<N>, physics_collider: &PhysicsCollider<N>)
where
    N: RealField,
    P: Position<N>,
{
    debug!("Modified PhysicsCollider with id: {}", id);
    let collider_handle = physics_collider.handle.unwrap();
    let collider_world = physics.world.collider_world_mut();

    // update collision groups
    collider_world.set_collision_groups(collider_handle, physics_collider.collision_groups);

    info!(
        "Updated collider in world with values: {:?}",
        physics_collider
    );
}

fn remove_collider<N, P>(id: Index, physics: &mut Physics<N>)
where
    N: RealField,
    P: Position<N>,
{
    debug!("Removed PhysicsCollider with id: {}", id);
    if let Some(handle) = physics.collider_handles.remove(&id) {
        // we have to check if the collider still exists in the nphysics World before
        // attempting to delete it as removing a collider that does not exist anymore
        // causes the nphysics World to panic; colliders are implicitly removed when a
        // parent body is removed so this is actually a valid scenario
        if physics.world.collider(handle).is_some() {
            physics.world.remove_colliders(&[handle]);
        }

        info!("Removed collider from world with id: {}", id);
    }
}

#[cfg(test)]
mod tests {
    use specs::prelude::*;

    use crate::{
        colliders::Shape,
        nalgebra::Isometry3,
        systems::SyncCollidersToPhysicsSystem,
        Physics,
        PhysicsColliderBuilder,
        SimplePosition,
    };

    #[test]
    fn add_collider() {
        let mut world = World::new();
        let mut dispatcher = DispatcherBuilder::new()
            .with(
                SyncCollidersToPhysicsSystem::<f32, SimplePosition<f32>>::default(),
                "sync_colliders_to_physics_system",
                &[],
            )
            .build();
        dispatcher.setup(&mut world);

        // create an Entity with the PhysicsCollider component and execute the
        // dispatcher
        world
            .create_entity()
            .with(SimplePosition::<f32>(Isometry3::<f32>::translation(
                1.0, 1.0, 1.0,
            )))
            .with(PhysicsColliderBuilder::<f32>::from(Shape::Ball { radius: 5.0 }).build())
            .build();
        dispatcher.dispatch(&world);

        // fetch the Physics instance and check for new colliders
        let physics = world.read_resource::<Physics<f32>>();
        assert_eq!(physics.collider_handles.len(), 1);
        assert_eq!(physics.world.colliders().count(), 1);
    }
}
