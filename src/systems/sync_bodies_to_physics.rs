use std::marker::PhantomData;

use nalgebra::RealField;
use nphysics::{
    math::Velocity,
    object::{Body, RigidBodyDesc},
};
use specs::{
    storage::ComponentEvent,
    world::Index,
    BitSet,
    Join,
    ReadStorage,
    ReaderId,
    Resources,
    System,
    SystemData,
    WriteExpect,
    WriteStorage,
};

use crate::{
    bodies::{PhysicsBody, Position},
    Physics,
};

use super::iterate_component_events;

/// The `SyncBodiesToPhysicsSystem` handles the synchronisation of `PhysicsBody`
/// `Component`s into the physics `World`.
pub struct SyncBodiesToPhysicsSystem<N, P> {
    positions_reader_id: Option<ReaderId<ComponentEvent>>,
    physics_bodies_reader_id: Option<ReaderId<ComponentEvent>>,

    n_marker: PhantomData<N>,
    p_marker: PhantomData<P>,
}

impl<'s, N, P> System<'s> for SyncBodiesToPhysicsSystem<N, P>
where
    N: RealField,
    P: Position<N>,
{
    type SystemData = (
        ReadStorage<'s, P>,
        WriteExpect<'s, Physics<N>>,
        WriteStorage<'s, PhysicsBody<N>>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (positions, mut physics, mut physics_bodies) = data;

        // collect all ComponentEvents for the Position storage
        let (inserted_positions, modified_positions, removed_positions) =
            iterate_component_events(&positions, self.positions_reader_id.as_mut().unwrap());

        // collect all ComponentEvents for the PhysicsBody storage
        let (inserted_physics_bodies, modified_physics_bodies, removed_physics_bodies) =
            iterate_component_events(
                &physics_bodies,
                self.physics_bodies_reader_id.as_mut().unwrap(),
            );

        // iterate over PhysicsBody and Position components with an id/Index that
        // exists in either of the collected ComponentEvent BitSets
        for (position, mut physics_body, id) in (
            &positions,
            &mut physics_bodies,
            &inserted_positions
                | &modified_positions
                | &removed_positions
                | &inserted_physics_bodies
                | &modified_physics_bodies
                | &removed_physics_bodies,
        )
            .join()
        {
            // handle inserted events
            if inserted_positions.contains(id) || inserted_physics_bodies.contains(id) {
                debug!("Inserted PhysicsBody with id: {}", id);
                add_rigid_body::<N, P>(id, &position, &mut physics, &mut physics_body);
            }

            // handle modified events
            if modified_positions.contains(id) || modified_physics_bodies.contains(id) {
                debug!("Modified PhysicsBody with id: {}", id);
                update_rigid_body::<N, P>(
                    id,
                    &position,
                    &mut physics,
                    &mut physics_body,
                    &modified_positions,
                    &modified_physics_bodies,
                );
            }

            // handle removed events
            if removed_positions.contains(id) || removed_physics_bodies.contains(id) {
                debug!("Removed PhysicsBody with id: {}", id);
                remove_rigid_body::<N, P>(id, &mut physics);
            }
        }
    }

    fn setup(&mut self, res: &mut Resources) {
        info!("SyncBodiesToPhysicsSystem.setup");
        Self::SystemData::setup(res);

        // initialise required resources
        res.entry::<Physics<N>>().or_insert_with(Physics::default);

        // register reader id for the Position storage
        let mut position_storage: WriteStorage<P> = SystemData::fetch(&res);
        self.positions_reader_id = Some(position_storage.register_reader());

        // register reader id for the PhysicsBody storage
        let mut physics_body_storage: WriteStorage<PhysicsBody<N>> = SystemData::fetch(&res);
        self.physics_bodies_reader_id = Some(physics_body_storage.register_reader());
    }
}

impl<N, P> Default for SyncBodiesToPhysicsSystem<N, P>
where
    N: RealField,
    P: Position<N>,
{
    fn default() -> Self {
        Self {
            positions_reader_id: None,
            physics_bodies_reader_id: None,
            n_marker: PhantomData,
            p_marker: PhantomData,
        }
    }
}

fn add_rigid_body<N, P>(
    id: Index,
    position: &P,
    physics: &mut Physics<N>,
    physics_body: &mut PhysicsBody<N>,
) where
    N: RealField,
    P: Position<N>,
{
    // remove already existing bodies for this inserted component;
    // this technically should never happen but we need to keep the list of body
    // handles clean
    if let Some(body_handle) = physics.body_handles.remove(&id) {
        warn!("Removing orphaned body handle: {:?}", body_handle);
        physics.world.remove_bodies(&[body_handle]);
    }

    let delta_time = physics.world.timestep();

    // create a new RigidBody in the PhysicsWorld and store its
    // handle for later usage
    let handle = RigidBodyDesc::new()
        .position(*position.isometry())
        .gravity_enabled(physics_body.gravity_enabled)
        .status(physics_body.body_status)
        .velocity(Velocity::linear(
            physics_body.velocity.x / delta_time,
            physics_body.velocity.y / delta_time,
            physics_body.velocity.z / delta_time,
        ))
        .angular_inertia(physics_body.angular_inertia)
        .mass(physics_body.mass)
        .local_center_of_mass(physics_body.local_center_of_mass)
        .user_data(id)
        .build(&mut physics.world)
        .handle();

    physics_body.handle = Some(handle);
    physics.body_handles.insert(id, handle);

    info!(
        "Inserted rigid body to world with values: {:?}",
        physics_body
    );
}

fn update_rigid_body<N, P>(
    id: Index,
    position: &P,
    physics: &mut Physics<N>,
    physics_body: &mut PhysicsBody<N>,
    modified_positions: &BitSet,
    modified_physics_bodies: &BitSet,
) where
    N: RealField,
    P: Position<N>,
{
    let delta_time = physics.world.timestep();

    if let Some(rigid_body) = physics.world.rigid_body_mut(physics_body.handle.unwrap()) {
        // the PhysicsBody was modified, update everything but the position
        if modified_physics_bodies.contains(id) {
            rigid_body.enable_gravity(physics_body.gravity_enabled);
            rigid_body.set_status(physics_body.body_status);
            rigid_body.set_velocity(Velocity::linear(
                physics_body.velocity.x / delta_time,
                physics_body.velocity.y / delta_time,
                physics_body.velocity.z / delta_time,
            ));
            rigid_body.set_angular_inertia(physics_body.angular_inertia);
            rigid_body.set_mass(physics_body.mass);
            rigid_body.set_local_center_of_mass(physics_body.local_center_of_mass);
        }

        // the Position was modified, update the position directly
        if modified_positions.contains(id) {
            rigid_body.set_position(*position.isometry());
        }

        trace!(
            "Updated rigid body in world with values: {:?}",
            physics_body
        );
    }
}

fn remove_rigid_body<N, P>(id: Index, physics: &mut Physics<N>)
where
    N: RealField,
    P: Position<N>,
{
    if let Some(handle) = physics.body_handles.remove(&id) {
        // remove body if it still exists in the PhysicsWorld
        physics.world.remove_bodies(&[handle]);
        info!("Removed rigid body from world with id: {}", id);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        bodies::{util::SimplePosition, BodyStatus},
        math::Isometry3,
        PhysicsBodyBuilder,
    };
    use specs::{world::Builder, DispatcherBuilder, World};

    #[test]
    fn add_rigid_body() {
        let mut world = World::new();
        let mut dispatcher = DispatcherBuilder::new()
            .with(
                SyncBodiesToPhysicsSystem::<f32, SimplePosition<f32>>::default(),
                "sync_bodies_to_physics_system",
                &[],
            )
            .build();
        dispatcher.setup(&mut world.res);

        // create an Entity with the PhysicsBody component and execute the dispatcher
        world
            .create_entity()
            .with(SimplePosition::<f32>(Isometry3::<f32>::translation(
                1.0, 1.0, 1.0,
            )))
            .with(PhysicsBodyBuilder::<f32>::from(BodyStatus::Dynamic).build())
            .build();
        dispatcher.dispatch(&mut world.res);

        // fetch the Physics instance and check for new bodies
        let physics = world.read_resource::<Physics<f32>>();
        assert_eq!(physics.body_handles.len(), 1);
        assert_eq!(physics.world.bodies().count(), 1);
    }
}
