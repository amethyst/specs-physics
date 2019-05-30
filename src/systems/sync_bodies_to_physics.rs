use std::marker::PhantomData;

use nalgebra::{RealField, Vector3};
use nphysics::{math::Velocity, object::RigidBodyDesc};
use specs::{
    storage::ComponentEvent,
    world::Index,
    Component,
    DenseVecStorage,
    FlaggedStorage,
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
    body::{PhysicsBody, Position},
    Physics,
};

use super::iterate_component_events;

pub struct SyncBodiesToPhysicsSystem<N, P> {
    positions_reader_id: Option<ReaderId<ComponentEvent>>,
    physics_bodies_reader_id: Option<ReaderId<ComponentEvent>>,

    n_marker: PhantomData<N>,
    p_marker: PhantomData<P>,
}

impl<'s, N, P> System<'s> for SyncBodiesToPhysicsSystem<N, P>
where
    N: RealField,
    P: Component<Storage = FlaggedStorage<P, DenseVecStorage<P>>> + Position<N> + Send + Sync,
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
            if modified_positions.contains(id) || modified_physics_bodies.contains(id) {}

            // handle removed events
            if removed_positions.contains(id) || removed_physics_bodies.contains(id) {}
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
    P: Component<Storage = FlaggedStorage<P, DenseVecStorage<P>>> + Position<N> + Send + Sync,
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
    P: Component<Storage = FlaggedStorage<P, DenseVecStorage<P>>> + Position<N> + Send + Sync,
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
        .translation(Vector3::new(
            position.position().0,
            position.position().1,
            position.position().2,
        ))
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

    physics_body.handle = Some(handle.clone());
    physics.body_handles.insert(id, handle);

    info!(
        "Inserted rigid body to world with values: {:?}",
        physics_body
    );
}
