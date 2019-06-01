use nalgebra::{Isometry3, RealField};
use specs::{
    Component,
    Join,
    ReadExpect,
    ReadStorage,
    Resources,
    System,
    SystemData,
    WriteStorage,
};
use std::marker::PhantomData;

use crate::{
    bodies::{PhysicsBody, Position},
    Physics,
};

/// The `SyncPositionsFromPhysicsSystem` synchronised the updated position of
/// the `RigidBody`s in the nphysics `World` with their Specs counterparts. This
/// affects the `Position` `Component` related to the `Entity`.
pub struct SyncPositionsFromPhysicsSystem<N, P> {
    n_marker: PhantomData<N>,
    p_marker: PhantomData<P>,
}

impl<'s, N, P> System<'s> for SyncPositionsFromPhysicsSystem<N, P>
where
    N: RealField,
    P: Component + Position<N> + Send + Sync,
{
    type SystemData = (
        ReadExpect<'s, Physics<N>>,
        ReadStorage<'s, PhysicsBody<N>>,
        WriteStorage<'s, P>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (physics, physics_bodies, mut positions) = data;

        // iterate over all PhysicBody components joined with their Positions
        for (physics_body, position) in (&physics_bodies, &mut positions).join() {
            // if a RigidBody exists in the nphysics World we fetch it and update the
            // Position component accordingly
            if let Some(rigid_body) = physics.world.rigid_body(physics_body.handle.unwrap()) {
                let isometry: &Isometry3<N> = rigid_body.position();

                position.set_position(
                    isometry.translation.vector.x,
                    isometry.translation.vector.y,
                    isometry.translation.vector.z,
                );
            }
        }
    }

    fn setup(&mut self, res: &mut Resources) {
        info!("SyncPositionsFromPhysicsSystem.setup");
        Self::SystemData::setup(res);

        // initialise required resources
        res.entry::<Physics<N>>().or_insert_with(Physics::default);
    }
}

impl<N, P> Default for SyncPositionsFromPhysicsSystem<N, P>
where
    N: RealField,
    P: Component + Position<N> + Send + Sync,
{
    fn default() -> Self {
        Self {
            n_marker: PhantomData,
            p_marker: PhantomData,
        }
    }
}
