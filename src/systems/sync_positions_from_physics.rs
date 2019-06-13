use nalgebra::RealField;
use specs::{Join, ReadExpect, ReadStorage, Resources, System, SystemData, WriteStorage};
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
    P: Position<N>,
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
                position.set_isometry(rigid_body.position());
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
    P: Position<N>,
{
    fn default() -> Self {
        Self {
            n_marker: PhantomData,
            p_marker: PhantomData,
        }
    }
}
