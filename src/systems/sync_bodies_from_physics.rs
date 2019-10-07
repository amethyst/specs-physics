use crate::{
    nalgebra::RealField,
    position::Position,
    world::ReadBodyStorage,
};

use specs::{Join, System, WriteStorage};

use std::{marker::PhantomData};

/// The `SyncBodiesFromPhysicsSystem` synchronised the updated position of
/// the `RigidBody`s in the nphysics `World` with their Specs counterparts. This
/// affects the `Position` `Component` related to the `Entity`.
pub struct SyncBodiesFromPhysicsSystem<N, P> {
    n_marker: PhantomData<N>,
    p_marker: PhantomData<P>,
}

impl<'s, N, P> System<'s> for SyncBodiesFromPhysicsSystem<N, P>
where
    N: RealField,
    P: Position<N>,
{
    type SystemData = (
        WriteStorage<'s, P>,
        ReadBodyStorage<'s, N>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (mut positions, body_set) = data;

        // iterate over all PhysicBody components joined with their Positions
        for (body, position) in (&body_set, &mut positions).join() {
            // if a RigidBody exists in the nphysics World we fetch it and update the
            // Position component accordingly
            *position.isometry_mut() = body.position();
        }
    }
}

impl<N, P> Default for SyncBodiesFromPhysicsSystem<N, P>
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
