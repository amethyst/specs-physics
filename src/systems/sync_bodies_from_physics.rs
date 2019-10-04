use crate::{
    handle::{BodyHandle, BodyPartHandle, EntityHandleExt},
    nalgebra::RealField,
    position::Position,
    BodySetType,
};

use specs::{Join, ReadExpect, ReadStorage, System, WriteStorage};

use std::{marker::PhantomData, ops::Deref};

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
        ReadExpect<'s, BodySetType<N>>,
        ReadStorage<'s, BodyHandle>,
        ReadStorage<'s, BodyPartHandle>,
        WriteStorage<'s, P>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (body_set, body_handles, body_part_handles, mut positions) = data;

        // iterate over all PhysicBody components joined with their Positions
        for (rigid_body, position) in (&body_set.deref().join(&body_handles), &mut positions).join()
        {
            // if a RigidBody exists in the nphysics World we fetch it and update the
            // Position component accordingly
            if let Some(body) = rigid_body {
                if let Some(part) = body.part(0) {
                    *position.isometry_mut() = part.position();
                }
            }
        }

        for (rigid_body, position) in (
            &body_set.deref().join_part(&body_part_handles),
            &mut positions,
        )
            .join()
        {
            // if a RigidBody exists in the nphysics World we fetch it and update the
            // Position component accordingly
            if let Some(body) = rigid_body {
                *position.isometry_mut() = body.position();
            }
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
