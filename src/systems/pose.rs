use crate::{nalgebra::RealField, position::Position, world::BodyComponent};
use specs::{Join, ReadStorage, System, WriteStorage};
use std::marker::PhantomData;

/// The `SyncBodiesFromPhysicsSystem` synchronised the updated position of
/// the `RigidBody`s in the nphysics `World` with their Specs counterparts. This
/// affects the `Position` `Component` related to the `Entity`.
pub struct PhysicsPoseSystem<N: RealField, P: Position<N>>(PhantomData<(N, P)>);

impl<'s, N: RealField, P: Position<N>> System<'s> for PhysicsPoseSystem<N, P> {
    type SystemData = (WriteStorage<'s, P>, ReadStorage<'s, BodyComponent<N>>);

    fn run(&mut self, (mut positions, bodies): Self::SystemData) {
        // iterate over all PhysicBody components joined with their Positions
        for (position, body) in (&mut positions, &bodies).join() {
            // if a RigidBody exists in the nphysics World we fetch it and update the
            // Position component accordingly
            if let Some(part) = body.part(0) {
                *position.isometry_mut() = part.position();
            }
        }
    }
}

impl<N: RealField, P: Position<N>> Default for PhysicsPoseSystem<N, P> {
    fn default() -> Self {
        Self(PhantomData)
    }
}
