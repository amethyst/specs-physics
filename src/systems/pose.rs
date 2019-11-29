use crate::{bodies::BodyComponent, nalgebra::RealField, pose::Pose};
use specs::{Join, ReadStorage, System, WriteStorage};
use std::marker::PhantomData;

/// The `SyncBodiesFromPhysicsSystem` synchronised the updated position of
/// the `RigidBody`s in the nphysics `World` with their Specs counterparts. This
/// affects the `Position` `Component` related to the `Entity`.
pub struct PhysicsPoseSystem<N: RealField, P: Pose<N>>(PhantomData<(N, P)>);

impl<'s, N: RealField, P: Pose<N>> System<'s> for PhysicsPoseSystem<N, P> {
    type SystemData = (WriteStorage<'s, P>, ReadStorage<'s, BodyComponent<N>>);

    fn run(&mut self, (mut poses, bodies): Self::SystemData) {
        // iterate over all PhysicBody components joined with their Positions
        for (pose, body) in (&mut poses, &bodies).join() {
            // if a RigidBody exists in the nphysics World we fetch it and update the
            // Position component accordingly
            if let Some(part) = body.part(0) {
                pose.sync(&part.position());
            }
        }
    }
}

impl<N: RealField, P: Pose<N>> Default for PhysicsPoseSystem<N, P> {
    fn default() -> Self {
        Self(PhantomData)
    }
}
