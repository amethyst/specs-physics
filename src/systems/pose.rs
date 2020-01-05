use crate::{bodies::{BodyComponent, BodyPartHandle}, nalgebra::RealField, pose::Pose};
use specs::{Join, ReadStorage, System, WriteStorage};
use std::marker::PhantomData;

/// The `SyncBodiesFromPhysicsSystem` synchronised the updated position of
/// the `RigidBody`s in the nphysics `World` with their Specs counterparts. This
/// affects the `Position` `Component` related to the `Entity`.
pub struct PhysicsPoseSystem<N: RealField, P: Pose<N>>(PhantomData<(N, P)>);

// TODO: Add logging to me!
impl<'s, N: RealField, P: Pose<N>> System<'s> for PhysicsPoseSystem<N, P> {
    type SystemData = (
        WriteStorage<'s, P>, 
        ReadStorage<'s, BodyComponent<N>>,
        ReadStorage<'s, BodyPartHandle>,
    );

    fn run(&mut self, (mut poses, bodies, handles): Self::SystemData) {
        // Iterate over all BodyPartHandles and apply their pose.
        for (pose, handle) in (&mut poses, &handles).join() {
            if let Some(body) = bodies.get(handle.0) {
                if let Some(part) = body.part(handle.1) {
                    pose.sync(&part.position());
                }
            }
        }

        // Iterate over all Body Components without Handles and apply their pose.
        for (pose, body, _) in (&mut poses, &bodies, !&handles).join() {
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
