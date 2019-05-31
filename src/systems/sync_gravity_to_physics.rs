use nalgebra::RealField;
use specs::{Read, Resources, System, SystemData, WriteExpect};

use crate::{Gravity, Physics};
use std::marker::PhantomData;

/// The `SyncGravityToPhysicsSystem` synchronises the `Gravity` values with the
/// nphysics `World`s gravity. This way the gravity can be affected on demand.
///
/// The `Gravity` `Resource` is treated as *optional* and needs to be registered
/// in order to change affect the `World`s gravity.
pub struct SyncGravityToPhysicsSystem<N> {
    n_marker: PhantomData<N>,
}

impl<'s, N: RealField> System<'s> for SyncGravityToPhysicsSystem<N> {
    type SystemData = (Option<Read<'s, Gravity<N>>>, WriteExpect<'s, Physics<N>>);

    fn run(&mut self, data: Self::SystemData) {
        let (gravity, mut physics) = data;

        // if a Gravity resource exists, synchronise its values with the nphysics World
        if let Some(gravity) = gravity {
            physics.world.set_gravity(gravity.0);
        }
    }

    fn setup(&mut self, res: &mut Resources) {
        info!("SyncGravityToPhysicsSystem.setup");
        Self::SystemData::setup(res);

        // initialise required resources
        res.entry::<Physics<N>>().or_insert_with(Physics::default);
    }
}

impl<N> Default for SyncGravityToPhysicsSystem<N>
where
    N: RealField,
{
    fn default() -> Self {
        Self {
            n_marker: PhantomData,
        }
    }
}
