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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Physics;
    use nalgebra::Vector3;
    use specs::{DispatcherBuilder, World};

    #[test]
    fn update_gravity() {
        let mut world = World::new();
        let mut dispatcher = DispatcherBuilder::new()
            .with(
                SyncGravityToPhysicsSystem::<f32>::default(),
                "sync_gravity_to_physics_system",
                &[],
            )
            .build();
        dispatcher.setup(&mut world.res);

        world.add_resource(Gravity(Vector3::<f32>::new(1.0, 2.0, 3.0).into()));
        dispatcher.dispatch(&mut world.res);

        let physics = world.read_resource::<Physics<f32>>();
        assert_eq!(physics.world.gravity().x, 1.0);
        assert_eq!(physics.world.gravity().y, 2.0);
        assert_eq!(physics.world.gravity().z, 3.0);
    }

}
