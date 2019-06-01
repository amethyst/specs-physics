use nalgebra::RealField;
use specs::{Read, Resources, System, SystemData, WriteExpect};
use std::marker::PhantomData;

use crate::{Physics, TimeStep};

/// The `PhysicsStepperSystem` progresses the nphysics `World`.
pub struct PhysicsStepperSystem<N> {
    n_marker: PhantomData<N>,
}

impl<'s, N: RealField> System<'s> for PhysicsStepperSystem<N> {
    type SystemData = (Option<Read<'s, TimeStep<N>>>, WriteExpect<'s, Physics<N>>);

    fn run(&mut self, data: Self::SystemData) {
        let (time_step, mut physics) = data;

        // if a TimeStep resource exits, set the timestep for the nphysics integration
        // accordingly; this should not be required if the Systems are executed in a
        // fixed interval
        if let Some(time_step) = time_step {
            // only update timestep if it actually differs from the current nphysics World
            // one; keep in mind that changing the Resource will destabilize the simulation
            if physics.world.timestep() != time_step.0 {
                warn!(
                    "TimeStep and world.timestep() differ, changing worlds timestep from {} to: {:?}",
                    physics.world.timestep(),
                    time_step.0
                );
                physics.world.set_timestep(time_step.0);
            }
        }

        physics.world.step();
    }

    fn setup(&mut self, res: &mut Resources) {
        info!("PhysicsStepperSystem.setup");
        Self::SystemData::setup(res);

        // initialise required resources
        res.entry::<Physics<N>>().or_insert_with(Physics::default);
    }
}

impl<N> Default for PhysicsStepperSystem<N>
where
    N: RealField,
{
    fn default() -> Self {
        Self {
            n_marker: PhantomData,
        }
    }
}
