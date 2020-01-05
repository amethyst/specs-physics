use crate::{
    bodies::BodySet,
    colliders::ColliderSet,
    joints::JointConstraintSet,
    nalgebra::{convert as na_convert, RealField},
    stepper::StepperRes,
    world::{ForceGeneratorSetRes, GeometricalWorldRes, MechanicalWorldRes},
};

use specs::{Read, System, WriteExpect};
use std::marker::PhantomData;

/// This system steps the physics world once when called.
/// To ensure the visual motion of the simulation matches the speeds within the
/// simulation, you will want to
pub struct PhysicsStepperSystem<N: RealField>(PhantomData<N>);

impl<'s, N: RealField> System<'s> for PhysicsStepperSystem<N> {
    type SystemData = (
        WriteExpect<'s, MechanicalWorldRes<N>>,
        WriteExpect<'s, GeometricalWorldRes<N>>,
        BodySet<'s, N>,
        ColliderSet<'s, N>,
        JointConstraintSet<'s, N>,
        WriteExpect<'s, ForceGeneratorSetRes<N>>,
        Option<Read<'s, StepperRes>>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            mut mechanical_world,
            mut geometrical_world,
            mut body_set,
            mut collider_set,
            mut joint_constraint_set,
            mut force_generator_set,
            step,
        ) = data;

        // If we've added a batch time step resource to the world, check if we need to
        // update our timestep from that resource.
        if let Some(step_data) = step {
            if step_data.is_dirty() {
                mechanical_world
                    .set_timestep(na_convert(step_data.current_time_step().as_secs_f64()));
            }
        }

        mechanical_world.step(
            &mut *geometrical_world,
            &mut body_set,
            &mut collider_set,
            &mut joint_constraint_set,
            &mut *force_generator_set,
        );
    }
}

impl<N: RealField> Default for PhysicsStepperSystem<N> {
    fn default() -> Self {
        Self(PhantomData)
    }
}
