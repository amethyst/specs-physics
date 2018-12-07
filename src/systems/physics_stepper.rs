use crate::World;
use amethyst::core::Time;
use amethyst::ecs::{Read, System, WriteExpect};

/// Simulates a step of the physics world.
#[derive(Default)]
pub struct PhysicsStepperSystem;

impl PhysicsStepperSystem {
    pub fn new() -> Self {
        Default::default()
    }
}

impl<'a> System<'a> for PhysicsStepperSystem {
    type SystemData = (WriteExpect<'a, World>, Read<'a, Time>);

    fn run(&mut self, (mut physical_world, time): Self::SystemData) {
        // Simulate world using the current time frame
        // TODO: Bound timestep deltas
        physical_world.set_timestep(time.delta_seconds());
        physical_world.step();
    }
}
