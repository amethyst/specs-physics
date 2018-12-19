use crate::PhysicsWorld;
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
    type SystemData = (WriteExpect<'a, PhysicsWorld>, Read<'a, Time>);

    // Simulate world using the current time frame
    // TODO: Bound timestep deltas
    fn run(&mut self, (mut physical_world, time): Self::SystemData) {
        let delta = time.delta_seconds();

        trace!("Setting timestep with delta: {}", delta);
        physical_world.set_timestep(delta);

        trace!("Stepping physical world simulation.");
        physical_world.step();
    }
}
