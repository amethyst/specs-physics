use crate::PhysicsWorld;
use amethyst::core::Time;
use amethyst::ecs::{Read, System, WriteExpect};

/// Simulates a step of the physics world.
pub struct PhysicsStepperSystem {
    intended_timestep: f32,
}

impl Default for PhysicsStepperSystem {
    fn default() -> Self {
        PhysicsStepperSystem {
            intended_timestep: 1.0 / 120.0,
        }
    }
}

impl PhysicsStepperSystem {
    pub fn new(intended_timestep: f32) -> Self {
        PhysicsStepperSystem { intended_timestep }
    }
}

impl<'a> System<'a> for PhysicsStepperSystem {
    type SystemData = (WriteExpect<'a, PhysicsWorld>, Read<'a, Time>);

    // Simulate world using the current time frame
    // TODO: Bound timestep deltas
    fn run(&mut self, (mut physical_world, time): Self::SystemData) {
        if physical_world.timestep() != self.intended_timestep {
            warn!("Physics world timestep out of sync with intended timestep! Leave me alone!!!");
            physical_world.set_timestep(self.intended_timestep);
        }

        let mut delta = time.delta_seconds();

        while delta >= self.intended_timestep {
            physical_world.step();
            delta -= self.intended_timestep;
        }
    }
}
