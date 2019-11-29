/*!
Data types used for the optional `PhysicsBatchSystem` stepper implementation.

If you choose to use some other way to perform fixed stepping, such as using Amethyst's fixed
dispatcher instead of [`PhysicsBatchSystem`], you can simply ignore this module.

[`PhysicsBatchSystem`]: ../systems/struct.PhysicsBatchSystem.html
*/

mod resource;
mod semi_fixed_step;

pub use resource::StepperRes;
pub use semi_fixed_step::{OutOfBoundsError, SemiFixedQualifierState, SemiFixedStep};

use std::{fmt, mem::drop, time::Duration};

/// Provides a constant fixed timestep for the stepper.
#[derive(Debug, Copy, Clone)]
pub struct FixedTimeStep(pub Duration);

impl Default for FixedTimeStep {
    fn default() -> Self {
        FixedTimeStep(Duration::from_secs(1) / 60)
    }
}

impl TimeStep for FixedTimeStep {
    fn current_time_step(&self) -> Duration {
        self.0
    }
}

/// A stepping implementation which decides what the timestep should be.
pub trait TimeStep: Send + Sync {
    /// Returns the delta for this timestep.
    fn current_time_step(&self) -> Duration;

    /// Called when the simulation is exhausting the aggregator at the indicated
    /// step.
    fn fast_at_step(&mut self, global_step_number: u64) {
        // Used to avoid underscore prefix lol.
        drop(global_step_number);
    }

    /// Called when the simulation is hitting frame limits and falling behind at
    /// the indicated step.
    fn degraded_at_step(&mut self, global_step_number: u64, info: SlowFrameError) {
        warn!(
            "Physics stepping has been postponed at step {} due to slowness. {}",
            global_step_number, info,
        );
    }
}

/// Error struct for slow frames.
// Max frame steps exceeded / Frame time limit excess
pub struct SlowFrameError(bool, Option<Duration>);

impl fmt::Display for SlowFrameError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            SlowFrameError(true, Some(duration)) => write!(
                f,
                r#"The stepper has hit the frame step limit
and exceeded the frame time limit by {:#?}."#,
                duration
            ),
            SlowFrameError(true, None) => write!(f, "The stepper has hit the frame step limit."),
            SlowFrameError(false, Some(duration)) => write!(
                f,
                "The stepper has exceeded the frame time limit by {:#?}.",
                duration
            ),
            SlowFrameError(false, None) => write!(f, "Haha, just kidding."),
        }
    }
}
