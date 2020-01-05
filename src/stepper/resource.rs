use super::{FixedTimeStep, SlowFrameError, TimeStep};

use std::time::{Duration, Instant};

/**
Resource used for fixed stepping within [`PhysicsBatchSystem`].

Should be inserted into the *Specs* World before `PhysicsBatchSystem` is called.

[`PhysicsBatchSystem`]: ../systems/struct.PhysicsBatchSystem.html
*/
pub struct StepperRes {
    /// When set to Some, the Batch system will only execute that many steps in
    /// one dispatch/frame. Useful for preventing death spirals in the stepper
    /// when your application does not require net synchronization.
    pub max_steps_per_frame: Option<u32>,

    /// When set to Some, the Batch system will postpone remaining steps to the
    /// next dispatch/frame after stepping for this time or longer. Useful for
    /// preventing death spirals in the stepper when your application does
    /// not require net synchronization.
    pub frame_time_limit: Option<Duration>,

    // Timestep interval state data
    pub time_step: Box<dyn TimeStep>,

    // Tracks how far "behind" physics time we are
    accumulator: Duration,

    is_dirty: bool,
    last_delta: Duration,

    // Number of steps since start
    // Safety: Given a liberal timestep of 120hz, this would take 4.8b years to saturate.
    // A u32 on the other hand would take 414 days to saturate. War Boys voice MEDIOCRE!
    global_steps: u64,

    // Steps in current iteration
    // Safety: Let's pray that nobody has a frame that takes 414 days to render. WITNESS ME!
    frame_steps: u32,

    // Tracks when the current frame began,
    frame_start: Option<Instant>,
}

impl StepperRes {
    pub fn new<T: TimeStep + 'static>(time_step: T) -> Self {
        Self::new_with_limits(time_step, None, None)
    }

    pub fn new_fixed(interval: u32) -> Self {
        Self::new(FixedTimeStep(Duration::from_secs(1) / interval))
    }

    pub fn new_with_limits<T: TimeStep + 'static>(
        time_step: T,
        max_steps_per_frame: Option<u32>,
        frame_time_limit: Option<Duration>,
    ) -> Self {
        Self {
            max_steps_per_frame,
            frame_time_limit,
            time_step: Box::new(time_step),
            accumulator: Duration::default(),
            is_dirty: true,
            // This forces updating on first iteration
            last_delta: Duration::from_millis(0),
            global_steps: 0,
            frame_steps: 0,
            frame_start: None,
        }
    }

    pub fn accumulator(&self) -> Duration {
        self.accumulator
    }

    pub fn current_time_step(&self) -> Duration {
        self.time_step.current_time_step()
    }

    pub fn frame_steps(&self) -> u32 {
        self.frame_steps
    }

    pub fn global_steps(&self) -> u64 {
        self.global_steps
    }

    pub fn is_dirty(&self) -> bool {
        self.is_dirty
    }

    fn check_if_frame_slow(&self) -> Option<SlowFrameError> {
        // Check if we've ran past the frame step limit
        let max_steps_failure = self
            .max_steps_per_frame
            .map_or(false, |max| self.frame_steps >= max);

        // Check if we've ran past the frame time limit, and calculate how much by
        let frame_time_limit_failure =
            if let Some(frame_duration) = self.frame_start.map(|x| x.elapsed()) {
                self.frame_time_limit.and_then(|limit| {
                    if frame_duration > limit {
                        Some(frame_duration - limit)
                    } else {
                        None
                    }
                })
            } else {
                None
            };

        if max_steps_failure || frame_time_limit_failure.is_some() {
            Some(SlowFrameError(max_steps_failure, frame_time_limit_failure))
        } else {
            None
        }
    }
}

// That's right, I'm clever.
impl Iterator for StepperRes {
    type Item = ();

    fn next(&mut self) -> Option<Self::Item> {
        let current_frame_delta = self.current_time_step();
        self.is_dirty = false;

        // First step in frame, initialize.
        if self.frame_steps == 0 || self.frame_start.is_none() {
            if let Some(last_frame) = self.frame_start {
                self.accumulator += last_frame.elapsed();
            }

            self.frame_steps = 0;
            self.frame_start = Some(Instant::now());
        }

        if let Some(slow_frame_error) = self.check_if_frame_slow() {
            // We've exhausted frame stepping limits and are running slow

            self.time_step
                .degraded_at_step(self.global_steps, slow_frame_error);

            // Signal end of stepping due to postponement.
            self.frame_steps = 0;
            None
        } else if self.accumulator >= current_frame_delta {
            // We may step the simulation once, drain the accumulator.
            self.frame_steps += 1;
            self.global_steps += 1;
            self.accumulator -= current_frame_delta;

            self.is_dirty = current_frame_delta != self.last_delta;
            self.last_delta = current_frame_delta;

            Some(())
        } else {
            // We've exhausted the accumulator.
            self.time_step.fast_at_step(self.global_steps);

            // Signal end of stepping.
            self.frame_steps = 0;
            None
        }
    }
}

impl Default for StepperRes {
    fn default() -> Self {
        Self::new(FixedTimeStep::default())
    }
}
