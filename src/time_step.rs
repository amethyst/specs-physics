use std::{
    cmp::Ordering,
    time::{Duration, Instant},
};

/// The type of time step to use for the physics simulation.
pub enum TimeStep {
    /// Physics will always use the given timestep.
    Fixed(f32),
    /// Physics use one of the given timesteps, changing when physics are falling behind.
    SemiFixed(TimeStepConstraint),
}

impl Default for TimeStep {
    fn default() -> Self {
        TimeStep::Fixed(1. / 120.)
    }
}

/// Error when trying to change the actual timestep for a semi-fixed timestep.
#[derive(Debug)]
pub enum TimeStepChangeError {
    /// No smaller timestep available.
    MinimumTimestepReached,
    /// No larger timestep available.
    MaximumTimestepReached,
}

impl std::fmt::Display for TimeStepChangeError {
    fn fmt(&self, f: &mut std::fmt::Formatter) -> std::fmt::Result {
        match self {
            TimeStepChangeError::MaximumTimestepReached => {
                write!(f, "No larger timestep available!")
            }
            TimeStepChangeError::MinimumTimestepReached => {
                write!(f, "No smaller timestep available!")
            }
        }
    }
}

/// Constraints for a semi-fixed timestep.
pub struct TimeStepConstraint {
    /// Vector of possible timesteps to use.
    time_steps: Vec<f32>,
    /// Index of the currently used timestep.
    current_index: usize,
    /// Fraction of frame time physics are allowed to take.
    max_physics_time_fraction: f32,
    /// Minimum time the simulation has to be running slow before the timestep is changed.
    minimum_time_running_slow: Duration,
    /// Minimum time the simulation has to be running fast before the timestep is changed.
    minimum_time_running_fast: Duration,
    /// Time when the simulation started running slow.
    running_slow_since: Option<Instant>,
    /// Time when the simulation started running fast.
    running_fast_since: Option<Instant>,
}

impl TimeStepConstraint {
    /// Creates a new `TimeStepConstraint` from the specified timesteps to use, the maximum physics
    /// time fraction and the minimum times before changing timestep.
    ///
    /// If physics take more than the specified fraction of simulated time, the timestep will be
    /// increased. If physics with a smaller timestep would still take less than the specified
    /// fraction of simulated time, the timestep will be decreased. Timesteps will only be changed
    /// if physics are running fast/slow for the specified durations.
    ///
    /// # Examples
    ///
    /// ```
    /// let game_data = GameDataBuilder::default()
    ///     .with_bundle(
    ///         PhysicsBundle::new()
    ///             .with_timestep(TimeStep::SemiFixed(TimeStepConstraint::new(
    ///                 vec![1. / 240., 1. / 120., 1. / 60.],
    ///                 0.4,
    ///                 Duration::from_millis(50),
    ///                 Duration::from_millis(500),
    ///             )))
    ///     )?
    ///     ...
    /// );
    /// ```
    /// Here physics will start out with a timestep of 1/240 seconds. If physics take more than 40% of
    /// this timestep to simulate, it's running slowly. If it's running slowly for 50ms in a row, the
    /// timestep will be increased to 1/120 seconds. If physics are running fast for 500ms in a row, the
    /// timestep will be decreased again.
    ///
    /// # Panics
    ///
    /// This constructor will panic if no timesteps are given or if any negative timesteps are specified.
    pub fn new(
        time_steps: impl Into<Vec<f32>>,
        max_physics_time_fraction: f32,
        minimum_time_running_slow: Duration,
        minimum_time_running_fast: Duration,
    ) -> Self {
        let mut time_steps = time_steps.into();
        assert!(
            !time_steps.is_empty(),
            "No timesteps given in TimeStepConstraint"
        );
        time_steps.sort_by(|a, b| a.partial_cmp(b).unwrap_or(Ordering::Equal));
        time_steps.dedup();
        assert!(time_steps[0] > 0., "Negative timesteps are not allowed");

        Self {
            time_steps,
            current_index: 0,
            max_physics_time_fraction,
            minimum_time_running_slow,
            minimum_time_running_fast,
            running_slow_since: None,
            running_fast_since: None,
        }
    }

    /// Increase the timestep. This corresponds to fewer updates per second.
    ///
    /// Shouldn't be called from outside the `PhysicsStepperSystem`, otherwise bad things may happen.
    pub fn increase_timestep(&mut self) -> Result<f32, TimeStepChangeError> {
        if self.current_index >= self.time_steps.len() - 1 {
            return Err(TimeStepChangeError::MaximumTimestepReached);
        }
        self.current_index += 1;
        self.running_slow_since = None;
        self.running_fast_since = None;
        Ok(self.current_timestep())
    }

    /// Decrease the timestep. This corresponds to more updates per second.
    ///
    /// Shouldn't be called from outside the `PhysicsStepperSystem`, otherwise bad things may happen.
    pub fn decrease_timestep(&mut self) -> Result<f32, TimeStepChangeError> {
        if self.current_index == 0 {
            return Err(TimeStepChangeError::MinimumTimestepReached);
        }
        self.current_index -= 1;
        self.running_slow_since = None;
        self.running_fast_since = None;
        Ok(self.current_timestep())
    }

    /// Get the currently used timestep.
    pub fn current_timestep(&self) -> f32 {
        self.time_steps[self.current_index]
    }

    /// Get next smaller timestep.
    pub fn smaller_timestep(&self) -> Option<f32> {
        if self.current_index == 0 {
            None
        } else {
            Some(self.time_steps[self.current_index - 1])
        }
    }

    /// Get the fraction of frame time physics are allowed to take.
    pub fn max_physics_time_fraction(&self) -> f32 {
        self.max_physics_time_fraction
    }

    /// Set whether physics are currently running slow or not. Intended to be called every frame as changing
    /// values only happens when `is_running_slow` changes between calls.
    ///
    /// Shouldn't be called from outside the `PhysicsStepperSystem`, otherwise bad things may happen.
    pub fn set_running_slow(&mut self, is_running_slow: bool) {
        self.running_slow_since = match (self.running_slow_since, is_running_slow) {
            (None, true) => {
                warn!("Physics seem to be running slow! Timestep will be changed if we keep running slow.");
                Some(Instant::now())
            }
            (Some(_), false) => {
                debug!("Physics aren't running slow anymore.");
                None
            }
            (_, _) => self.running_slow_since,
        };
    }

    /// Set whether physics are currently running fast or not. Intended to be called every frame as changing
    /// values only happens when `is_running_fast` changes between calls.
    ///
    /// Shouldn't be called from outside the `PhysicsStepperSystem`, otherwise bad things may happen.
    pub fn set_running_fast(&mut self, is_running_fast: bool) {
        self.running_fast_since = match (self.running_fast_since, is_running_fast) {
            (None, true) => {
                debug!("Physics seem to be running fast. Timestep will be changed if we keep running fast.");
                Some(Instant::now())
            }
            (Some(_), false) => {
                debug!("Physics aren't running fast anymore.");
                None
            }
            (_, _) => self.running_fast_since,
        };
    }

    /// Get whether physics have been running slow for the specified minimum time and thus the timestep should
    /// be increased.
    pub fn should_increase_timestep(&self) -> bool {
        match self.running_slow_since {
            None => false,
            Some(time) => time.elapsed() > self.minimum_time_running_slow,
        }
    }

    /// Get whether physics have been running fast for the specified minimum time and thus the timestep should
    /// be decreased.
    pub fn should_decrease_timestep(&self) -> bool {
        match self.running_fast_since {
            None => false,
            Some(time) => time.elapsed() > self.minimum_time_running_fast,
        }
    }
}
