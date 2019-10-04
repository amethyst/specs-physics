/*!
# Stepper Data Module

This module contains types exclusively used for the specs-physics
fixed stepping implementation. If you choose to use some other way to
perform fixed stepping, such as using Amethyst's fixed dispatcher instead,
you can simply ignore this module.


*/
use std::{
    fmt,
    time::{Duration, Instant},
};

#[derive(Copy, Clone, Debug)]
pub enum TimeError {
    WrongStepType,
    OutOfSemiStepBounds,
}

impl fmt::Display for TimeError {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            formatter,
            "{}",
            match self {
                TimeError::WrongStepType =>
                    r#"Operation performed on wrong step type. 
Did you use a fixed timestep when you intended to use a semi-fixed timstep?"#,
                TimeError::OutOfSemiStepBounds => "Step index not in bounds of step list.",
            }
        )
    }
}

pub struct StepperRes {
    /// When set to Some, the Batch system will only execute that many steps in
    /// one dispatch/frame. Useful for preventing death spirals in the stepper
    /// when your application does not require synchronization. In such cases
    /// where synchronization of physics state is performed, you should
    /// instead watch the value of `BatchTimeRes::accumulator()` and
    /// invalidate local state after crossing a certain threshold.
    pub max_steps_per_frame: Option<u32>,

    /// When set to Some, the Batch system will postpone remaining steps to the
    /// next dispatch/frame after stepping for this time or longer. Useful for
    /// preventing death spirals in the stepper when your application does
    /// not require synchronization. In such cases where synchronization of
    /// physics state is performed, you should instead watch the value of
    /// `BatchTimeRes::accumulator()` and invalidate local state after
    /// crossing a certain threshold.
    pub frame_time_limit: Option<Duration>,

    // Tracks how far "behind" physics time we are
    accumulator: Duration,

    // Timestep interval state data
    time_step: TimeStep,

    // Whether the MechanicalWorld timestep should be updated
    time_step_dirty: bool,

    // Number of steps since start
    // Safety: Given a liberal timestep of 120hz, this would take 4.8b years to saturate.
    // A u32 on the other hand would take 414 days to saturate. War Boys voice MEDIOCRE!
    global_steps: u64,

    // Steps in current iteration
    // Safety: Let's pray that nobody has a frame that takes 414 days to render. WITNESS ME!
    frame_steps: u32,

    frame_start: Option<Instant>,
}

impl StepperRes {
    pub fn new_fixed(interval: u32) -> Self {
        Self::new_fixed_exact(Duration::from_secs(1) / interval, None, None)
    }

    pub fn new_fixed_exact(
        step_delta: Duration,
        max_steps_per_frame: Option<u32>,
        frame_time_limit: Option<Duration>,
    ) -> Self {
        Self {
            max_steps_per_frame,
            frame_time_limit,
            accumulator: Duration::default(),
            time_step: TimeStep::Fixed(step_delta),
            time_step_dirty: true,
            global_steps: 0,
            frame_steps: 0,
            frame_start: None,
        }
    }

    pub fn set_fixed_time_step(&mut self, interval: u32) {
        self.time_step = TimeStep::Fixed(Duration::from_secs(1) / interval);
        self.time_step_dirty = true;
    }

    /// Creates a new resource instance utilizing a semi-fixed timestep method.
    pub fn new_semi_fixed(
        steps: &[u32],
        minimum_time_running_slow: Option<Duration>,
        minimum_time_running_fast: Option<Duration>,
        max_steps_per_frame: Option<u32>,
        frame_time_limit: Option<Duration>,
    ) -> Self {
        Self::new_semi_fixed_exact(
            steps.iter().map(|x| Duration::from_secs(1) / *x).collect(),
            minimum_time_running_slow,
            minimum_time_running_fast,
            max_steps_per_frame,
            frame_time_limit,
        )
    }

    pub fn new_semi_fixed_exact(
        steps: Vec<Duration>,
        minimum_time_running_slow: Option<Duration>,
        minimum_time_running_fast: Option<Duration>,
        max_steps_per_frame: Option<u32>,
        frame_time_limit: Option<Duration>,
    ) -> Self {
        assert!(steps.len() > 0, "No steps provided for semi-fixed timer.");
        Self {
            max_steps_per_frame,
            frame_time_limit,
            accumulator: Duration::default(),
            time_step: TimeStep::SemiFixed(SemiFixedStep {
                minimum_time_running_slow,
                minimum_time_running_fast,
                steps,
                active_step: 0,
                first_slow_step_in_last_series: None,
                last_slow_step: None,
            }),
            time_step_dirty: true,
            global_steps: 0,
            frame_steps: 0,
            frame_start: None,
        }
    }

    /// When utilizing the semi-fixed timestep method, attempts to switch to the
    /// step at `index` in the list of steps, optionally changing the state of
    /// the qualifier for switching steps.
    ///
    /// The following are the options for `qualifier_mod`:
    /// - `None` - Do nothing. If there were timestamps for when slow frames
    ///   were occuring they are left alone.
    /// - `SemiFixedQualifierState::NoPostponement` - Remove any timestamps for
    ///   slow frames, setting the qualifier state as though no frames have
    ///   recently been postponed. This is what is done internally when the
    ///   `minimum_time_running_fast` duration is hit, if it is set, and the
    ///   `index` is subsequently brought to zero.
    /// - `SemiFixedQualifierState::StartPostponementNow(u64)` - Update
    ///   timestamps for slow frames to begin new slow period right now, stamped
    ///   with the u64 frame number. This is what is done internally when the
    ///   `minimum_time_running_slow` duration is hit, if it is set.
    pub fn semi_fixed_switch_to_step(
        &mut self,
        index: usize,
        qualifier_mod: Option<SemiFixedQualifierState>,
    ) -> Result<Duration, TimeError> {
        match &mut self.time_step {
            TimeStep::Fixed(_) => Err(TimeError::WrongStepType),
            TimeStep::SemiFixed(step_data) => {
                if index < step_data.steps.len() {
                    step_data.active_step = index;
                    self.time_step_dirty = true;
                    match qualifier_mod {
                        Some(SemiFixedQualifierState::NoPostponement) => {
                            step_data.last_slow_step = None;
                            step_data.first_slow_step_in_last_series = None;
                        }
                        Some(SemiFixedQualifierState::StartPostponementNow(frame)) => {
                            let tuple = Some((frame, Instant::now()));
                            step_data.last_slow_step = tuple.clone();
                            step_data.first_slow_step_in_last_series = tuple;
                        }
                        _ => {}
                    }
                    Ok(step_data.steps[index])
                } else {
                    Err(TimeError::OutOfSemiStepBounds)
                }
            }
        }
    }

    pub fn accumulator(&self) -> Duration {
        self.accumulator
    }

    pub fn time_step(&self) -> Duration {
        match &self.time_step {
            TimeStep::Fixed(duration) => *duration,
            TimeStep::SemiFixed(data) => data.steps[data.active_step],
        }
    }

    pub fn frame_steps(&self) -> u32 {
        self.frame_steps
    }

    pub fn global_steps(&self) -> u64 {
        self.global_steps
    }
}

// That's right, I'm clever.
impl Iterator for StepperRes {
    type Item = Step;

    fn next(&mut self) -> Option<Self::Item> {
        self.time_step_dirty = false;

        // Initialize frame.
        if self.frame_start.is_none() || self.frame_steps == 0 {
            self.frame_steps = 0;
            self.accumulator += match self.frame_start {
                Some(instant) => instant.elapsed(),
                None => self.time_step(),
            };
            self.frame_start = Some(Instant::now());
        }

        let current_frame_delta = self.time_step();
        if let Some(slow_frame_error) = SlowFrameError::check_for(
            self.frame_steps,
            self.frame_start
                .expect(
                    "Frame start stamp should've been initialized at the beginning of this frame.",
                )
                .elapsed(),
            self.max_steps_per_frame,
            self.frame_time_limit,
        ) {
            // We're running slow and the user's requested
            // we postpone steps to the next frame.
            if let TimeStep::SemiFixed(step_data) = &mut self.time_step {
                if step_data.degrade_from_being_slow(self.global_steps, slow_frame_error) {
                    self.time_step_dirty = true;
                }
            } else {
                warn!("Physics stepping is falling behind. {}", slow_frame_error);
            }

            // Signal end of stepping due to postponement.
            self.frame_steps = 0;
            None
        } else if self.accumulator >= current_frame_delta {
            // Drain the accumulator by the length of our timestep.
            self.frame_steps += 1;
            self.global_steps += 1;
            self.accumulator -= current_frame_delta;

            Some(Step {
                delta: current_frame_delta,
                step_dirty: self.time_step_dirty,
                accumulator_remaining: self.accumulator,
                global_step_number: self.global_steps,
                frame_step_number: self.frame_steps,
            })
        } else {
            if let TimeStep::SemiFixed(step_data) = &mut self.time_step {
                if step_data.upgrade_from_being_fast(self.global_steps) {
                    self.time_step_dirty = true;
                }
            }

            // Signal end of stepping due to exhaustion of accumulator.
            None
        }
    }
}

impl Default for StepperRes {
    fn default() -> Self {
        StepperRes::new_fixed(60)
    }
}

/// Affect to be applied to state of rate change qualifier via
/// `BatchTimeRes::semi_fixed_switch_to_step`.
#[derive(Debug, Clone, Copy)]
pub enum SemiFixedQualifierState {
    /// Remove any timestamps for slow frames, setting the qualifier state as
    /// though no frames have recently been postponed. This is what is done
    /// internally when the `minimum_time_running_fast` duration is hit, if it
    /// is set, and the `index` is subsequently brought to zero.
    NoPostponement,
    /// Update timestamps for slow frames to begin new slow period right now,
    /// stamped with the u64 frame number. This is what is done internally when
    /// the `minimum_time_running_slow` duration is hit, if it is set.
    StartPostponementNow(u64),
}

/// Data associated with a single step performed by `<BatchTimeRes as
/// Iterator>::next`.
#[derive(Debug, Default, Clone)]
pub struct Step {
    delta: Duration,
    step_dirty: bool,
    accumulator_remaining: Duration,
    global_step_number: u64,
    frame_step_number: u32,
}

impl Step {
    /// Delta time of the current physics step.
    /// If you're using a fixed step, this will always be the same.
    pub fn delta(&self) -> Duration {
        self.delta
    }

    /// Time left in the delta accumulator for the stepper.
    pub fn accumulator_remaining(&self) -> Duration {
        self.accumulator_remaining
    }

    /// Whether the delta of the step has been modified.
    pub fn step_dirty(&self) -> bool {
        self.step_dirty
    }

    /// How many physics steps have occured globally
    pub fn global_step_number(&self) -> u64 {
        self.global_step_number
    }

    /// How many physics steps have occured in this frame.
    pub fn frame_step_number(&self) -> u32 {
        self.frame_step_number
    }

    pub(crate) fn update(&mut self, other: Self) {
        self.delta = other.delta;
        self.step_dirty = other.step_dirty;
        self.accumulator_remaining = other.accumulator_remaining;
        self.global_step_number = other.global_step_number;
        self.frame_step_number = other.frame_step_number;
    }
}

enum TimeStep {
    Fixed(Duration),
    SemiFixed(SemiFixedStep),
}

struct SemiFixedStep {
    minimum_time_running_slow: Option<Duration>,
    minimum_time_running_fast: Option<Duration>,
    steps: Vec<Duration>,
    active_step: usize,
    last_slow_step: Option<(u64, Instant)>,
    first_slow_step_in_last_series: Option<(u64, Instant)>,
}

impl SemiFixedStep {
    fn degrade_from_being_slow(
        &mut self,
        step_number: u64,
        slow_frame_error: SlowFrameError,
    ) -> bool {
        let tuple = Some((step_number, Instant::now()));

        if self.first_slow_step_in_last_series.is_none() {
            warn!(
                "Physics stepping is starting to fall behind. {}",
                slow_frame_error
            );

            self.first_slow_step_in_last_series = tuple.clone();
            self.last_slow_step = tuple;

            false
        } else if self.last_slow_step.is_some()
            && self.minimum_time_running_slow.map_or(false, |minimum| {
                self.last_slow_step.unwrap().1.elapsed() > minimum
            })
        {
            if self.active_step >= self.steps.len() - 1 {
                error!(
                    r#"Physics stepping has fallen beyond the threshold
for setting the step to a lower level. However,
there are no lower level step rates to fall back to. {}"#,
                    slow_frame_error
                );

                self.last_slow_step = tuple;

                false
            } else {
                warn!(
                    r#"Physics stepping has fallen beyond the threshold
and is lowering the step rate to the next level. {}"#,
                    slow_frame_error
                );

                self.first_slow_step_in_last_series = tuple.clone();
                self.last_slow_step = tuple;
                self.active_step += 1;

                true
            }
        } else {
            // We've fallen behind, but not yet triggered a threshold to change step rate.
            warn!("Physics stepping has fallen behind. {}", slow_frame_error);

            self.last_slow_step = tuple;

            false
        }
    }

    fn upgrade_from_being_fast(&mut self, step_number: u64) -> bool {
        if self.minimum_time_running_fast.is_some()
            && self.active_step > 0
            && (self.last_slow_step.is_none()
                || self.last_slow_step.unwrap().1.elapsed()
                    > self.minimum_time_running_fast.unwrap())
        {
            self.active_step -= 1;

            if self.active_step == 0 {
                info!("Physics stepping has resumed base stepping rate.");

                self.last_slow_step = None;
                self.first_slow_step_in_last_series = None;
            } else {
                info!(
                    "Physics stepping has upgraded to the stepping rate index {}.",
                    self.active_step
                );

                let tuple = Some((step_number, Instant::now()));
                self.first_slow_step_in_last_series = tuple.clone();
                self.last_slow_step = tuple;
            }

            true
        } else {
            false
        }
    }
}

struct SlowFrameError(bool, Option<Duration>);

impl SlowFrameError {
    fn check_for(
        frame_steps: u32,
        frame_duration: Duration,
        max_steps_per_frame: Option<u32>,
        frame_time_limit: Option<Duration>,
    ) -> Option<Self> {
        // Check if we've ran past the frame step limit
        let max_steps_failure = max_steps_per_frame.map_or(false, |max| frame_steps >= max);

        // Check if we've ran past the frame time limit, and calculate how much by
        let frame_time_limit_failure = frame_time_limit.and_then(|limit| {
            if frame_duration > limit {
                Some(frame_duration - limit)
            } else {
                None
            }
        });

        if max_steps_failure || frame_time_limit_failure.is_some() {
            Some(SlowFrameError(max_steps_failure, frame_time_limit_failure))
        } else {
            None
        }
    }
}

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
