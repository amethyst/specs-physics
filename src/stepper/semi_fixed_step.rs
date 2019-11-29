use std::{
    fmt,
    time::{Duration, Instant},
};

use super::{SlowFrameError, TimeStep};

/// A variable stepping algorithm for single player games or gibs. WIP.
pub struct SemiFixedStep {
    minimum_time_running_slow: Option<Duration>,
    minimum_time_running_fast: Option<Duration>,
    steps: Vec<Duration>,
    active_step: usize,
    last_slow_step: Option<(u64, Instant)>,
    first_slow_step_in_last_series: Option<(u64, Instant)>,
}

impl TimeStep for SemiFixedStep {
    fn current_time_step(&self) -> Duration {
        self.steps[self.active_step]
    }

    fn fast_at_step(&mut self, global_step_number: u64) {
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

                let tuple = Some((global_step_number, Instant::now()));
                self.first_slow_step_in_last_series = tuple.clone();
                self.last_slow_step = tuple;
            }
        }
    }

    fn degraded_at_step(&mut self, global_step_number: u64, info: SlowFrameError) {
        let tuple = Some((global_step_number, Instant::now()));

        if self.first_slow_step_in_last_series.is_none() {
            warn!("Physics stepping is starting to fall behind. {}", info);

            self.first_slow_step_in_last_series = tuple.clone();
            self.last_slow_step = tuple;
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
                    info
                );

                self.last_slow_step = tuple;
            } else {
                warn!(
                    r#"Physics stepping has fallen beyond the threshold
and is lowering the step rate to the next level. {}"#,
                    info
                );

                self.first_slow_step_in_last_series = tuple.clone();
                self.last_slow_step = tuple;
                self.active_step += 1;
            }
        } else {
            // We've fallen behind, but not yet triggered a threshold to change step rate.
            warn!("Physics stepping has fallen behind. {}", info);

            self.last_slow_step = tuple;
        }
    }
}

/// When utilizing the semi-fixed timestep method, attempts to switch to the
/// step at `index` in the list of steps, optionally changing the state of
/// the qualifier for switching steps.
///
/// The following are the options for `qualifier_mod`:
/// - `None` - Do nothing. If there were timestamps for when slow frames were
///   occuring they are left alone.
/// - `SemiFixedQualifierState::NoPostponement` - Remove any timestamps for slow
///   frames, setting the qualifier state as though no frames have recently been
///   postponed. This is what is done internally when the
///   `minimum_time_running_fast` duration is hit, if it is set, and the `index`
///   is subsequently brought to zero.
/// - `SemiFixedQualifierState::StartPostponementNow(u64)` - Update timestamps
///   for slow frames to begin new slow period right now, stamped with the u64
///   frame number. This is what is done internally when the
///   `minimum_time_running_slow` duration is hit, if it is set.
impl SemiFixedStep {
    pub fn switch_to_step(
        &mut self,
        index: usize,
        qualifier_mod: Option<SemiFixedQualifierState>,
    ) -> Result<Duration, OutOfBoundsError> {
        if index < self.steps.len() {
            self.active_step = index;
            match qualifier_mod {
                Some(SemiFixedQualifierState::NoPostponement) => {
                    self.last_slow_step = None;
                    self.first_slow_step_in_last_series = None;
                }
                Some(SemiFixedQualifierState::StartPostponementNow(frame)) => {
                    let tuple = Some((frame, Instant::now()));
                    self.last_slow_step = tuple.clone();
                    self.first_slow_step_in_last_series = tuple;
                }
                _ => {}
            }
            Ok(self.steps[index])
        } else {
            Err(OutOfBoundsError)
        }
    }
}

/**
Effect to be applied to state of rate change qualifier via
[`SemiFixedStep::switch_to_step`].
*/
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

/// Error when indexing a step in [`SemiFixedStep`] which doesn't exist.
pub struct OutOfBoundsError;

impl fmt::Debug for OutOfBoundsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Selected index is out of step bounds.")
    }
}

impl fmt::Display for OutOfBoundsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "Selected index is out of step bounds.")
    }
}
