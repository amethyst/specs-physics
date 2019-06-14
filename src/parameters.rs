//! # Parameters module
//! Resources for modifying the various simulation parameters of the
//! nphysics World.

use crate::math::{self as na, RealField, Scalar, Vector3};
use nphysics::solver::IntegrationParameters;
use std::ops::{Deref, DerefMut};

/// The `TimeStep` is used to set the timestep of the nphysics integration, see
/// `nphysics::world::World::set_timestep(..)`.
///
/// Warning: Do **NOT** change this value every frame, doing so will destabilize
/// the simulation. The stepping system itself should be called in a "fixed"
/// update which maintains a running delta. See [this blog post][gaffer]
/// by Glenn Fiedler to learn more about timesteps.
///
/// [gaffer]: https://gafferongames.com/game-physics/fix-your-timestep/%22
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct TimeStep<N: RealField>(pub N);

impl<N: RealField> Deref for TimeStep<N> {
    type Target = N;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<N: RealField> DerefMut for TimeStep<N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<N: RealField> Default for TimeStep<N> {
    fn default() -> Self {
        Self(na::convert(1.0 / 60.0))
    }
}

/// `Gravity` is a newtype for `Vector3`. It represents a constant
/// acceleration affecting all physical objects in the scene.
#[derive(Debug, PartialEq)]
pub struct Gravity<N: RealField + Scalar>(pub Vector3<N>);

impl<N: RealField + Scalar> Deref for Gravity<N> {
    type Target = Vector3<N>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<N: RealField + Scalar> DerefMut for Gravity<N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl<N: RealField + Scalar> Default for Gravity<N> {
    fn default() -> Self {
        Self(Vector3::<N>::zeros())
    }
}

/// Enables reporting of `nphysics::counters`,
/// which can be read via `Physics::performance_counters`
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct PhysicsProfilingEnabled(pub bool);

impl Deref for PhysicsProfilingEnabled {
    type Target = bool;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for PhysicsProfilingEnabled {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

impl Default for PhysicsProfilingEnabled {
    fn default() -> Self {
        Self(false)
    }
}

/// Essentially identical to the nphysics IntegrationParameters struct except
/// without the t and dt fields. Manages the details of physics integration.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PhysicsIntegrationParameters<N: RealField> {
    /// The `[0,1]` proportion of the positional error to be corrected at each
    /// time step.
    ///
    /// default: `0.2`
    pub error_reduction_parameter: N,

    /// Each cached impulse are multiplied by this `[0, 1]` coefficient when
    /// they are re-used to initialize the solver.
    ///
    /// default: `1.0`
    pub warmstart_coefficient: N,

    /// Contacts at points where the involved bodies have a relative velocity
    /// smaller than this threshold won't be affected by the restitution force.
    ///
    /// default: `1.0`
    pub restitution_velocity_threshold: N,

    /// Ammount of penetration the engine won't attempt to correct.
    ///
    /// default: `0.001m`
    pub allowed_linear_error: N,

    /// Ammount of angular drift of joint limits the engine won't attempt to
    /// correct.
    ///
    /// default: `0.001rad`
    pub allowed_angular_error: N,

    /// Maximum linear correction during one step of the non-linear position
    /// solver.
    ///
    /// default: `100.0`
    pub max_linear_correction: N,

    /// Maximum angular correction during one step of the non-linear position
    /// solver.
    ///
    /// default: `0.2`
    pub max_angular_correction: N,

    /// Maximum nonlinera SOR-prox scaling parameter when the constraint
    /// correction direction is close to the kernel of the involved multibody's
    /// jacobian.
    ///
    /// default: `0.2`
    pub max_stabilization_multiplier: N,

    /// Maximum number of iterations performed by the velocity constraints
    /// solver.
    ///
    /// default: `8`
    pub max_velocity_iterations: usize,

    /// Maximum number of iterations performed by the position-based constraints
    /// solver.
    ///
    /// default: `3`
    pub max_position_iterations: usize,
}

impl<N: RealField> PhysicsIntegrationParameters<N> {
    pub(crate) fn apply(&self, to: &mut IntegrationParameters<N>) {
        to.erp = self.error_reduction_parameter;
        to.warmstart_coeff = self.warmstart_coefficient;
        to.restitution_velocity_threshold = self.restitution_velocity_threshold;
        to.allowed_linear_error = self.allowed_linear_error;
        to.allowed_angular_error = self.allowed_angular_error;
        to.max_linear_correction = self.max_linear_correction;
        to.max_angular_correction = self.max_angular_correction;
        to.max_stabilization_multiplier = self.max_stabilization_multiplier;
        to.max_velocity_iterations = self.max_velocity_iterations;
        to.max_position_iterations = self.max_position_iterations;
    }
}

impl<N: RealField> PartialEq<IntegrationParameters<N>> for PhysicsIntegrationParameters<N> {
    fn eq(&self, other: &IntegrationParameters<N>) -> bool {
        self.error_reduction_parameter == other.erp
            && self.warmstart_coefficient == other.warmstart_coeff
            && self.restitution_velocity_threshold == other.restitution_velocity_threshold
            && self.allowed_linear_error == other.allowed_linear_error
            && self.allowed_angular_error == other.allowed_angular_error
            && self.max_linear_correction == other.max_linear_correction
            && self.max_angular_correction == other.max_angular_correction
            && self.max_stabilization_multiplier == other.max_stabilization_multiplier
            && self.max_velocity_iterations == other.max_velocity_iterations
            && self.max_position_iterations == other.max_position_iterations
    }
}

impl<N: RealField> Default for PhysicsIntegrationParameters<N> {
    fn default() -> Self {
        PhysicsIntegrationParameters {
            error_reduction_parameter: na::convert(0.2),
            warmstart_coefficient: na::convert(1.0),
            restitution_velocity_threshold: na::convert(1.0),
            allowed_linear_error: na::convert(0.001),
            allowed_angular_error: na::convert(0.001),
            max_linear_correction: na::convert(100.0),
            max_angular_correction: na::convert(0.2),
            max_stabilization_multiplier: na::convert(0.2),
            max_velocity_iterations: 8,
            max_position_iterations: 3,
        }
    }
}
