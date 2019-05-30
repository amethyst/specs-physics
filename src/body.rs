use nalgebra::RealField;
use nphysics::object::BodyHandle;
pub use nphysics::object::BodyStatus;
use specs::{Component, DenseVecStorage, FlaggedStorage};

use crate::math::{Matrix3, Point3, Vector3};

/// An implementation of the `Position` trait is required for the
/// synchronisation of the position of Specs and nphysics objects.
///
/// Initially, it is used to position bodies in the nphysics `World`. Then after
/// progressing the `World` it is used to synchronise the updated positions back
/// towards Specs.
pub trait Position<N: RealField> {
    fn position(&self) -> (N, N, N);
    fn set_position(&mut self, x: N, y: N, z: N);
}

/// The `PhysicsBody` `Component` represents a `PhysicsWorld` `RigidBody` in
/// Specs and contains all the data required for the synchronisation between
/// both worlds.
#[derive(Clone, Copy, Debug)]
pub struct PhysicsBody<N: RealField> {
    pub(crate) handle: Option<BodyHandle>,
    pub gravity_enabled: bool,
    pub body_status: BodyStatus,
    pub velocity: Vector3<N>,
    pub angular_inertia: Matrix3<N>,
    pub mass: N,
    pub local_center_of_mass: Point3<N>,
}

impl<N: RealField> Component for PhysicsBody<N> {
    type Storage = FlaggedStorage<Self, DenseVecStorage<Self>>;
}

/// The `PhysicsBodyBuilder` implements the builder pattern for `PhysicsBody`s
/// and is the recommended way of instantiating and customising new
/// `PhysicsBody` instances.
///
/// # Example
///
/// ```rust
/// use specs_physics::{
///     body::BodyStatus,
///     math::{Matrix3, Point3, Vector3},
///     PhysicsBodyBuilder,
/// };
///
/// let physics_body = PhysicsBodyBuilder::from(BodyStatus::Dynamic)
///     .gravity_enabled(true)
///     .velocity(Vector3::new(1.0, 1.0, 1.0))
///     .angular_inertia(Matrix3::from_diagonal_element(3.0))
///     .mass(1.3)
///     .local_center_of_mass(Point3::new(0.0, 0.0, 0.0))
///     .build();
/// ```
pub struct PhysicsBodyBuilder<N: RealField> {
    gravity_enabled: bool,
    body_status: BodyStatus,
    velocity: Vector3<N>,
    angular_inertia: Matrix3<N>,
    mass: N,
    local_center_of_mass: Point3<N>,
}

impl<N: RealField> From<BodyStatus> for PhysicsBodyBuilder<N> {
    /// Creates a new `PhysicsBodyBuilder` from the given `BodyStatus`. This
    /// also populates the `PhysicsBody` with sane defaults.
    fn from(body_status: BodyStatus) -> Self {
        Self {
            gravity_enabled: false,
            body_status,
            velocity: Vector3::new(N::zero(), N::zero(), N::zero()),
            angular_inertia: Matrix3::zeros(),
            mass: N::from_f32(1.2).unwrap(),
            local_center_of_mass: Point3::new(N::zero(), N::zero(), N::zero()),
        }
    }
}

impl<N: RealField> PhysicsBodyBuilder<N> {
    /// Sets the `gravity_enabled` value of the `PhysicsBodyBuilder`.
    pub fn gravity_enabled(mut self, gravity_enabled: bool) -> Self {
        self.gravity_enabled = gravity_enabled;
        self
    }

    // Sets the `velocity` value of the `PhysicsBodyBuilder`.
    pub fn velocity(mut self, velocity: Vector3<N>) -> Self {
        self.velocity = velocity;
        self
    }

    /// Sets the `angular_inertia` value of the `PhysicsBodyBuilder`.
    pub fn angular_inertia(mut self, angular_inertia: Matrix3<N>) -> Self {
        self.angular_inertia = angular_inertia;
        self
    }

    /// Sets the `mass` value of the `PhysicsBodyBuilder`.
    pub fn mass(mut self, mass: N) -> Self {
        self.mass = mass;
        self
    }

    /// Sets the `local_center_of_mass` value of the `PhysicsBodyBuilder`.
    pub fn local_center_of_mass(mut self, local_center_of_mass: Point3<N>) -> Self {
        self.local_center_of_mass = local_center_of_mass;
        self
    }

    /// Builds the `PhysicsBody` from the values set in the `PhysicsBodyBuilder`
    /// instance.
    pub fn build(self) -> PhysicsBody<N> {
        PhysicsBody {
            handle: None,
            gravity_enabled: self.gravity_enabled,
            body_status: self.body_status,
            velocity: self.velocity,
            angular_inertia: self.angular_inertia,
            mass: self.mass,
            local_center_of_mass: self.local_center_of_mass,
        }
    }
}
