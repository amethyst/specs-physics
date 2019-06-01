use std::{f32::consts::PI, fmt};

use nalgebra::RealField;
use ncollide::shape::{Ball, Cuboid, ShapeHandle};
pub use ncollide::world::CollisionGroups;
pub use nphysics::material;
use nphysics::object::ColliderHandle;
use specs::{Component, DenseVecStorage, FlaggedStorage};

use crate::math::{Isometry3, Vector3};

use self::material::{BasicMaterial, MaterialHandle};

/// `Shape` serves as an abstraction over nphysics `ShapeHandle`s and makes it
/// easier to configure and define said `ShapeHandle`s for the user without
/// having to know the underlying nphysics API.
#[derive(Clone, Copy, Debug)]
pub enum Shape<N: RealField> {
    Circle(N),
    Rectangle(N, N, N),
}

impl<N: RealField> Shape<N> {
    /// Converts a `Shape` and its values into its corresponding `ShapeHandle`
    /// type. The `ShapeHandle` is used to define a `Collider` in the
    /// `PhysicsWorld`.
    fn handle(&self, margin: N) -> ShapeHandle<N> {
        match *self {
            Shape::Circle(radius) => ShapeHandle::new(Ball::new(radius)),
            Shape::Rectangle(width, height, depth) => ShapeHandle::new(Cuboid::new(Vector3::new(
                width / N::from_f32(2.0).unwrap() - margin,
                height / N::from_f32(2.0).unwrap() - margin,
                depth / N::from_f32(2.0).unwrap() - margin,
            ))),
        }
    }
}

/// The `PhysicsCollider` `Component` represents a `Collider` in the physics
/// world. A physics `Collider` is automatically created when this `Component`
/// is added to an `Entity`. Value changes are automatically synchronised with
/// the physic worlds `Collider`.
#[derive(Clone)]
pub struct PhysicsCollider<N: RealField> {
    pub(crate) handle: Option<ColliderHandle>,
    pub shape: Shape<N>,
    pub offset_from_parent: Isometry3<N>,
    pub density: N,
    pub material: MaterialHandle<N>,
    pub margin: N,
    pub collision_groups: CollisionGroups,
    pub linear_prediction: N,
    pub angular_prediction: N,
    pub sensor: bool,
}

impl<N: RealField> Component for PhysicsCollider<N> {
    type Storage = FlaggedStorage<Self, DenseVecStorage<Self>>;
}

impl<N: RealField> fmt::Debug for PhysicsCollider<N> {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(
            f,
            "PhysicsCollider {{ \
             handle: {:?}, \
             offset_from_parent: {:?}, \
             density: {}, \
             margin: {}, \
             collision_group: {:?}, \
             linear_prediction: {}, \
             angular_prediction: {}, \
             sensor: {} \
             }}",
            self.handle,
            self.offset_from_parent,
            self.density,
            self.margin,
            self.collision_groups,
            self.linear_prediction,
            self.angular_prediction,
            self.sensor,
        )?;
        Ok(())
    }
}

impl<N: RealField> PhysicsCollider<N> {
    /// Returns the `ShapeHandle` for `shape`, taking the `margin` into
    /// consideration.
    pub(crate) fn shape_handle(&self) -> ShapeHandle<N> {
        self.shape.handle(self.margin)
    }
}

/// The `PhysicsColliderBuilder` implements the builder pattern for
/// `PhysicsCollider`s and is the recommended way of instantiating and
/// customising new `PhysicsCollider` instances.
///
/// # Example
///
/// ```rust
/// use specs_physics::{
///     colliders::{
///         material::{BasicMaterial, MaterialHandle},
///         CollisionGroups,
///         Shape,
///     },
///     math::Isometry3,
///     PhysicsColliderBuilder,
/// };
///
/// let physics_collider = PhysicsColliderBuilder::from(Shape::Rectangle(10.0, 10.0, 1.0))
///     .offset_from_parent(Isometry3::identity())
///     .density(1.2)
///     .material(MaterialHandle::new(BasicMaterial::default()))
///     .margin(0.02)
///     .collision_groups(CollisionGroups::default())
///     .linear_prediction(0.001)
///     .angular_prediction(0.0)
///     .sensor(true)
///     .build();
/// ```
pub struct PhysicsColliderBuilder<N: RealField> {
    shape: Shape<N>,
    offset_from_parent: Isometry3<N>,
    density: N,
    material: MaterialHandle<N>,
    margin: N,
    collision_groups: CollisionGroups,
    linear_prediction: N,
    angular_prediction: N,
    sensor: bool,
}

impl<N: RealField> From<Shape<N>> for PhysicsColliderBuilder<N> {
    /// Creates a new `PhysicsColliderBuilder` from the given `Shape`. This
    //  also populates the `PhysicsCollider` with sane defaults.
    fn from(shape: Shape<N>) -> Self {
        Self {
            shape,
            offset_from_parent: Isometry3::identity(),
            density: N::from_f32(1.3).unwrap(),
            material: MaterialHandle::new(BasicMaterial::default()),
            margin: N::from_f32(0.2).unwrap(), // default was: 0.01
            collision_groups: CollisionGroups::default(),
            linear_prediction: N::from_f32(0.002).unwrap(),
            angular_prediction: N::from_f32(PI / 180.0 * 5.0).unwrap(),
            sensor: false,
        }
    }
}

impl<N: RealField> PhysicsColliderBuilder<N> {
    /// Sets the `offset_from_parent` value of the `PhysicsColliderBuilder`.
    pub fn offset_from_parent(mut self, offset_from_parent: Isometry3<N>) -> Self {
        self.offset_from_parent = offset_from_parent;
        self
    }

    /// Sets the `density` value of the `PhysicsColliderBuilder`.
    pub fn density(mut self, density: N) -> Self {
        self.density = density;
        self
    }

    /// Sets the `material` value of the `PhysicsColliderBuilder`.
    pub fn material(mut self, material: MaterialHandle<N>) -> Self {
        self.material = material;
        self
    }

    /// Sets the `margin` value of the `PhysicsColliderBuilder`.
    pub fn margin(mut self, margin: N) -> Self {
        self.margin = margin;
        self
    }

    /// Sets the `collision_groups` value of the `PhysicsColliderBuilder`.
    pub fn collision_groups(mut self, collision_groups: CollisionGroups) -> Self {
        self.collision_groups = collision_groups;
        self
    }

    /// Sets the `linear_prediction` value of the `PhysicsColliderBuilder`.
    pub fn linear_prediction(mut self, linear_prediction: N) -> Self {
        self.linear_prediction = linear_prediction;
        self
    }

    /// Sets the `angular_prediction` value of the `PhysicsColliderBuilder`.
    pub fn angular_prediction(mut self, angular_prediction: N) -> Self {
        self.angular_prediction = angular_prediction;
        self
    }

    /// Sets the `sensor` value of the `PhysicsColliderBuilder`.
    pub fn sensor(mut self, sensor: bool) -> Self {
        self.sensor = sensor;
        self
    }

    /// Builds the `PhysicsCollider` from the values set in the
    /// `PhysicsColliderBuilder` instance.
    pub fn build(self) -> PhysicsCollider<N> {
        PhysicsCollider {
            handle: None,
            shape: self.shape,
            offset_from_parent: self.offset_from_parent,
            density: self.density,
            material: self.material,
            margin: self.margin,
            collision_groups: self.collision_groups,
            linear_prediction: self.linear_prediction,
            angular_prediction: self.angular_prediction,
            sensor: self.sensor,
        }
    }
}
