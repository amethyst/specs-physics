use std::{f32::consts::PI, fmt, ops::Deref};

use specs::{Component, DenseVecStorage, FlaggedStorage};

use crate::{
    nalgebra::{DMatrix, Isometry3, Point2, Point3, RealField, Unit, Vector3},
    ncollide::{
        shape::{
            Ball,
            Capsule,
            Compound,
            ConvexHull,
            Cuboid,
            HeightField,
            Plane,
            Polyline,
            Segment,
            ShapeHandle,
            TriMesh,
            Triangle,
        },
        world::CollisionGroups,
    },
    nphysics::{
        material::{BasicMaterial, MaterialHandle},
        object::ColliderHandle,
    },
};

pub type MeshData<N> = (Vec<Point3<N>>, Vec<Point3<usize>>, Option<Vec<Point2<N>>>);

pub trait IntoMesh: objekt::Clone + Send + Sync {
    type N: RealField;

    fn points(&self) -> MeshData<Self::N>;
}

impl<'clone, N: RealField> IntoMesh for Box<(dyn IntoMesh<N = N> + 'clone)> {
    type N = N;

    fn points(&self) -> MeshData<Self::N> {
        self.deref().points()
    }
}

impl<'clone, N: RealField> Clone for Box<dyn IntoMesh<N = N> + 'clone> {
    fn clone(&self) -> Self {
        objekt::clone_box(&*self)
    }
}

/// `Shape` serves as an abstraction over nphysics `ShapeHandle`s and makes it
/// easier to configure and define said `ShapeHandle`s for the user without
/// having to know the underlying nphysics API.
#[derive(Clone)]
pub enum Shape<N: RealField> {
    Ball {
        radius: N,
    },
    Capsule {
        half_height: N,
        radius: N,
    },
    Compound {
        parts: Vec<(Isometry3<N>, Shape<N>)>,
    },
    ConvexHull {
        points: Vec<Point3<N>>,
    },
    Cuboid {
        half_extents: Vector3<N>,
    },
    HeightField {
        heights: DMatrix<N>,
        scale: Vector3<N>,
    },
    Plane {
        normal: Unit<Vector3<N>>,
    },
    Polyline {
        points: Vec<Point3<N>>,
        indices: Option<Vec<Point2<usize>>>,
    },
    Segment {
        a: Point3<N>,
        b: Point3<N>,
    },
    TriMesh {
        handle: Box<dyn IntoMesh<N = N>>,
    },
    Triangle {
        a: Point3<N>,
        b: Point3<N>,
        c: Point3<N>,
    },
}

impl<N: RealField> Shape<N> {
    /// Converts a `Shape` and its values into its corresponding `ShapeHandle`
    /// type. The `ShapeHandle` is used to define a `Collider` in the
    /// `PhysicsWorld`.
    fn handle(&self) -> ShapeHandle<N> {
        match self {
            Shape::Ball { radius } => ShapeHandle::new(Ball::<N>::new(*radius)),
            Shape::Capsule {
                half_height,
                radius,
            } => ShapeHandle::new(Capsule::new(*half_height, *radius)),
            Shape::Compound { parts } => ShapeHandle::new(Compound::new(
                parts.iter().map(|part| (part.0, part.1.handle())).collect(),
            )),
            Shape::ConvexHull { points } => ShapeHandle::new(
                ConvexHull::try_from_points(&points)
                    .expect("Failed to generate Convex Hull from points."),
            ),
            Shape::Cuboid { half_extents } => ShapeHandle::new(Cuboid::new(*half_extents)),
            Shape::HeightField { heights, scale } => {
                ShapeHandle::new(HeightField::new(heights.clone(), *scale))
            }
            Shape::Plane { normal } => ShapeHandle::new(Plane::new(*normal)),
            Shape::Polyline { points, indices } => {
                ShapeHandle::new(Polyline::new(points.clone(), indices.clone()))
            }
            Shape::Segment { a, b } => ShapeHandle::new(Segment::new(*a, *b)),
            Shape::TriMesh { handle } => {
                let data = handle.points();
                ShapeHandle::new(TriMesh::new(data.0, data.1, data.2))
            }
            Shape::Triangle { a, b, c } => ShapeHandle::new(Triangle::new(*a, *b, *c)),
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
        self.shape.handle()
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
///     colliders::Shape,
///     nalgebra::{Isometry3, Vector3},
///     ncollide::world::CollisionGroups,
///     nphysics::material::{BasicMaterial, MaterialHandle},
///     PhysicsColliderBuilder,
/// };
///
/// let physics_collider = PhysicsColliderBuilder::from(Shape::Cuboid{ half_extents: Vector3::new(10.0, 10.0, 1.0) })
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
