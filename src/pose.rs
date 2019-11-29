use crate::{
    nalgebra::{
        base::{
            dimension::{DimNameSum, U1},
            MatrixN,
        },
        RealField,
    },
    nphysics::math::{Dim, Isometry, Point},
};

use specs::{Component, DenseVecStorage};

/**
Implement this for the Component type you want to sync transformation data to from the simulation.

If an entity shares this Component and a [`BodyPartHandle`], [`PhysicsPoseSystem`] will call `sync`
with the transform data of the Body part pointed at by the handle. Otherwise, if this Component is
on an entity with a [`BodyComponent`], the first part on that Body (for non-multipart bodies, the
Body itself) will be passed to `sync`. [`SimplePosition`] is provided as a simpl utility type which
implements this trait.
*/
pub trait Pose<N: RealField>: Component + Send + Sync {
    fn sync(&mut self, pose: &Isometry<N>);
}

// TODO: 64 bit implementation for amethyst
#[cfg(all(feature = "amethyst", feature = "dim3"))]
impl Pose<f32> for amethyst::core::Transform {
    fn sync(&mut self, pose: &Isometry<f32>) {
        *self.isometry_mut() = *pose;
    }
}

#[cfg(all(feature = "amethyst", feature = "dim2"))]
impl Pose<f32> for amethyst::core::Transform {
    fn sync(&mut self, pose: &Isometry<N>) {
        let euler = self.rotation().euler_angles();
        self.set_rotation_euler(euler.0, euler.1, pose.rotation.angle());
        self.set_translation_x(pose.translation.x);
        self.set_translation_y(pose.translation.y);
    }
}

/// A utility type you may use for synchronizing poses from the simulation.
///
/// Like any `Pose` data, you should not modify this component
/// directly when it is being synchronized to,
/// but should instead modify the `Body` relating to this component.
#[derive(Copy, Clone, Debug, PartialEq, Hash)]
pub struct SimplePosition<N: RealField>(pub Isometry<N>);

impl<N: RealField> SimplePosition<N> {
    /// Sugar for retreiving the transformation matrix of the position.
    pub fn matrix(&self) -> MatrixN<N, DimNameSum<Dim, U1>> {
        self.0.to_homogeneous()
    }

    /// Sugar for retreiving the translational component of the position.
    pub fn translation(&self) -> Point<N> {
        self.0.translation.vector.into()
    }

    /// In 2D, the rotation angle within the range `[-pi, pi]`.
    /// In 3D, the rotation angle of the unit quaternion in the range `[0, pi]`.
    pub fn angle(&self) -> N {
        self.0.rotation.angle()
    }
}

impl<N: RealField> Pose<N> for SimplePosition<N> {
    fn sync(&mut self, pose: &Isometry<N>) {
        self.0 = *pose;
    }
}

impl<N: RealField> Component for SimplePosition<N> {
    type Storage = DenseVecStorage<Self>;
}

impl<N: RealField> Default for SimplePosition<N> {
    fn default() -> Self {
        Self(Isometry::identity())
    }
}
