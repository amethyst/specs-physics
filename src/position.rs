use crate::{
    nalgebra::RealField,
    nphysics::math::{Isometry, Point},
};

use specs::{Component, DenseVecStorage, FlaggedStorage};

/// An implementation of the `Position` trait is required for the
/// synchronisation of the position of Specs and nphysics objects.
///
/// Initially, it is used to position bodies in the nphysics `World`. Then after
/// progressing the `World` it is used to synchronise the updated positions back
/// towards Specs.
pub trait Pose<N: RealField>: Component + Send + Sync {
    fn sync(&mut self, pose: &Isometry<N>);
}

#[cfg(all(feature = "amethyst", feature = "dim3"))]
impl Pose<f32> for amethyst::core::Transform {
    fn sync(&mut self, pose: &Isometry<N>) {
        *self.isometry_mut() = pose;
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

// TODO: 64 bit implementation for amethyst

#[derive(Copy, Clone, Debug, PartialEq, Hash, Shrinkwrap)]
#[shrinkwrap(mutable)]
pub struct SimplePosition<N: RealField>(pub Isometry<N>);

impl<N: RealField> SimplePosition<N> {
    /// Helper function to extract the location of this `Position`. Using
    /// `Position::isometry()` is preferable, but can be harder to work
    /// with. The translation of this `Position` can be set using `Position:
    /// :isometry_mut()`.
    fn translation(&self) -> Point<N> {
        self.0.translation.vector.into()
    }

    /// Helper function to extract the rotation of this `Position`. Using
    /// `Position::isometry()` is preferable, but can be harder to work
    /// with. The rotation of this `Position` can be set using `Position::
    /// isometry_mut()`. This is only available when the `physics2d` feature is
    /// enabled.
    #[cfg(feature = "dim2")]
    fn angle(&self) -> N {
        self.0.rotation.angle()
    }
}

impl<N: RealField> Position<N> for SimplePosition<N> {
    fn isometry(&self) -> &Isometry<N> {
        &self.0
    }

    fn isometry_mut(&mut self) -> &mut Isometry<N> {
        &mut self.0
    }
}

impl<N: RealField> Component for SimplePosition<N> {
    type Storage = FlaggedStorage<Self, DenseVecStorage<Self>>;
}

impl<N: RealField> Default for SimplePosition<N> {
    fn default() -> Self {
        Self(Isometry::identity())
    }
}
