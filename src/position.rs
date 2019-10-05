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
pub trait Position<N: RealField>:
    Component<Storage = FlaggedStorage<Self, DenseVecStorage<Self>>> + Send + Sync
{
    fn isometry(&self) -> &Isometry<N>;
    fn isometry_mut(&mut self) -> &mut Isometry<N>;

    /// Helper function to extract the location of this `Position`. Using
    /// `Position::isometry()` is preferable, but can be harder to work
    /// with. The translation of this `Position` can be set using `Position:
    /// :isometry_mut()`.
    fn translation(&self) -> Point<N> {
        self.isometry().translation.vector.into()
    }

    /// Helper function to extract the rotation of this `Position`. Using
    /// `Position::isometry()` is preferable, but can be harder to work
    /// with. The rotation of this `Position` can be set using `Position::
    /// isometry_mut()`. This is only available when the `physics2d` feature is
    /// enabled.
    #[cfg(feature = "dim2")]
    fn angle(&self) -> N {
        self.isometry().rotation.angle()
    }
}

#[cfg(all(feature = "amethyst", feature = "dim3"))]
impl Position<f32> for amethyst_core::Transform {
    fn isometry(&self) -> &Isometry<f32> {
        self.isometry()
    }

    fn isometry_mut(&mut self) -> &mut Isometry<f32> {
        self.isometry_mut()
    }
}

// Two dimensional Amethyst code from
// https://github.com/olvyko/specs-physics/blob/master/src/pose.rs
// TODO: Write tests for this code.
#[cfg(all(feature = "amethyst", feature = "dim2"))]
impl Position<f32> for amethyst_core::Transform {
    fn isometry(&self) -> &Isometry<f32> {
        let iso = self.isometry();
        Isometry::from_parts(
            Translation2::new(iso.translation.x, iso.translation.y),
            UnitComplex::new(iso.rotation.euler_angles().2),
        )
    }

    fn isometry_mut(&mut self) -> &mut Isometry<f32> {
        let euler = self.rotation().euler_angles();
        self.set_rotation_euler(euler.0, euler.1, isometry.rotation.angle());
        self.set_translation_x(isometry.translation.x);
        self.set_translation_y(isometry.translation.y);
        self
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Hash, Shrinkwrap)]
#[shrinkwrap(mutable)]
pub struct SimplePosition<N: RealField>(pub Isometry<N>);

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
