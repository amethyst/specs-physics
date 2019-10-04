use specs::{Component, DenseVecStorage, FlaggedStorage};
use std::ops::{Deref, DerefMut};
use nalgebra::RealField;
use nalgebra::{Point, Isometry};

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

    /// Helper function to extract the location of this `Position`. Using `Position::isometry()` is
    /// preferable, but can be harder to work with. The translation of this `Position` can be set
    /// using `Position::isometry_mut()`.
    fn translation(&self) -> Point<N> {
        self.isometry().translation.vector.into()
    }

    /// Helper function to extract the rotation of this `Position`. Using `Position::isometry()` is
    /// preferable, but can be harder to work with. The rotation of this `Position` can be set
    /// using `Position::isometry_mut()`. This is only available when the `physics2d` feature is
    /// enabled.
    #[cfg(feature = "physics2d")]
    fn angle(&self) -> N {
        self.isometry().rotation.angle()
    }
}

#[cfg(feature = "amethyst")]
impl Position<f32> for amethyst_core::Transform {
    fn isometry(&self) -> &Isometry<f32> {
        self.isometry()
    }

    fn isometry_mut(&mut self) -> &mut Isometry<f32> {
        self.isometry_mut()
    }
}

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

impl<N: RealField> Deref for SimplePosition<N> {
    type Target = Isometry<N>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<N: RealField> DerefMut for SimplePosition<N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}
