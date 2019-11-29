//! Component data of the bodies themselves

use crate::{
    nalgebra::RealField,
    nphysics::object::{Body, Ground, Multibody, RigidBody},
};

use specs::{Component, DenseVecStorage, Entity, FlaggedStorage};

/**
Component designating component as a `usize` index body part of `Entity`.

Attaching this component to your entity with a [`Pose`] component will make the syncing system
synchronize the isometry of the `usize` indexed part on the [`BodyComponent`] of `Entity`. This
relationship is one-way, however. If you'd like to update the position of a Body, **do so via that
Body's `BodyComponent`, and not the component used for `Pose` synchronization**.

[`Pose`]: ../trait.Pose.html
*/
#[derive(Copy, Clone, Debug)]
pub struct BodyPartHandle(pub Entity, pub usize);

impl Component for BodyPartHandle {
    type Storage = DenseVecStorage<Self>;
}

/**
The component type of all physics bodies.

Attaching this component to your entity with a [`Pose`] component will make the syncing system
synchronize the isometry of the first part in that Body to your `Pose` (or simply the position of
the body if it is a single part body). This relationship is one-way, however. If you'd like to
update the position of a Body, **do so via this component, and not from the component used for
`Pose` synchronization**.

If you'd like to synchronize individual parts of a body to a [`Pose`], you should not attach a
`Pose` to the entity with this Component, and should instead attach a `BodyPartHandle`, which points
to the multipart body, to the entity with the `Pose` for a single part.

[`Pose`]: ../trait.Pose.html
*/
#[derive(Shrinkwrap)]
#[shrinkwrap(mutable)]
// Ouch! Bad allocation story here with DenseVecStorage<Box<dyn Body>>.
// However, this is hard if not impossible to avoid due to nphysics API
// limitations.
pub struct BodyComponent<N: RealField>(pub Box<dyn Body<N>>);

impl<N: RealField> Component for BodyComponent<N> {
    type Storage = FlaggedStorage<Self, DenseVecStorage<Self>>;
}

impl<N: RealField> BodyComponent<N> {
    /// Creates a new Body Component.
    /// This is made useful by inserting the returned struct into the
    /// BodyComponent's Storage.
    pub fn new<B: Body<N>>(body: B) -> Self {
        Self(Box::new(body))
    }

    /// Attempts to cast this Body to a RigidBody.
    /// Just sugar for `Body::downcast_ref()`.
    pub fn as_rigid_body(&self) -> Option<&RigidBody<N>> {
        self.0.downcast_ref()
    }

    /// Attempts to mutably cast this Body to a RigidBody.
    /// Just sugar for `Body::downcast_mut()`.
    pub fn as_rigid_body_mut(&mut self) -> Option<&mut RigidBody<N>> {
        self.0.downcast_mut()
    }

    /// Attempts to cast this Body to a Multibody.
    /// Just sugar for `Body::downcast_ref()`.
    pub fn as_multi_body(&self) -> Option<&Multibody<N>> {
        self.0.downcast_ref()
    }

    /// Attempts to mutably cast this Body to a Multibody.
    /// Just sugar for `Body::downcast_mut()`.
    pub fn as_multi_body_mut(&mut self) -> Option<&mut Multibody<N>> {
        self.0.downcast_mut()
    }

    /// Attempts to cast this Body to Ground.
    /// Just sugar for `Body::downcast_ref()`.
    pub fn as_ground(&self) -> Option<&Ground<N>> {
        self.0.downcast_ref()
    }

    /// Attempts to mutably cast this Body to Ground.
    /// Just sugar for `Body::downcast_mut()`.
    pub fn as_ground_mut(&mut self) -> Option<&mut Ground<N>> {
        self.0.downcast_mut()
    }
}
