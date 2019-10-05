use crate::{handle::BodyHandle, nalgebra::RealField, nphysics::object::Body, BodySetRes};

use specs::{world::Builder, EntityBuilder};

pub trait EntityBuilderExt {
    fn with_body<N: RealField>(self, set: &mut BodySetRes<N>, body: impl Body<N>) -> Self;
}

impl EntityBuilderExt for EntityBuilder<'_> {
    fn with_body<N: RealField>(self, set: &mut BodySetRes<N>, body: impl Body<N>) -> Self {
        self.with(BodyHandle(set.insert(body)))
    }
}
