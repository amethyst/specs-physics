use crate::{
    nalgebra::RealField,
    nphysics::object::{Body, BodyPartHandle, ColliderDesc},
    world::{BodyComponent, ColliderComponent},
};

use specs::{world::Builder, EntityBuilder, WorldExt};

/// Provides methods on `EntityBuilder` that make building
/// physics bodies easier.
///
/// # Usage
///
/// ```
/// use specs::Builder;
/// use specs_physics::{
///     ncollide::shape::{Ball, ShapeHandle},
///     nphysics::{math::Vector, object::{ColliderDesc, RigidBodyDesc}},
///     EntityBuilderExt, SimplePosition,
/// };
///
/// # use specs::WorldExt;
/// # let mut world = specs::World::new();
/// # world.register::<SimplePosition<f32>>();
/// # world.register::<specs_physics::ColliderComponent<f32>>();
/// # world.register::<specs_physics::BodyComponent<f32>>();
/// let body = RigidBodyDesc::new().translation(Vector::x() * 2.0).build();
/// let shape = ShapeHandle::new(Ball::new(1.6));
/// let collider_desc = ColliderDesc::new(shape);
///
/// let entity = world
///     .create_entity()
///     .with(SimplePosition::<f32>::default())
///     .with_body::<f32, _>(body)
///     .with_collider::<f32>(&collider_desc)
///     .build();
/// ```
pub trait EntityBuilderExt {
    /// Attaches `body` to this entity.
    fn with_body<N: RealField, B: Body<N>>(self, body: B) -> Self;
    fn with_collider<N: RealField>(self, collider: &ColliderDesc<N>) -> Self;
}

impl EntityBuilderExt for EntityBuilder<'_> {
    fn with_body<N: RealField, B: Body<N>>(self, body: B) -> Self {
        self.with(BodyComponent::new(body))
    }

    fn with_collider<N: RealField>(self, collider: &ColliderDesc<N>) -> Self {
        {
            let mut storage = self.world.write_storage::<ColliderComponent<N>>();
            storage
                .insert(
                    self.entity,
                    ColliderComponent(collider.build(BodyPartHandle(self.entity, 0))),
                )
                .unwrap();
        }
        self
    }
}
