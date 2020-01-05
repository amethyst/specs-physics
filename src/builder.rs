use crate::{
    bodies::{BodyComponent, GroundMarker, MultibodyMarker, RigidBodyMarker},
    colliders::ColliderComponent,
    nalgebra::RealField,
    nphysics::object::{Body, BodyPartHandle, ColliderDesc},
};

use specs::{world::Builder, EntityBuilder, WorldExt};

/**
Provides extension methods to build physics objects with [`EntityBuilder`].

# Usage

```
use specs::Builder;
use specs_physics::{
    ncollide::shape::{Ball, ShapeHandle},
    nphysics::{math::Vector, object::{ColliderDesc, RigidBodyDesc}},
    EntityBuilderExt, SimplePosition,
};

# use specs::WorldExt;
# let mut world = specs::World::new();
# world.register::<SimplePosition<f32>>();
# world.register::<specs_physics::ColliderComponent<f32>>();
# world.register::<specs_physics::BodyComponent<f32>>();
let body = RigidBodyDesc::new().translation(Vector::x() * 2.0).build();
let shape = ShapeHandle::new(Ball::new(1.6));
let collider_desc = ColliderDesc::new(shape);

let entity = world
    .create_entity()
    .with(SimplePosition::<f32>::default())
    .with_body::<f32, _>(body)
    .with_collider::<f32>(&collider_desc)
    .build();
```
*/
pub trait EntityBuilderExt {
    /// Attaches `body` to this entity.
    fn with_body<N: RealField, B: Body<N>>(self, body: B) -> Self;
    /// Builds a `collider` to point at the body part of index `0` on this
    /// entity. So, the body itself for bodies without parts, such as
    /// Ground's or RigidBody's.
    fn with_collider<N: RealField>(self, collider: &ColliderDesc<N>) -> Self;
}

impl EntityBuilderExt for EntityBuilder<'_> {
    fn with_body<N: RealField, B: Body<N>>(self, body: B) -> Self {
        let component = BodyComponent::new(body);

        // Reflect on the component type and add relevant markers
        // Branches should be optimized away at compilation time
        if component.as_rigid_body().is_some() {
            self.world
                .write_storage::<RigidBodyMarker>()
                .insert(self.entity, RigidBodyMarker)
                // Guaranteed to not fail by the lifetime in the EntityBuilder.
                .unwrap();
        } else if component.as_multi_body().is_some() {
            self.world
                .write_storage::<MultibodyMarker>()
                .insert(self.entity, MultibodyMarker)
                // Ditto.
                .unwrap();
        } else if component.as_ground().is_some() {
            self.world
                .write_storage::<GroundMarker>()
                .insert(self.entity, GroundMarker)
                // Ditto.
                .unwrap();
        }

        self.with(component)
    }

    fn with_collider<N: RealField>(self, collider: &ColliderDesc<N>) -> Self {
        self.world
            .write_storage::<ColliderComponent<N>>()
            .insert(
                self.entity,
                ColliderComponent(collider.build(BodyPartHandle(self.entity, 0))),
            )
            // Guaranteed to not fail by the lifetime in the EntityBuilder.
            .unwrap();

        self
    }
}
