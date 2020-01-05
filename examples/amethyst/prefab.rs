use amethyst::{
    assets::{PrefabData, ProgressCounter},
    core::Transform,
    derive::PrefabData,
    ecs::prelude::*,
    error::Error,
    renderer::{
        camera::{ActiveCamera, Camera, Projection},
        formats::GraphicsPrefab,
        light::LightPrefab,
        rendy::mesh::MeshBuilder,
        shape::FromShape,
    },
    utils::removal::Removal,
};
use serde::{Deserialize, Serialize};
use specs_physics::{
    ncollide::shape::{Cuboid, ShapeHandle},
    nphysics::{
        math::{Vector, Velocity},
        object::{BodyPartHandle, BodyStatus, ColliderDesc, RigidBodyDesc}
    },
    BodyComponent,
    ColliderComponent,
};
use std::fmt::Debug;

#[derive(Deserialize, Debug, Serialize, PrefabData)]
#[serde(default)]
#[serde(deny_unknown_fields)]
pub struct CustomScenePrefab<V, R = ()>
where
    R: PartialEq + Debug + Clone + Send + Sync + 'static,
    V: FromShape + Into<MeshBuilder<'static>>,
{
    camera: Option<CustomCameraPrefab>,
    graphics: Option<GraphicsPrefab<V>>,
    light: Option<LightPrefab>,
    physics: Option<PhysicsPrefab>,
    removal: Option<Removal<R>>,
}

impl<V, R> Default for CustomScenePrefab<V, R>
where
    R: PartialEq + Debug + Clone + Send + Sync + 'static,
    V: FromShape + Into<MeshBuilder<'static>>,
{
    fn default() -> Self {
        CustomScenePrefab {
            camera: None,
            graphics: None,
            light: None,
            physics: None,
            removal: None,
        }
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct CustomCameraPrefab {
    #[serde(default)]
    active_camera: bool,
    projection: CameraProjection,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
enum CameraProjection {
    Orthographic {
        left: f32,
        right: f32,
        bottom: f32,
        top: f32,
        znear: f32,
        zfar: f32,
    },
    Perspective {
        aspect: f32,
        fovy: f32,
        znear: f32,
        zfar: f32,
    },
}

impl<'a> PrefabData<'a> for CustomCameraPrefab {
    type SystemData = (
        Write<'a, ActiveCamera>,
        WriteStorage<'a, Camera>,
    );
    type Result = ();

    fn add_to_entity(
        &self,
        entity: Entity,
        data: &mut Self::SystemData,
        _: &[Entity],
        _: &[Entity],
    ) -> Result<(), Error> {
        let (ref mut active_camera, ref mut camera_storage) = data;
        camera_storage.insert(
            entity,
            match self.projection {
                CameraProjection::Orthographic {
                    left,
                    right,
                    bottom,
                    top,
                    znear,
                    zfar,
                } => Camera::from(Projection::orthographic(left, right, bottom, top, znear, zfar)),
                CameraProjection::Perspective {
                    aspect,
                    fovy,
                    znear,
                    zfar,
                } => Camera::from(Projection::perspective(aspect, fovy, znear, zfar)),
            },
        )?;
        if self.active_camera {
            active_camera.entity = Some(entity);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub enum ColliderShape {
    Cuboid((f32, f32, f32)),
    None,
}

impl Default for ColliderShape {
    fn default() -> Self {
        ColliderShape::None
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub enum BodyStatusPrefab {
    Disabled,
    Static,
    Dynamic,
    Kinematic,
}

impl Default for BodyStatusPrefab {
    fn default() -> Self {
        BodyStatusPrefab::Dynamic
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct PhysicsPrefab {
    #[serde(default)]
    collider: ColliderShape,
    #[serde(default)]
    mass: f32,
    #[serde(default)]
    status: BodyStatusPrefab,
    #[serde(default)]
    transform: Transform,
    #[serde(default)]
    velocity: [f32; 3],
}

impl<'a> PrefabData<'a> for PhysicsPrefab {
    type SystemData = (
        WriteStorage<'a, BodyComponent<f32>>,
        WriteStorage<'a, ColliderComponent<f32>>,
        <Transform as PrefabData<'a>>::SystemData,
    );
    type Result = ();

    fn add_to_entity(
        &self,
        entity: Entity,
        system_data: &mut Self::SystemData,
        entities: &[Entity],
        children: &[Entity],
    ) -> Result<(), Error> {
        let (ref mut bodies, ref mut colliders, inner_transform) = system_data;
        match self.collider {
            ColliderShape::Cuboid((x, y, z)) => {
                let shape = ShapeHandle::new(Cuboid::new(Vector::new(x/2., y/2., z/2.)));
                colliders.insert(entity,
                    ColliderComponent(
                        ColliderDesc::new(shape)
                            .build(BodyPartHandle(entity, 0))
                    )
                )?;
            },
            ColliderShape::None => (),
        };
        bodies.insert(entity,
            BodyComponent::new(RigidBodyDesc::new()
                .mass(self.mass)
                .position(*self.transform.isometry())
                .status(match self.status {
                    BodyStatusPrefab::Disabled => BodyStatus::Disabled,
                    BodyStatusPrefab::Static => BodyStatus::Static,
                    BodyStatusPrefab::Dynamic => BodyStatus::Dynamic,
                    BodyStatusPrefab::Kinematic => BodyStatus::Kinematic,
                })
                .velocity(Velocity::linear(self.velocity[0], self.velocity[1], self.velocity[2]))
                .build()
        ))?;
        self.transform.add_to_entity(entity, inner_transform, entities, children)?;
        Ok(())
    }
}