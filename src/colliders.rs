use amethyst::ecs::{Component, DenseVecStorage, FlaggedStorage};
use nalgebra::Isometry3;
use ncollide::{shape::ShapeHandle, world::GeometricQueryType};
use ncollide3d::world::CollisionGroups;
use nphysics::material::BasicMaterial;
use nphysics::object::ColliderHandle;

#[derive(Clone, Serialize, Deserialize, Debug, new)]
pub enum ColliderType {
    Collider,
    Trigger,
}

impl Default for ColliderType {
    fn default() -> Self {
        ColliderType::Collider
    }
}

impl ColliderType {
    pub fn to_geometric_query_type(
        &self,
        margin: f32,
        prediction: f32,
        angular_prediction: f32,
    ) -> GeometricQueryType<f32> {
        match *self {
            ColliderType::Collider => {
                GeometricQueryType::Contacts(margin + prediction * 0.5, angular_prediction)
            }
            ColliderType::Trigger => GeometricQueryType::Proximity(prediction * 0.5),
        }
    }
}

#[derive(new, Clone, Builder)]
#[builder(pattern = "owned")]
pub struct Collider {
    #[new(default)]
    //#[serde(skip)]
    #[builder(default)]
    pub(crate) handle: Option<ColliderHandle>,
    /// Warning: Changing the margin after inserting the entity will have no effect.
    pub margin: f32,
    pub shape: ShapeHandle<f32>,
    pub offset_from_parent: Isometry3<f32>,
    pub physics_material: BasicMaterial<f32>,
    pub collision_group: CollisionGroups,
    pub query_type: ColliderType,
}

impl From<ShapeHandle<f32>> for ColliderBuilder {
    fn from(shape: ShapeHandle<f32>) -> ColliderBuilder {
        ColliderBuilder::default()
            .margin(0.01)
            .shape(shape)
            .offset_from_parent(Isometry3::identity())
            .query_type(ColliderType::default())
            .physics_material(BasicMaterial::default())
            .collision_group(CollisionGroups::default())
    }
}

impl ColliderBuilder {
    pub fn trigger(mut self) -> Self {
        // query type = trigger
        self.query_type = Some(ColliderType::Trigger);
        self
    }
}

impl Component for Collider {
    type Storage = FlaggedStorage<Self, DenseVecStorage<Self>>;
}
