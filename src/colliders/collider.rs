
use ncollide::{
    shape::ShapeHandle,
    world::GeometricQueryType,
};
use nphysics::object::{Material, ColliderHandle};
use nalgebra::Isometry3;
use amethyst::ecs::{Component, FlaggedStorage, DenseVecStorage};

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
    pub fn to_geometric_query_type(&self, margin: f32, prediction: f32, angular_prediction: f32) -> GeometricQueryType<f32> {
        match *self {
            ColliderType::Collider => GeometricQueryType::Contacts(margin + prediction * 0.5, angular_prediction),
            ColliderType::Trigger => GeometricQueryType::Proximity(prediction * 0.5),
        }
    }
}

#[derive(new, Clone, Builder/*, Serialize, Deserialize*/)]
#[builder(pattern = "owned")]
pub struct Collider {
    #[new(default)]
    //#[serde(skip)]
    pub(crate) handle: Option<ColliderHandle>,
    /// Warning: Changing the margin after inserting the entity will have no effect.
    pub margin: f32,
    pub shape: ShapeHandle<f32>,
    pub offset_from_parent: Isometry3<f32>,
    pub physics_material: Material<f32>,
    pub collision_group: u32,
    pub query_type: ColliderType,
}

impl From<ShapeHandle<f32>> for ColliderBuilder {
    fn from(shape: ShapeHandle<f32>) -> ColliderBuilder {
        ColliderBuilder::default()
            .margin(0.01)
            .shape(shape)
            .offset_from_parent(Isometry3::identity())
            .query_type(ColliderType::default())
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
