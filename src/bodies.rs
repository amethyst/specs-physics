use amethyst::ecs::{Component, FlaggedStorage};
use nalgebra::Matrix3;
use nphysics3d::math::{Force, Point, Velocity};
use nphysics3d::object::{BodyHandle, BodyStatus};

/// Rigid physics body, for use in `PhysicsBody` Component.
/// Currently only the velocity is read and updated at runtime.
/// The properties of mass are only written at physics body creation time.
#[derive(Serialize, Deserialize, Clone, Copy, Debug, new)]
pub struct DynamicBody {
    #[serde(skip)]
    #[new(default)]
    pub(crate) handle: Option<BodyHandle>,
    pub velocity: Velocity<f32>,

    // TODO: update these in the physics system below.
    pub mass: f32,
    pub angular_mass: Matrix3<f32>,
    pub center_of_mass: Point<f32>,

    pub external_forces: Force<f32>,
    #[serde(skip)]
    pub body_status: BodyStatus,
}

impl DynamicBody {
    pub fn new_rigidbody(
        mass: f32,
        angular_mass: Matrix3<f32>,
        center_of_mass: Point<f32>,
    ) -> Self {
        DynamicBody {
            handle: None,
            velocity: Velocity::<f32>::zero(),
            mass,
            angular_mass,
            center_of_mass,
            external_forces: Force::<f32>::zero(),
            body_status: BodyStatus::Dynamic,
        }
    }

    pub fn new_rigidbody_with_velocity(
        velocity: Velocity<f32>,
        mass: f32,
        angular_mass: Matrix3<f32>,
        center_of_mass: Point<f32>,
    ) -> Self {
        DynamicBody {
            handle: None,
            velocity,
            mass,
            angular_mass,
            center_of_mass,
            external_forces: Force::<f32>::zero(),
            body_status: BodyStatus::Dynamic,
        }
    }

    pub fn handle(&self) -> Option<BodyHandle> {
        self.handle
    }
}

impl Component for DynamicBody {
    type Storage = FlaggedStorage<Self>;
}
