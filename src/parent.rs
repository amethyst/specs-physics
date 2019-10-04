
/// The `PhysicsParent` `Component` is used to represent a parent/child
/// relationship between physics based `Entity`s.
#[derive(Debug, Clone, Eq, Ord, PartialEq, PartialOrd)]
pub struct PhysicsParent {
    pub entity: Entity,
}

impl Component for PhysicsParent {
    type Storage = FlaggedStorage<Self, DenseVecStorage<Self>>;
}

impl Parent for PhysicsParent {
    fn parent_entity(&self) -> Entity {
        self.entity
    }
}