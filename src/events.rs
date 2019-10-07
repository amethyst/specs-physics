use specs::Entity;

use crate::{ncollide::query::Proximity, world::ColliderHandleType};
use specs::shrev::EventChannel;

/// The `ContactType` is set accordingly to whether a contact began or ended.
#[derive(Debug)]
pub enum ContactType {
    /// Event occurring when two collision objects start being in contact.
    Started,
    /// Event occurring when two collision objects stop being in contact.    
    Stopped,
}

/// The `ContactEvent` type contains information about the objects that
/// collided.
#[derive(Debug)]
pub struct ContactEvent {
    pub collider1: (Option<Entity>, ColliderHandleType),
    pub collider2: (Option<Entity>, ColliderHandleType),

    pub contact_type: ContactType,
}

/// `ContactEvents` is a custom `EventChannel` type used to expose
/// `ContactEvent`s.
pub type ContactEvents = EventChannel<ContactEvent>;

/// The `ProximityEvent` type contains information about the objects that
/// triggered a proximity "collision". These kind of events contain at least one
/// *sensor* `PhysicsCollider`.
#[derive(Debug)]
pub struct ProximityEvent {
    pub collider1: (Option<Entity>, ColliderHandleType),
    pub collider2: (Option<Entity>, ColliderHandleType),

    pub prev_status: Proximity,
    pub new_status: Proximity,
}

/// `ProximityEvent` is a custom `EventChannel` type used to expose
/// `ProximityEvent`s.
pub type ProximityEvents = EventChannel<ProximityEvent>;
