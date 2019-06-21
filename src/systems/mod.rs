use std::ops::Deref;

use specs::{
    storage::{ComponentEvent, MaskedStorage},
    BitSet,
    Component,
    ReaderId,
    Storage,
    Tracked,
};

pub use self::{
    physics_stepper::PhysicsStepperSystem,
    sync_bodies_from_physics::SyncBodiesFromPhysicsSystem,
    sync_bodies_to_physics::SyncBodiesToPhysicsSystem,
    sync_colliders_to_physics::SyncCollidersToPhysicsSystem,
    sync_parameters_to_physics::SyncParametersToPhysicsSystem,
};

mod physics_stepper;
mod sync_bodies_from_physics;
mod sync_bodies_to_physics;
mod sync_colliders_to_physics;
mod sync_parameters_to_physics;

/// Iterated over the `ComponentEvent::Inserted`s of a given, tracked `Storage`
/// and returns the results in a `BitSet`.
pub(crate) fn iterate_component_events<T, D>(
    tracked_storage: &Storage<T, D>,
    reader_id: &mut ReaderId<ComponentEvent>,
) -> (BitSet, BitSet, BitSet)
where
    T: Component,
    T::Storage: Tracked,
    D: Deref<Target = MaskedStorage<T>>,
{
    let (mut inserted, mut modified, mut removed) = (BitSet::new(), BitSet::new(), BitSet::new());
    for component_event in tracked_storage.channel().read(reader_id) {
        match component_event {
            ComponentEvent::Inserted(id) => {
                debug!("Got Inserted event with id: {}", id);
                inserted.add(*id);
            }
            ComponentEvent::Modified(id) => {
                debug!("Got Modified event with id: {}", id);
                modified.add(*id);
            }
            ComponentEvent::Removed(id) => {
                debug!("Got Removed event with id: {}", id);
                removed.add(*id);
            }
        }
    }

    (inserted, modified, removed)
}
