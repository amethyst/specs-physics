/*use std::ops::Deref;

use specs::{
    storage::{ComponentEvent, MaskedStorage},
    BitSet,
    Component,
    ReaderId,
    Storage,
    Tracked,
};*/

pub use self::physics_stepper::PhysicsStepperSystem;

mod physics_stepper;
//mod sync_bodies_from_physics;
//mod sync_bodies_to_physics;
//mod sync_colliders_to_physics;
//mod sync_parameters_to_physics;

/*

/// Convenience function for configuring and building a `Dispatcher` with all
/// required physics related `System`s.
///
/// # Examples
/// ```rust
/// use specs_physics::SimplePosition;
/// let dispatcher = specs_physics::physics_dispatcher::<f32, SimplePosition<f32>>();
/// ```
pub fn physics_dispatcher<'a, 'b, N, P>() -> Dispatcher<'a, 'b>
    where
        N: RealField,
        P: Position<N>,
{
    let mut dispatcher_builder = DispatcherBuilder::new();
    register_physics_systems::<N, P>(&mut dispatcher_builder);

    dispatcher_builder.build()
}

/// Convenience function for registering all required physics related `System`s
/// to the given `DispatcherBuilder`. This also serves as a blueprint on how
/// to properly set up the `System`s and have them depend on each other.
pub fn register_physics_systems<N, P>(dispatcher_builder: &mut DispatcherBuilder)
    where
        N: RealField,
        P: Position<N>,
{
    // add SyncBodiesToPhysicsSystem first since we have to start with bodies;
    // colliders can exist without a body but in most cases have a body parent
    dispatcher_builder.add(
        SyncBodiesToPhysicsSystem::<N, P>::default(),
        "sync_bodies_to_physics_system",
        &[],
    );

    // add SyncCollidersToPhysicsSystem next with SyncBodiesToPhysicsSystem as its
    // dependency
    dispatcher_builder.add(
        SyncCollidersToPhysicsSystem::<N, P>::default(),
        "sync_colliders_to_physics_system",
        &["sync_bodies_to_physics_system"],
    );

    // add SyncParametersToPhysicsSystem; this System can be added at any point in
    // time as it merely synchronizes the simulation parameters of the world,
    // thus it has no other dependencies.
    dispatcher_builder.add(
        SyncParametersToPhysicsSystem::<N>::default(),
        "sync_parameters_to_physics_system",
        &[],
    );

    // add PhysicsStepperSystem after all other Systems that write data to the
    // nphysics World and has to depend on them; this System is used to progress the
    // nphysics World for all existing objects
    dispatcher_builder.add(
        PhysicsStepperSystem::<N>::default(),
        "physics_stepper_system",
        &[
            "sync_bodies_to_physics_system",
            "sync_colliders_to_physics_system",
            "sync_parameters_to_physics_system",
        ],
    );

    // add SyncBodiesFromPhysicsSystem last as it handles the
    // synchronisation between nphysics World bodies and the Position
    // components; this depends on the PhysicsStepperSystem
    dispatcher_builder.add(
        SyncBodiesFromPhysicsSystem::<N, P>::default(),
        "sync_bodies_from_physics_system",
        &["physics_stepper_system"],
    );
}


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
*/
