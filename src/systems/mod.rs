use crate::{nalgebra::RealField, position::Position};

use specs::{Dispatcher, DispatcherBuilder};

mod physics_stepper;
mod sync_bodies_from_physics;
mod sync_bodies_to_physics;

pub use self::{
    physics_stepper::PhysicsStepperSystem, sync_bodies_from_physics::SyncBodiesFromPhysicsSystem,
    sync_bodies_to_physics::SyncBodiesToPhysicsSystem,
};

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
    //dispatcher_builder.add(
    //    SyncBodiesToPhysicsSystem::<N, P>::default(),
    //    "sync_bodies_to_physics_system",
    //    &[],
    //);

    // add PhysicsStepperSystem after all other Systems that write data to the
    // nphysics World and has to depend on them; this System is used to progress the
    // nphysics World for all existing objects
    dispatcher_builder.add(
        PhysicsStepperSystem::<N>::default(),
        "physics_stepper_system",
        &["sync_bodies_to_physics_system"],
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
