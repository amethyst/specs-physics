mod physics_stepper;
mod sync_bodies_from_physics;
mod sync_bodies_to_physics;
mod sync_force_generators_to_physics;
mod sync_gravity_to_physics;

use crate::forces::ForceGenerators;
use amethyst::core::bundle::{Result, SystemBundle};
use amethyst::core::specs::DispatcherBuilder;
use core::marker::PhantomData;

pub use self::physics_stepper::PhysicsStepperSystem;
pub use self::sync_bodies_from_physics::SyncBodiesFromPhysicsSystem;
pub use self::sync_bodies_to_physics::SyncBodiesToPhysicsSystem;
pub use self::sync_force_generators_to_physics::SyncForceGeneratorsToPhysicsSystem;
pub use self::sync_gravity_to_physics::SyncGravityToPhysicsSystem;

// TODO: Implement contact events, use Entity id's instead of nphysics handles.
// contact_events.iter_write(physical_world.contact_events());

pub const SYNC_FORCE_GENERATORS_TO_PHYSICS_SYSTEM: &str = "sync_force_generators_to_physics_system";
pub const SYNC_BODIES_TO_PHYSICS_SYSTEM: &str = "sync_bodies_to_physics_system";
pub const SYNC_GRAVITY_TO_PHYSICS_SYSTEM: &str = "sync_gravity_to_physics_system";
pub const PHYSICS_STEPPER_SYSTEM: &str = "physics_stepper_system";
pub const SYNC_BODIES_FROM_PHYSICS_SYSTEM: &str = "sync_bodies_from_physics_system";

#[derive(Default)]
pub struct PhysicsBundle<'a, F>
where
    F: ForceGenerators,
{
    dep: &'a [&'a str],
    phantom_force_generators: PhantomData<F>,
}

impl<'a, F> PhysicsBundle<'a, F>
where
    F: ForceGenerators,
{
    pub fn new() -> Self {
        Default::default()
    }

    pub fn with_dep(mut self, dep: &'a [&'a str]) -> Self {
        self.dep = dep;
        self
    }
}

impl<'a, 'b, 'c, F: 'a> SystemBundle<'a, 'b> for PhysicsBundle<'c, F>
where
    F: ForceGenerators,
{
    fn build(self, builder: &mut DispatcherBuilder<'a, 'b>) -> Result<()> {
        builder.add(
            SyncForceGeneratorsToPhysicsSystem::<F>::new(),
            SYNC_FORCE_GENERATORS_TO_PHYSICS_SYSTEM,
            self.dep,
        );
        builder.add(
            SyncBodiesToPhysicsSystem::new(),
            SYNC_BODIES_TO_PHYSICS_SYSTEM,
            self.dep,
        );
        builder.add(
            SyncGravityToPhysicsSystem::new(),
            SYNC_GRAVITY_TO_PHYSICS_SYSTEM,
            self.dep,
        );

        builder.add(
            PhysicsStepperSystem::new(),
            PHYSICS_STEPPER_SYSTEM,
            &[
                SYNC_FORCE_GENERATORS_TO_PHYSICS_SYSTEM,
                SYNC_BODIES_TO_PHYSICS_SYSTEM,
                SYNC_GRAVITY_TO_PHYSICS_SYSTEM,
            ],
        );

        builder.add(
            SyncBodiesFromPhysicsSystem::new(),
            SYNC_BODIES_FROM_PHYSICS_SYSTEM,
            &[PHYSICS_STEPPER_SYSTEM],
        );

        Ok(())
    }
}
