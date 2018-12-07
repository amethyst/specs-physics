use crate::forces::{DefaultForceGenerators, ForceGenerators};
use crate::World;
use amethyst::ecs::{ReadStorage, System, Write, WriteExpect};
use amethyst::shrev::EventChannel;
use core::marker::PhantomData;
use ncollide3d::events::ContactEvent;

// TODO: Synchronize force generators, tidy up api.

pub struct SyncForceGeneratorsToPhysicsSystem<F = DefaultForceGenerators>
where
    F: ForceGenerators,
{
    phantom_force_generators: PhantomData<F>,
}

impl<F> SyncForceGeneratorsToPhysicsSystem<F>
where
    F: ForceGenerators,
{
    pub fn new() -> Self {
        Default::default()
    }
}

impl<F> Default for SyncForceGeneratorsToPhysicsSystem<F>
where
    F: ForceGenerators,
{
    fn default() -> Self {
        SyncForceGeneratorsToPhysicsSystem::<F> {
            phantom_force_generators: PhantomData,
        }
    }
}

impl<'a, F> System<'a> for SyncForceGeneratorsToPhysicsSystem<F>
where
    F: ForceGenerators,
{
    type SystemData = (
        WriteExpect<'a, World>,
        Write<'a, EventChannel<ContactEvent>>,
        ReadStorage<'a, F::LocalForceGenerators>,
        ReadStorage<'a, F::LinkedForceGenerators>,
    );

    fn run(&mut self, _data: Self::SystemData) {}
}
