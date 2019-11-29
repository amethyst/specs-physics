use crate::{nalgebra::RealField, stepper::StepperRes};

use specs::{
    AccessorCow, BatchAccessor, BatchController, BatchUncheckedWorld, Dispatcher, RunningTime,
    System, World, WriteExpect,
};

use std::{marker::PhantomData, ops::DerefMut};

/// Provides a batch system which uses `StepperRes` to keep a contained
/// dispatcher up to time
pub struct PhysicsBatchSystem<'a, 'b, N: RealField> {
    accessor: BatchAccessor,
    dispatcher: Dispatcher<'a, 'b>,
    marker: PhantomData<N>,
}

impl<'a, 'b, N: RealField> BatchController<'a, 'b> for PhysicsBatchSystem<'a, 'b, N> {
    type BatchSystemData = WriteExpect<'a, StepperRes>;

    unsafe fn create(accessor: BatchAccessor, dispatcher: Dispatcher<'a, 'b>) -> Self {
        PhysicsBatchSystem {
            accessor,
            dispatcher,
            marker: PhantomData,
        }
    }
}

impl<'a, 'b, 's, N: RealField> System<'s> for PhysicsBatchSystem<'a, 'b, N> {
    type SystemData = BatchUncheckedWorld<'s>;

    fn run(&mut self, data: Self::SystemData) {
        for _ in data.0.fetch_mut::<StepperRes>().deref_mut() {
            self.dispatcher.dispatch(data.0);
        }
    }

    fn running_time(&self) -> RunningTime {
        RunningTime::VeryLong
    }

    fn accessor<'c>(&'c self) -> AccessorCow<'s, 'c, Self> {
        AccessorCow::Ref(&self.accessor)
    }

    fn setup(&mut self, world: &mut World) {
        self.dispatcher.setup(world);
    }
}

unsafe impl<N: RealField> Send for PhysicsBatchSystem<'_, '_, N> {}
