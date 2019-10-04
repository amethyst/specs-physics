use crate::{
    nalgebra::RealField,
    stepper::{Step, StepperRes},
};

use specs::{
    AccessorCow, BatchAccessor, BatchController, BatchUncheckedWorld, Dispatcher, RunningTime,
    System, World, WriteExpect,
};

use std::{marker::PhantomData, ops::DerefMut};

pub struct PhysicsBatchSystem<'a, 'b, N: RealField> {
    accessor: BatchAccessor,
    dispatcher: Dispatcher<'a, 'b>,
    marker: PhantomData<N>,
}

impl<'a, 'b, N: RealField> BatchController<'a, 'b> for PhysicsBatchSystem<'a, 'b, N> {
    type BatchSystemData = (WriteExpect<'a, StepperRes>, WriteExpect<'a, Step>);

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
        for step in data.0.fetch_mut::<StepperRes>().deref_mut() {
            data.0.fetch_mut::<Step>().update(step);
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
