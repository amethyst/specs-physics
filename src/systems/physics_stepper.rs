use std::marker::PhantomData;

use specs::{world::Index, Entities, Entity, Read, System, SystemData, World, Write, WriteExpect};

use crate::{
    events::{ContactEvent, ContactEvents, ContactType, ProximityEvent, ProximityEvents},
    nalgebra::RealField,
    ncollide::pipeline::{CollisionObjectSet, ContactEvent as NContactEvent},
    nphysics::object::{DefaultColliderHandle, DefaultColliderSet},
    parameters::TimeStep,
    Physics,
};

/// The `PhysicsStepperSystem` progresses the nphysics `World`.
pub struct PhysicsStepperSystem<N> {
    n_marker: PhantomData<N>,
}

impl<'s, N: RealField> System<'s> for PhysicsStepperSystem<N> {
    type SystemData = (
        Entities<'s>,
        Option<Read<'s, TimeStep<N>>>,
        Write<'s, ContactEvents>,
        Write<'s, ProximityEvents>,
        WriteExpect<'s, Physics<N>>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (entities, time_step, mut contact_events, mut proximity_events, mut physics) = data;

        // Convert physics from Write to &mut pointer so rustc can correctly reason
        // about independence of &mut borrows to struct components
        let physics: &mut Physics<N> = &mut *physics;

        // if a TimeStep resource exits, set the timestep for the nphysics integration
        // accordingly; this should not be required if the Systems are executed in a
        // fixed interval
        if let Some(time_step) = time_step {
            // only update timestep if it actually differs from the current nphysics World
            // one; keep in mind that changing the Resource will destabilize the simulation
            if physics.mechanical_world.timestep() != time_step.0 {
                warn!(
                    "TimeStep and world.timestep() differ, changing worlds timestep from {} to: {:?}",
                    physics.mechanical_world.timestep(),
                    time_step.0
                );
                physics.mechanical_world.set_timestep(time_step.0);
            }
        }

        physics.mechanical_world.step(
            &mut physics.geometrical_world,
            &mut physics.bodies,
            &mut physics.colliders,
            &mut physics.joint_constraints,
            &mut physics.force_generators,
        );

        // map occurred ncollide ContactEvents to a custom ContactEvent type; this
        // custom type contains data that is more relevant for Specs users than
        // CollisionObjectHandles, such as the Entities that took part in the collision
        contact_events.iter_write(physics.geometrical_world.contact_events().iter().map(
            |contact_event| {
                debug!("Got ContactEvent: {:?}", contact_event);
                // retrieve CollisionObjectHandles from ContactEvent and map the ContactEvent
                // type to our own custom ContactType
                let (handle1, handle2, contact_type) = match contact_event {
                    NContactEvent::Started(handle1, handle2) => {
                        (*handle1, *handle2, ContactType::Started)
                    }
                    NContactEvent::Stopped(handle1, handle2) => {
                        (*handle1, *handle2, ContactType::Stopped)
                    }
                };

                // create our own ContactEvent from the extracted data; mapping the
                // CollisionObjectHandles to Entities is error prone but should work as intended
                // as long as we're the only ones working directly with the nphysics World
                ContactEvent {
                    collider1: entity_from_collision_object_handle(
                        &entities,
                        handle1,
                        &physics.colliders,
                    ),
                    collider2: entity_from_collision_object_handle(
                        &entities,
                        handle2,
                        &physics.colliders,
                    ),
                    contact_type,
                }
            },
        ));

        // map occurred ncollide ProximityEvents to a custom ProximityEvent type; see
        // ContactEvents for reasoning
        proximity_events.iter_write(physics.geometrical_world.proximity_events().iter().map(
            |proximity_event| {
                debug!("Got ProximityEvent: {:?}", proximity_event);
                // retrieve CollisionObjectHandles and Proximity statuses from the ncollide
                // ProximityEvent
                let (handle1, handle2, prev_status, new_status) = (
                    proximity_event.collider1,
                    proximity_event.collider2,
                    proximity_event.prev_status,
                    proximity_event.new_status,
                );

                // create our own ProximityEvent from the extracted data; mapping
                // CollisionObjectHandles to Entities is once again error prone, but yeah...
                // ncollides Proximity types are mapped to our own types
                ProximityEvent {
                    collider1: entity_from_collision_object_handle(
                        &entities,
                        handle1,
                        &physics.colliders,
                    ),
                    collider2: entity_from_collision_object_handle(
                        &entities,
                        handle2,
                        &physics.colliders,
                    ),
                    prev_status,
                    new_status,
                }
            },
        ));
    }

    fn setup(&mut self, res: &mut World) {
        info!("PhysicsStepperSystem.setup");
        Self::SystemData::setup(res);

        // initialise required resources
        res.entry::<Physics<N>>().or_insert_with(Physics::default);
    }
}

impl<N> Default for PhysicsStepperSystem<N>
where
    N: RealField,
{
    fn default() -> Self {
        Self {
            n_marker: PhantomData,
        }
    }
}

fn entity_from_collision_object_handle<N: RealField>(
    entities: &Entities,
    collision_object_handle: DefaultColliderHandle,
    collider_set: &DefaultColliderSet<N>,
) -> Entity {
    entities.entity(
        *collider_set
            .collision_object(collision_object_handle)
            .unwrap()
            .user_data()
            .unwrap()
            .downcast_ref::<Index>()
            .unwrap(),
    )
}
