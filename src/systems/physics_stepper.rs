use crate::{
    events::{ContactEvent, ContactEvents, ContactType, ProximityEvent, ProximityEvents},
    nalgebra::RealField,
    ncollide::pipeline::narrow_phase::ContactEvent as NContactEvent,
    BodySetType,
    ColliderHandleType,
    ColliderSetType,
    ForceGeneratorSetType,
    GeometricalWorldType,
    JointConstraintSetType,
    MechanicalWorldType,
};

use specs::{world::Index, Entities, Entity, System, Write, WriteExpect};

use std::{convert::identity, marker::PhantomData};

/// The `PhysicsStepperSystem` progresses the nphysics `World`.
pub struct PhysicsStepperSystem<N: RealField> {
    n_marker: PhantomData<N>,
}

impl<'a, N: RealField> System<'a> for PhysicsStepperSystem<N> {
    type SystemData = (
        Entities<'a>,
        WriteExpect<'a, MechanicalWorldType<N>>,
        WriteExpect<'a, GeometricalWorldType<N>>,
        WriteExpect<'a, BodySetType<N>>,
        WriteExpect<'a, ColliderSetType<N>>,
        WriteExpect<'a, JointConstraintSetType<N>>,
        WriteExpect<'a, ForceGeneratorSetType<N>>,
        Write<'a, ContactEvents>,
        Write<'a, ProximityEvents>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            entities,
            mut mechanical_world,
            mut geometrical_world,
            mut body_set,
            mut collider_set,
            mut joint_constraint_set,
            mut force_generator_set,
            mut contact_events,
            mut proximity_events,
        ) = data;

        mechanical_world.step(
            &mut *geometrical_world,
            &mut *body_set,
            &mut *collider_set,
            &mut *joint_constraint_set,
            &mut *force_generator_set,
        );

        // map occurred ncollide ContactEvents to a custom ContactEvent type; this
        // custom type contains data that is more relevant for Specs users than
        // CollisionObjectHandles, such as the Entities that took part in the collision
        contact_events.iter_write(
            geometrical_world
                .contact_events()
                .iter()
                .map(|contact_event| {
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
                        collider1: (
                            entity_from_collision_object_handle(&entities, handle1, &collider_set),
                            handle1,
                        ),
                        collider2: (
                            entity_from_collision_object_handle(&entities, handle2, &collider_set),
                            handle2,
                        ),
                        contact_type,
                    }
                }),
        );

        // map occurred ncollide ProximityEvents to a custom ProximityEvent type; see
        // ContactEvents for reasoning
        proximity_events.iter_write(geometrical_world.proximity_events().iter().map(
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
                    collider1: (
                        entity_from_collision_object_handle(&entities, handle1, &collider_set),
                        handle1,
                    ),
                    collider2: (
                        entity_from_collision_object_handle(&entities, handle2, &collider_set),
                        handle2,
                    ),
                    prev_status,
                    new_status,
                }
            },
        ));
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
    collision_object_handle: ColliderHandleType,
    collider_set: &ColliderSetType<N>,
) -> Option<Entity> {
    collider_set
        .get(collision_object_handle)
        .map(|x| x.user_data())
        .and_then(identity)
        .map(|x| x.downcast_ref::<Index>())
        .and_then(identity)
        .map(|x| entities.entity(*x))
}
