use crate::time_step::TimeStep;
use crate::PhysicsWorld;
use amethyst::core::Time;
use amethyst::ecs::{Entity, Read, System, Write, WriteExpect};
use amethyst::shrev::EventChannel;
use ncollide3d::events::{ContactEvent, ProximityEvent};
use std::f32::EPSILON;
use std::time::Instant;

// TODO: why is this here
// Might want to replace by better types.
pub type EntityContactEvent = (Entity, Entity, ContactEvent);
pub type EntityProximityEvent = (Entity, Entity, ProximityEvent);

/// Falloff factor for calculating the moving average step time.
const AVERAGE_STEP_TIME_FALLOFF: f32 = 0.33;
/// Factor to apply to available physics time before decreasing the timestep. Makes sure that the
/// timestep isn't switched too eagerly.
const TIME_STEP_DECREASE_HYSTERESIS: f32 = 1.5;

/// Simulates a step of the physics world.
pub struct PhysicsStepperSystem {
    timestep_iter_limit: i32,
    time_accumulator: f32,
    avg_step_time: Option<f32>,
}

impl Default for PhysicsStepperSystem {
    fn default() -> Self {
        PhysicsStepperSystem {
            timestep_iter_limit: 10,
            time_accumulator: 0.,
            avg_step_time: None,
        }
    }
}

impl PhysicsStepperSystem {
    pub fn new(timestep_iter_limit: i32) -> Self {
        PhysicsStepperSystem {
            timestep_iter_limit,
            time_accumulator: 0.,
            avg_step_time: None,
        }
    }
}

impl<'a> System<'a> for PhysicsStepperSystem {
    type SystemData = (
        WriteExpect<'a, PhysicsWorld>,
        Read<'a, Time>,
        Write<'a, TimeStep>,
        Write<'a, EventChannel<EntityContactEvent>>,
        Write<'a, EventChannel<EntityProximityEvent>>,
    );

    // Simulate world using the current time frame
    fn run(&mut self, data: Self::SystemData) {
        let (
            mut physical_world,
            time,
            mut intended_timestep,
            mut contact_events,
            mut proximity_events,
        ) = data;

        let (timestep, mut change_timestep) = match &mut *intended_timestep {
            TimeStep::Fixed(timestep) => (*timestep, false),
            TimeStep::SemiFixed(constraint) => {
                let mut timestep = (constraint.current_timestep(), false);
                if let Some(avg_step) = self.avg_step_time {
                    // If the timestep is smaller than it takes to simulate that step, we have a problem.
                    // As simulated time is affected by the time scale, simulated time step / time scale
                    // is the maximum real time the step may take, so we take that into account here. We
                    // also take into account the maximum fraction of time physics are allowed to take
                    let adjusted_step_time =
                        avg_step * time.time_scale() / constraint.max_physics_time_fraction();
                    constraint.set_running_slow(constraint.current_timestep() < adjusted_step_time);
                    if constraint.should_increase_timestep() {
                        match constraint.increase_timestep() {
                            Err(error) => {
                                warn!("Failed to increase physics timestep! Error: {}", error);
                            }
                            Ok(new_timestep) => {
                                info!("Increasing physics timestep to {:.8} seconds", new_timestep);
                                timestep = (new_timestep, true);
                            }
                        }
                    } else if let Some(smaller_timestep) = constraint.smaller_timestep() {
                        // Check if we have enough time to simulate with a smaller timestep.
                        constraint.set_running_fast(
                            smaller_timestep > adjusted_step_time * TIME_STEP_DECREASE_HYSTERESIS,
                        );
                        if constraint.should_decrease_timestep() {
                            match constraint.decrease_timestep() {
                                Err(error) => {
                                    warn!("Failed to decrease physics timestep! Error: {}", error);
                                }
                                Ok(new_timestep) => {
                                    info!(
                                        "Decreasing physics timestep to {:.8} seconds",
                                        new_timestep
                                    );
                                    timestep = (new_timestep, true);
                                }
                            }
                        }
                    }
                }
                timestep
            }
        };

        if (physical_world.timestep() - timestep).abs() > EPSILON && !change_timestep {
            warn!("Physics world timestep out of sync with intended timestep! Physics timestep: {}, Requested timestep: {}", physical_world.timestep(), timestep);
            change_timestep = true;
        }

        if change_timestep {
            trace!("Changing physics timestep to {}", timestep);
            // reset average when changing timestep
            self.avg_step_time = None;
            physical_world.set_timestep(timestep);
        }

        self.time_accumulator += time.delta_seconds();
        let mut steps = 0;

        while steps <= self.timestep_iter_limit && self.time_accumulator >= timestep {
            let physics_time = Instant::now();

            trace!(
                "Stepping physics system. Step: {}, Timestep: {}, Time accumulator: {}",
                steps,
                timestep,
                self.time_accumulator
            );

            physical_world.step();

            trace!("iterating collision events.");

            let collision_world = physical_world.collider_world();

            let contact_ev = collision_world.contact_events().iter().cloned().flat_map(|ev| {
                    trace!("Emitting contact event: {:?}", ev);

                    let (handle1, handle2) = match ev {
                        ContactEvent::Started(h1, h2) => (h1, h2),
                        ContactEvent::Stopped(h1, h2) => (h1, h2),
                    };
                    let coll1 = physical_world.collider(handle1);
                    let coll2 = physical_world.collider(handle2);
                    if let (Some(c1), Some(c2)) = (coll1, coll2) {
                        // TODO: Check if the data is in fact the one we want. There might be
                        // user-inserted one.
                        let e1 = c1.user_data().map(|data| data.downcast_ref::<Entity>().unwrap());
                        let e2 = c2.user_data().map(|data| data.downcast_ref::<Entity>().unwrap());
                        if let (Some(e1), Some(e2)) = (e1, e2) {
                            Some((*e1, *e2, ev))
                        } else {
                            error!("Failed to find entity for collider during proximity event iteration. Was the entity removed?");
                            None
                        }
                    } else {
                        error!("Failed to fetch the rigid body from the physical world using the collider handle of the collision event. Was the entity removed?.");
                        None
                    }
                }).collect::<Vec<_>>();

            contact_events.iter_write(contact_ev.into_iter());

            let proximity_ev = collision_world
                    .proximity_events()
                    .iter()
                    .cloned()
                    .flat_map(|ev| {
                        trace!("Emitting proximity event: {:?}", ev);
                        println!("hello there");
                        let coll1 = physical_world.collider(ev.collider1);
                        let coll2 = physical_world.collider(ev.collider2);
                        if let (Some(c1), Some(c2)) = (coll1, coll2) {
                            // TODO: Check if the data is in fact the one we want. There might be
                            // user-inserted one.
                            let e1 = c1.user_data().map(|data| data.downcast_ref::<Entity>().unwrap());
                            let e2 = c2.user_data().map(|data| data.downcast_ref::<Entity>().unwrap());
                            if let (Some(e1), Some(e2)) = (e1, e2) {
                                Some((*e1, *e2, ev))
                            } else {
                                error!("Failed to find entity for collider during proximity event iteration. Was the entity removed?");
                                None
                            }
                        } else {
                            error!("Failed to fetch the rigid body from the physical world using the collider handle of the collision event. Was the entity removed?.");
                            None
                        }
                    }).collect::<Vec<_>>();

            proximity_events.iter_write(proximity_ev.into_iter());

            let physics_time = physics_time.elapsed();
            let physics_time =
                physics_time.as_secs() as f32 + physics_time.subsec_nanos() as f32 * 1e-9;
            self.avg_step_time = Some(match self.avg_step_time {
                None => physics_time,
                Some(avg) => {
                    // calculate exponentially weighted moving average
                    // basic formula: AVG_n = alpha * value_n + (1 - alpha) * AVG_n-1
                    avg + AVERAGE_STEP_TIME_FALLOFF * (physics_time - avg)
                }
            });
            self.time_accumulator -= timestep;
            steps += 1;
        }

        trace!(
            "Average time per physics step: {:.8} seconds",
            self.avg_step_time.unwrap_or_default()
        );

        if steps > self.timestep_iter_limit {
            // This shouldn't normally happen. If it does, one of the following might be true:
            // - TimeStep::Fixed was chosen too small
            // - TimeStep::SemiFixed can't increase the timestep
            // - Game itself is running slow, not leaving enough time for physics
            warn!("Physics running slow!");
        }
    }
}
