use std::marker::PhantomData;

use specs::{
    storage::ComponentEvent, BitSet, Join, ReaderId, System,
    World, WriteStorage, Tracked,
};

use crate::{
    nalgebra::RealField,
    position::Position,
    world::WriteBodyStorage,
};

/// The `SyncBodiesToPhysicsSystem` handles the synchronisation of `PhysicsBody`
/// `Component`s into the physics `World`.
pub struct SyncBodiesToPhysicsSystem<N, P> {
    positions_reader_id: ReaderId<ComponentEvent>,
    n_marker: PhantomData<N>,
    p_marker: PhantomData<P>,
}

impl<N: RealField, P: Position<N>> SyncBodiesToPhysicsSystem<N, P> {
    fn new(world: &mut World) -> Self {
        SyncBodiesToPhysicsSystem {
            positions_reader_id: world.write_component::<P>().register_reader()
        }
    }
}

impl<'s, N, P> System<'s> for SyncBodiesToPhysicsSystem<N, P>
where
    N: RealField,
    P: Position<N>,
{
    type SystemData = (
        WriteBodyStorage<'s, N>,
        WriteStorage<'s, P>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            mut body_set,
            mut positions
        ) = data;

        // collect all ComponentEvents for the Position storage
        let modified_positions = BitSet::new();

        for component_event in positions.channel().read(self.positions_reader_id) {
            match component_event {
                ComponentEvent::Modified(id) => {
                    debug!("Got Modified event with id: {}", id);
                    modified_positions.add(*id);
                }
                _ => {}
            }
        }

        // iterate over PhysicsBody and Position components with an id/Index that
        // exists in either of the collected ComponentEvent BitSets
        for (mut body, position, _) in (&mut body_set, &positions, &modified_positions).join() {
            //rigid_body.set_position(*position.isometry());
        }
    }
}

/*
#[cfg(all(test, feature = "physics3d"))]
mod tests {
    use crate::{systems::SyncBodiesToPhysicsSystem, Physics, PhysicsBodyBuilder, SimplePosition};
    use nalgebra::Isometry3;
    use nphysics::object::BodyStatus;

    use specs::prelude::*;

    #[test]
    fn add_rigid_body() {
        let mut world = World::new();
        let mut dispatcher = DispatcherBuilder::new()
            .with(
                SyncBodiesToPhysicsSystem::<f32, SimplePosition<f32>>::default(),
                "sync_bodies_to_physics_system",
                &[],
            )
            .build();
        dispatcher.setup(&mut world);

        // create an Entity with the PhysicsBody component and execute the dispatcher
        world
            .create_entity()
            .with(SimplePosition::<f32>(Isometry3::<f32>::translation(
                1.0, 1.0, 1.0,
            )))
            .with(PhysicsBodyBuilder::<f32>::from(BodyStatus::Dynamic).build())
            .build();
        dispatcher.dispatch(&world);

        // fetch the Physics instance and check for new bodies
        let physics = world.read_resource::<Physics<f32>>();
        assert_eq!(physics.body_handles.len(), 1);
        assert_eq!(physics.world.bodies().count(), 1);
    }
}
*/
