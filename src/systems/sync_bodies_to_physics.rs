use crate::bodies::DynamicBody;
use crate::World;
use amethyst::core::GlobalTransform;
use amethyst::ecs::storage::ComponentEvent;
use amethyst::ecs::{
    BitSet, Entities, Join, ReaderId, Resources, System, SystemData, Write, WriteExpect,
    WriteStorage,
};
use amethyst::shrev::EventChannel;
use nalgebra::try_convert;
use ncollide3d::events::ContactEvent;
use nphysics3d::math::Inertia;

#[derive(Default)]
pub struct SyncBodiesToPhysicsSystem {
    transforms_reader_id: Option<ReaderId<ComponentEvent>>,
    physics_bodies_reader_id: Option<ReaderId<ComponentEvent>>,
}

impl SyncBodiesToPhysicsSystem {
    pub fn new() -> Self {
        Default::default()
    }
}

impl<'a> System<'a> for SyncBodiesToPhysicsSystem {
    type SystemData = (
        WriteExpect<'a, World>,
        Write<'a, EventChannel<ContactEvent>>,
        Entities<'a>,
        WriteStorage<'a, GlobalTransform>,
        WriteStorage<'a, DynamicBody>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (mut physical_world, _contact_events, entities, mut transforms, mut physics_bodies) =
            data;

        let mut inserted_transforms = BitSet::new();
        let mut modified_transforms = BitSet::new();
        let mut inserted_physics_bodies = BitSet::new();
        let mut modified_physics_bodies = BitSet::new();

        // Get change flag events for transforms, removing deleted ones from the physics world.
        {
            let events = transforms
                .channel()
                .read(&mut self.transforms_reader_id.as_mut().unwrap());
            for event in events {
                match event {
                    ComponentEvent::Modified(id) => {
                        modified_transforms.add(*id);
                    }
                    ComponentEvent::Inserted(id) => {
                        inserted_transforms.add(*id);
                    }
                    ComponentEvent::Removed(id) => {
                        physical_world.remove_bodies(&[physics_bodies
                            .get(entities.entity(*id))
                            .unwrap()
                            .handle()
                            .unwrap()]);
                    }
                };
            }
        }

        // Get change flag events for physics bodies, removing deleted ones from the physics world.
        {
            let events = physics_bodies
                .channel()
                .read(&mut self.physics_bodies_reader_id.as_mut().unwrap());
            for event in events {
                match event {
                    ComponentEvent::Modified(id) => {
                        modified_physics_bodies.add(*id);
                    }
                    ComponentEvent::Inserted(id) => {
                        inserted_physics_bodies.add(*id);
                        println!("I'm in!");
                    }
                    ComponentEvent::Removed(id) => {
                        physical_world.remove_bodies(&[physics_bodies
                            .get(entities.entity(*id))
                            .unwrap()
                            .handle()
                            .unwrap()]);
                    }
                };
            }
        }

        // Update simulation world with the value of Components flagged as changed
        for (_entity, transform, mut body, id) in (
            &entities,
            &transforms,
            &mut physics_bodies,
            &modified_transforms
                | &inserted_transforms
                | &modified_physics_bodies
                | &inserted_physics_bodies,
        )
            .join()
        {
            if inserted_transforms.contains(id) || inserted_physics_bodies.contains(id) {
                println!("Detected inserted dynamics body with id {:?}", id);
                match body {
                    DynamicBody::RigidBody(ref mut rigid_body) => {
                        // Just inserted. Remove old one and insert new.
                        if rigid_body.handle.is_some()
                            && physical_world
                                .rigid_body(rigid_body.handle.unwrap())
                                .is_some()
                        {
                            physical_world.remove_bodies(&[rigid_body.handle.unwrap()]);
                        }

                        rigid_body.handle = Some(physical_world.add_rigid_body(
                            try_convert(transform.0).unwrap(),
                            Inertia::new(rigid_body.mass, rigid_body.angular_mass),
                            rigid_body.center_of_mass,
                        ));

                        let physical_body = physical_world
                            .rigid_body_mut(rigid_body.handle.unwrap())
                            .unwrap();

                        physical_body.set_velocity(rigid_body.velocity);
                    }
                    DynamicBody::Multibody(_) => {
                        // TODO
                    }
                }
            } else if modified_transforms.contains(id) || modified_physics_bodies.contains(id) {
                println!("Detected changed dynamics body with id {:?}", id);
                match body {
                    DynamicBody::RigidBody(ref mut rigid_body) => {
                        let physical_body = physical_world
                            .rigid_body_mut(rigid_body.handle.unwrap())
                            .unwrap();

                        physical_body.set_position(try_convert(transform.0).unwrap());
                        physical_body.set_velocity(rigid_body.velocity);

                        // if you changed the mass properties at all... too bad!
                    }
                    DynamicBody::Multibody(_) => {
                        // TODO
                    }
                }
            }
        }
    }

    fn setup(&mut self, res: &mut Resources) {
        Self::SystemData::setup(res);

        let mut transform_storage: WriteStorage<GlobalTransform> = SystemData::fetch(&res);
        self.transforms_reader_id = Some(transform_storage.register_reader());

        let mut physics_body_storage: WriteStorage<DynamicBody> = SystemData::fetch(&res);
        self.physics_bodies_reader_id = Some(physics_body_storage.register_reader());
    }
}
