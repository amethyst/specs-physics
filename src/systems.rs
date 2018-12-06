use crate::bodies::DynamicBody;
use crate::forces::{DefaultForceGenerators, ForceGenerators};
use crate::{Gravity, World};
use amethyst::core::{GlobalTransform, Time, Transform};
use amethyst::ecs::storage::ComponentEvent;
use amethyst::ecs::{
    BitSet, Entities, Join, Read, ReadExpect, ReadStorage, ReaderId, Resources, System, SystemData,
    Write, WriteExpect, WriteStorage,
};
use amethyst::shrev::EventChannel;
use core::marker::PhantomData;
use nalgebra::try_convert;
use nalgebra::Vector3;
use ncollide3d::events::ContactEvent;
use nphysics3d::math::Inertia;
use nphysics3d::object::Body;

/// Iterates over entities with both a GlobalTransform and a PhysicsBody,
/// and synchronizes their state to an nphysics world simulation,
/// stepping the simulation each frame.
pub struct Dumb3dPhysicsSystem<F = DefaultForceGenerators>
where
    F: ForceGenerators,
{
    inserted_transforms: BitSet,
    modified_transforms: BitSet,

    inserted_physics_bodies: BitSet,
    modified_physics_bodies: BitSet,

    transforms_reader_id: Option<ReaderId<ComponentEvent>>,
    physics_bodies_reader_id: Option<ReaderId<ComponentEvent>>,

    phantom_force_generators: PhantomData<F>,
}

impl<F> Default for Dumb3dPhysicsSystem<F>
where
    F: ForceGenerators,
{
    fn default() -> Self {
        Dumb3dPhysicsSystem::<F> {
            inserted_transforms: BitSet::default(),
            modified_transforms: BitSet::default(),
            inserted_physics_bodies: BitSet::default(),
            modified_physics_bodies: BitSet::default(),
            transforms_reader_id: None,
            physics_bodies_reader_id: None,
            phantom_force_generators: PhantomData,
        }
    }
}

impl<'a, F> System<'a> for Dumb3dPhysicsSystem<F>
where
    F: ForceGenerators,
{
    type SystemData = (
        WriteExpect<'a, World>,
        Write<'a, EventChannel<ContactEvent>>,
        Read<'a, Time>,
        Entities<'a>,
        WriteStorage<'a, GlobalTransform>,
        WriteStorage<'a, DynamicBody>,
        ReadStorage<'a, Transform>,
        ReadExpect<'a, Gravity>,
        ReadStorage<'a, F::LocalForceGenerators>,
        ReadStorage<'a, F::LinkedForceGenerators>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            mut physical_world,
            _contact_events,
            time,
            entities,
            mut transforms,
            mut physics_bodies,
            locals,
            gravity,
            _local_force_generators,
            _linked_force_generators,
        ) = data;

        // Clear bitsets
        self.inserted_transforms.clear();
        self.modified_transforms.clear();
        self.inserted_physics_bodies.clear();
        self.modified_physics_bodies.clear();

        // Get change flag events for transforms, removing deleted ones from the physics world.
        {
            let events = transforms
                .channel()
                .read(&mut self.transforms_reader_id.as_mut().unwrap());
            for event in events {
                match event {
                    ComponentEvent::Modified(id) => {
                        self.modified_transforms.add(*id);
                    }
                    ComponentEvent::Inserted(id) => {
                        self.inserted_transforms.add(*id);
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
                        self.modified_physics_bodies.add(*id);
                    }
                    ComponentEvent::Inserted(id) => {
                        self.inserted_physics_bodies.add(*id);
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
            &self.modified_transforms
                | &self.inserted_transforms
                | &self.modified_physics_bodies
                | &self.inserted_physics_bodies,
        )
            .join()
        {
            if self.inserted_transforms.contains(id) || self.inserted_physics_bodies.contains(id) {
                println!("heya I'm new here!");
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
            } else if self.modified_transforms.contains(id)
                || self.modified_physics_bodies.contains(id)
            {
                println!("oh shit, I'm changed!");
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

        physical_world.set_gravity(*gravity);

        // Simulate world using the current time frame
        physical_world.set_timestep(time.delta_seconds());
        physical_world.step();

        //Why does this break. why
        // TODO: Fix contact events, use Entity id's instead of nphysics handles.
        //contact_events.iter_write(physical_world.contact_events());

        // Apply the updated values of the simulated world to our Components
        for (mut transform, mut body, local) in
            (&mut transforms, &mut physics_bodies, locals.maybe()).join()
        {
            let updated_body = physical_world.body(body.handle().unwrap());

            if updated_body.is_ground() || !updated_body.is_active() || updated_body.is_static() {
                continue;
            }

            match (body, updated_body) {
                (
                    DynamicBody::RigidBody(ref mut rigid_body),
                    Body::RigidBody(ref updated_rigid_body),
                ) => {
                    println!(
                        "super power: change mehhh! new pos: {:?}",
                        updated_rigid_body.position()
                    );

                    // TODO: Might get rid of the scale!!!
                    transform.0 = updated_rigid_body
                        .position()
                        .to_homogeneous()
                        .prepend_nonuniform_scaling(
                            local
                                .map(|tr| tr.scale())
                                .unwrap_or(&Vector3::new(1.0, 1.0, 1.0)),
                        );

                    rigid_body.velocity = *updated_rigid_body.velocity();
                    let inertia = updated_rigid_body.inertia();
                    rigid_body.mass = inertia.linear;
                    rigid_body.angular_mass = inertia.angular;
                    rigid_body.center_of_mass = updated_rigid_body.center_of_mass();
                }
                (DynamicBody::Multibody(_multibody), Body::Multibody(_updated_multibody)) => {
                    // match updated_multibody.links().next() {
                    //    Some(link) => link.position(),
                    //    None => continue,
                    // };
                }
                _ => println!("Unexpected body component and nphysics body pair."),
            };
        }

        // Now that we changed them all, let's remove all those pesky events that were generated.
        transforms
            .channel()
            .read(&mut self.transforms_reader_id.as_mut().unwrap())
            .for_each(|_| ());
        physics_bodies
            .channel()
            .read(&mut self.physics_bodies_reader_id.as_mut().unwrap())
            .for_each(|_| ());
    }

    fn setup(&mut self, res: &mut Resources) {
        Self::SystemData::setup(res);

        res.entry::<Gravity>()
            .or_insert_with(|| Gravity::new(0.0, -9.80665, 0.0));
        res.entry::<World>().or_insert_with(World::new);

        let mut transform_storage: WriteStorage<GlobalTransform> = SystemData::fetch(&res);
        self.transforms_reader_id = Some(transform_storage.register_reader());

        let mut physics_body_storage: WriteStorage<DynamicBody> = SystemData::fetch(&res);
        self.physics_bodies_reader_id = Some(physics_body_storage.register_reader());
    }
}
