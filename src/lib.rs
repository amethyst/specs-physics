extern crate amethyst;
pub extern crate ncollide3d as ncollide;
pub extern crate nphysics3d as nphysics;

use self::ncollide::events::ContactEvent;
use self::nphysics::math::{Inertia, Point, Vector, Velocity};
use self::nphysics::object::{Body, BodyHandle, RigidBody};
use amethyst::core::nalgebra::base::Matrix3;
use amethyst::core::nalgebra::try_convert;
use amethyst::core::{GlobalTransform, Time};
use amethyst::ecs::prelude::*;
use amethyst::ecs::*;
use amethyst::shrev::EventChannel;

pub type World = self::nphysics::world::World<f32>;
pub type Gravity = Vector<f32>;

/// Physics body component for describing (currently) rigid body dynamics.
pub enum PhysicsBody {
    RigidBody(RigidPhysicsBody),
    Multibody(PhysicsMultibody),
    Ground(PhysicsGround),
}

impl Component for PhysicsBody {
    type Storage = FlaggedStorage<Self, DenseVecStorage<Self>>;
}

/// Rigid physics body, for use in `PhysicsBody` Component.
/// Currently only the velocity is read and updated at runtime.
/// The properties of mass are only written at physics body creation time.
pub struct RigidPhysicsBody {
    handle: Option<BodyHandle>,
    pub velocity: Velocity<f32>,

    // TODO: update these in the physics system below.
    pub mass: f32,
    pub angular_mass: Matrix3<f32>,
    pub center_of_mass: Point<f32>,
}

/// Multipart physics body, for use in `PhysicsBody` Component. Not implemented yet.
pub struct PhysicsMultibody {
    handle: Option<BodyHandle>,
}

/// Nphysics ground physics body, for use in `PhysicsBody` Not implemented yet.
pub struct PhysicsGround {
    handle: Option<BodyHandle>,
}

impl PhysicsBody {
    fn handle(&self) -> Option<BodyHandle> {
        match self {
            PhysicsBody::RigidBody(x) => x.handle,
            PhysicsBody::Multibody(x) => x.handle,
            PhysicsBody::Ground(x) => x.handle,
        }
    }
}

/// Iterates over entities with both a GlobalTransform and a PhysicsBody,
/// and synchronizes their state to an nphysics world simulation,
/// stepping the simulation each frame.
#[derive(Default)]
pub struct Dumb3dPhysicsSystem {
    inserted_transforms: BitSet,
    modified_transforms: BitSet,

    inserted_physics_bodies: BitSet,
    modified_physics_bodies: BitSet,

    transforms_reader_id: Option<ReaderId<ComponentEvent>>,
    physics_bodies_reader_id: Option<ReaderId<ComponentEvent>>,
}

impl<'a> System<'a> for Dumb3dPhysicsSystem {
    type SystemData = (
        WriteExpect<'a, World>,
        Write<'a, EventChannel<ContactEvent>>,
        Read<'a, Time>,
        Entities<'a>,
        WriteStorage<'a, GlobalTransform>,
        WriteStorage<'a, PhysicsBody>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (
            mut physical_world,
            mut contact_events,
            time,
            entities,
            mut transforms,
            mut physics_bodies,
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
        for (entity, mut transform, mut body, id) in (
            &entities,
            &mut transforms,
            &mut physics_bodies,
            &self.modified_transforms
                | &self.inserted_transforms
                | &self.modified_physics_bodies
                | &self.inserted_physics_bodies,
        )
            .join()
        {
            if self.inserted_transforms.contains(id) || self.inserted_physics_bodies.contains(id) {
                match body {
                    PhysicsBody::RigidBody(ref mut rigid_body) => {
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

                        let mut physical_body = physical_world
                            .rigid_body_mut(rigid_body.handle.unwrap())
                            .unwrap();

                        physical_body.set_velocity(rigid_body.velocity);
                    }
                    PhysicsBody::Multibody(x) => {
                        // TODO
                    }
                    PhysicsBody::Ground(x) => {
                        // TODO
                    }
                }
            } else if self.modified_transforms.contains(id)
                || self.modified_physics_bodies.contains(id)
            {
                match body {
                    PhysicsBody::RigidBody(ref mut rigid_body) => {
                        let mut physical_body = physical_world
                            .rigid_body_mut(rigid_body.handle.unwrap())
                            .unwrap();

                        physical_body.set_position(try_convert(transform.0).unwrap());
                        physical_body.set_velocity(rigid_body.velocity);

                        // if you changed the mass properties at all... too bad!
                    }
                    PhysicsBody::Multibody(x) => {
                        // TODO
                    }
                    PhysicsBody::Ground(x) => {
                        // TODO
                    }
                }
            }
        }

        // Simulate world using the current time frame
        physical_world.set_timestep(time.delta_seconds());
        physical_world.step();

        //Why does this break. why
        // TODO: Fix contact events, use Entity id's instead of nphysics handles.
        //contact_events.iter_write(physical_world.contact_events());

        // Apply the updated values of the simulated world to our Components
        for (mut transform, mut body) in (&mut transforms, &mut physics_bodies).join() {
            let updated_body = physical_world.body(body.handle().unwrap());

            if updated_body.is_ground() || !updated_body.is_active() || updated_body.is_static() {
                continue;
            }

            match (body, updated_body) {
                (
                    PhysicsBody::RigidBody(ref mut rigid_body),
                    Body::RigidBody(ref updated_rigid_body),
                ) => {
                    updated_rigid_body.position();
                    rigid_body.velocity = *updated_rigid_body.velocity();
                    let inertia = updated_rigid_body.inertia();
                    rigid_body.mass = inertia.linear;
                    rigid_body.angular_mass = inertia.angular;
                    rigid_body.center_of_mass = updated_rigid_body.center_of_mass();
                }
                (PhysicsBody::Multibody(multibody), Body::Multibody(updated_multibody)) => {
                    match updated_multibody.links().next() {
                        Some(link) => link.position(),
                        None => continue,
                    };
                }
                (PhysicsBody::Ground(ground), Body::Ground(updated_ground)) => {
                    updated_ground.position();
                }
                _ => {}
            };
        }
    }

    // TODO: resources need set up here. Including initializing the physics world.
    fn setup(&mut self, res: &mut Resources) {
        Self::SystemData::setup(res);

        res.entry::<Gravity>()
            .or_insert_with(|| Gravity::new(0.0, -9.80665, 0.0));
        res.entry::<World>().or_insert_with(|| World::new());

        let mut transform_storage: WriteStorage<GlobalTransform> = SystemData::fetch(&res);
        self.transforms_reader_id = Some(transform_storage.register_reader());

        let mut physics_body_storage: WriteStorage<PhysicsBody> = SystemData::fetch(&res);
        self.physics_bodies_reader_id = Some(physics_body_storage.register_reader());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use amethyst::assets::Handle;
    use amethyst::core::nalgebra::Vector3;
    use amethyst::core::transform::bundle::TransformBundle;
    use amethyst::core::Transform;
    use amethyst::prelude::Builder;
    use amethyst::renderer::*;
    use amethyst::{Application, GameData, GameDataBuilder, SimpleState, StateData};

    struct GameState;

    impl<'a, 'b> SimpleState<'a, 'b> for GameState {
        fn on_start(&mut self, data: StateData<GameData>) {
            data.world.register::<PhysicsBody>();
            data.world.register::<MeshData>();
            data.world.register::<Handle<Texture>>();

            // Create a texture for using.
            let texture = data
                .world
                .read_resource::<amethyst::assets::Loader>()
                .load_from_data::<Texture, ()>(
                    [170.0, 170.0, 255.0, 1.0].into(),
                    (),
                    &data.world.read_resource(),
                );

            // Get resolution of the screen.
            let (x, y) = {
                let resolution = data.world.res.fetch::<ScreenDimensions>();
                (resolution.width(), resolution.height())
            };

            {
                let mut camera_transform = Transform::from(Vector3::new(0.0, 0.0, -4.0));

                camera_transform.yaw_local(-3.142);

                // Add Camera
                data.world
                    .create_entity()
                    .with(Camera::standard_3d(x, y))
                    .with(camera_transform)
                    .build();
            }

            // Add Light
            data.world.add_resource(AmbientColor(Rgba::from([0.01; 3])));
            data.world
                .create_entity()
                .with(Light::Point(PointLight {
                    intensity: 3.0,
                    color: Rgba::white(),
                    radius: 5.0,
                    smoothness: 4.0,
                }))
                .with(Transform::from(Vector3::new(2.0, 2.0, -2.0)))
                .build();

            // Add Sphere (todo: add many, add rigidbodies and colliders)
            data.world
                .create_entity()
                .with(Shape::Sphere(32, 32).generate::<Vec<PosNormTex>>(None))
                .with(texture)
                .with(GlobalTransform::default())
                .build();
        }
    }

    #[test]
    fn app() -> amethyst::Result<()> {
        amethyst::start_logger(Default::default());

        let game_data = GameDataBuilder::default()
            .with_basic_renderer(
                "./resources/display.ron",
                DrawShaded::<PosNormTex>::new(),
                false,
            )?
            .with_bundle(TransformBundle::new())?
            .with(Dumb3dPhysicsSystem::default(), "physics", &[]);

        let mut application = Application::new("./", GameState, game_data);

        assert_eq!(application.is_ok(), true);

        application.ok().unwrap().run();

        Ok(())
    }
}
