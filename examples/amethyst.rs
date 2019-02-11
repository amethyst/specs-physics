use amethyst::assets::{Handle, Loader};
use amethyst::core::nalgebra::{Matrix3, Vector3};
use amethyst::core::specs::world::Builder;
use amethyst::core::specs::Join;
use amethyst::core::{GlobalTransform, Transform, TransformBundle};
use amethyst::renderer::{
    AmbientColor, Camera, DisplayConfig, DrawShaded, Light, Material, MaterialDefaults, MeshData,
    MeshHandle, Pipeline, PointLight, PosNormTex, RenderBundle, Rgba, ScreenDimensions, Shape,
    Stage, Texture, VirtualKeyCode,
};
use amethyst::input::{is_close_requested, is_key_down};
use amethyst::{Application, GameData, GameDataBuilder, SimpleState, StateData, StateEvent, SimpleTrans, Trans};
use nphysics_ecs_dumb::ncollide::shape::{Ball, ShapeHandle};
use nphysics_ecs_dumb::nphysics::math::Velocity;
use nphysics_ecs_dumb::nphysics::object::Material as PhysicsMaterial;
use nphysics_ecs_dumb::nphysics::volumetric::Volumetric;
use nphysics_ecs_dumb::*;
use num_traits::identities::One;
use std::time::Duration;

struct GameState;

impl SimpleState for GameState {
    fn on_start(&mut self, data: StateData<GameData>) {
        data.world.register::<DynamicBody>();
        data.world.register::<MeshData>();
        data.world.register::<Handle<Texture>>();

        // Create a texture for using.
        let texture = data
            .world
            .read_resource::<Loader>()
            .load_from_data::<Texture, ()>(
                [170.0, 170.0, 255.0, 1.0].into(),
                (),
                &data.world.read_resource(),
            );

        let material = Material {
            albedo: texture,
            ..data.world.read_resource::<MaterialDefaults>().0.clone()
        };

        // Get resolution of the screen.
        let (x, y) = {
            let resolution = data.world.res.fetch::<ScreenDimensions>();
            (resolution.width(), resolution.height())
        };

        let camera_transform = Transform::from(Vector3::new(0.0, 5.0, 5.0));

        // Add Camera
        data.world
            .create_entity()
            .with(Camera::standard_3d(x, y))
            .with(camera_transform)
            .build();

        // Add Light
        data.world.add_resource(AmbientColor(Rgba::from([0.2; 3])));
        data.world
            .create_entity()
            .with(Light::Point(PointLight {
                intensity: 50.0,
                color: Rgba::white(),
                radius: 5.0,
                smoothness: 4.0,
            }))
            .with(Transform::from(Vector3::new(2.0, 2.0, -2.0)))
            .build();

        let sphere_shape = Shape::Sphere(32, 32).generate::<Vec<PosNormTex>>(None);
        let sphere_handle: MeshHandle = data.world.read_resource::<Loader>().load_from_data(
            sphere_shape,
            (),
            &data.world.read_resource(),
        );

        let ball = ShapeHandle::new(Ball::new(1.0));

        // Add Sphere (todo: add many, add rigidbodies and colliders)
        data.world
            .create_entity()
            .with(sphere_handle.clone())
            .with(material.clone())
            .with(Transform::from(Vector3::new(0.0, 15.0, -10.0)))
            .with(GlobalTransform::default())
            .with(DynamicBody::new_rigidbody_with_velocity(
                Velocity::linear(0.0, 1.0, 0.0),
                10.0,
                Matrix3::one(),
                ball.center_of_mass(),
            ))
            .with(
                ColliderBuilder::from(ball.clone())
                    .collision_group(0)
                    .physics_material(PhysicsMaterial::default())
                    .build()
                    .unwrap(),
            )
            .build();

        // Add ground
        data.world
            .create_entity()
            .with(sphere_handle)
            .with(material)
            .with(Transform::from(Vector3::new(0.0, 0.0, -10.0)))
            .with(GlobalTransform::default())
            .with(
                //ColliderBuilder::from(ShapeHandle::new(Cuboid::new(Vector3::new(5.0, 1.0, 5.0))))
                ColliderBuilder::from(ball)
                    .collision_group(0)
                    .physics_material(PhysicsMaterial::default())
                    .build()
                    .unwrap(),
            )
            .build();

        //---------------------------------------------------- nphysics's ball3.rs adapted

        /*let mut physics_world = data.world.write_resource::<PhysicsWorld>();
        physics_world.set_gravity(Vector3::new(0.0, -9.81, 0.0));

        // Material for all objects.
        let material = PhysicsMaterial::default();
        let ground_shape =
            ShapeHandle::new(Cuboid::new(Vector3::repeat(1.0 - 0.01)));
        let ground_pos = Isometry3::new(Vector3::new(0.0, -0.5, -15.0), nalgebra::zero());

        physics_world.add_collider(
            0.01,
            ground_shape,
            BodyHandle::ground(),
            ground_pos,
            material.clone(),
        );
        let geom = ShapeHandle::new(Ball::new(1.0 - 0.01));
        let inertia = geom.inertia(1.0);
        let center_of_mass = geom.center_of_mass();


        let pos = Isometry3::new(Vector3::new(0.0, 5.0, -15.0), nalgebra::zero());
        let handle = physics_world.add_rigid_body(pos, inertia, center_of_mass);
        physics_world.add_collider(
            0.01,
            geom.clone(),
            handle,
            Isometry3::identity(),
            material.clone(),
        );*/
    }

    fn handle_event(
        &mut self,
        data: StateData<'_, GameData<'_, '_>>,
        event: StateEvent,
    ) -> SimpleTrans {
        if let StateEvent::Window(event) = &event {
            // Exit if user hits Escape or closes the window
            if is_close_requested(&event) || is_key_down(&event, VirtualKeyCode::Escape) {
                return Trans::Quit;
            }

            // 
            if is_key_down(&event, VirtualKeyCode::T) {
                *data.world.write_resource::<TimeStep>() = TimeStep::Fixed(1./120.);
                println!("Setting timestep to 1./120.");
            }

            if is_key_down(&event, VirtualKeyCode::Y) {
                *data.world.write_resource::<TimeStep>() = TimeStep::Fixed(1./60.);
                println!("Setting timestep to 1./60.");
            }

            if is_key_down(&event, VirtualKeyCode::S) {
                *data.world.write_resource::<TimeStep>() = TimeStep::SemiFixed(TimeStepConstraint::new(
                    vec![1. / 240., 1. / 120., 1. / 60.],
                    0.4,
                    Duration::from_millis(50),
                    Duration::from_millis(500),
                ))
            }

            // Reset the example
            if is_key_down(&event, VirtualKeyCode::Space) {
                *(&mut data.world.write_storage::<Transform>(), &data.world.read_storage::<DynamicBody>()).join().next().unwrap().0.translation_mut() = Vector3::new(0.0, 15.0, -10.0);
            }
        }
        Trans::None
    }
}

fn main() -> amethyst::Result<()> {
    amethyst::start_logger(Default::default());

    let display_config = DisplayConfig {
        title: "Amethyst + Nphysics".to_string(),
        fullscreen: false,
        dimensions: Some((800, 400)),
        min_dimensions: Some((800, 400)),
        max_dimensions: None,
        icon: None,
        vsync: true,
        multisampling: 0, // Must be multiple of 2, use 0 to disable
        visibility: true,
        always_on_top: false,
        decorations: true,
        maximized: false,
        multitouch: true,
        resizable: true,
        transparent: false,
    };
    let pipe = Pipeline::build().with_stage(
        Stage::with_backbuffer()
            .clear_target([0.1, 0.1, 0.1, 1.0], 1.0)
            .with_pass(DrawShaded::<PosNormTex>::new()),
    );

    let game_data = GameDataBuilder::default()
        .with_bundle(TransformBundle::new())?
        .with_bundle(
            PhysicsBundle::new()
                .with_dep(&["transform_system"])
                .with_timestep_iter_limit(20),
        )?
        .with_bundle(RenderBundle::new(pipe, Some(display_config)))?;

    let application = Application::new("./", GameState, game_data);

    assert_eq!(application.is_ok(), true);

    application.ok().unwrap().run();

    Ok(())
}
