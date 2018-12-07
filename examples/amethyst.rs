use amethyst::assets::{Handle, Loader};
use amethyst::core::nalgebra::{Matrix3, Vector3};
use amethyst::core::specs::world::Builder;
use amethyst::core::{GlobalTransform, Transform, TransformBundle};
use amethyst::renderer::{
    AmbientColor, Camera, DisplayConfig, DrawShaded, Light, Material, MaterialDefaults, MeshData,
    MeshHandle, Pipeline, PointLight, PosNormTex, RenderBundle, Rgba, ScreenDimensions, Shape,
    Stage, Texture,
};
use amethyst::{Application, GameData, GameDataBuilder, SimpleState, StateData};
use nphysics_ecs_dumb::bodies::DynamicBody;
use nphysics_ecs_dumb::forces::DefaultForceGenerators;
use nphysics_ecs_dumb::nphysics::math::{Point, Velocity};
use nphysics_ecs_dumb::systems::PhysicsBundle;
use num_traits::identities::One;

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

        let camera_transform = Transform::from(Vector3::new(0.0, 0.0, 0.0));

        // Add Camera
        data.world
            .create_entity()
            .with(Camera::standard_3d(x, y))
            .with(camera_transform)
            .build();

        // Add Light
        data.world.add_resource(AmbientColor(Rgba::from([0.5; 3])));
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

        let sphere_shape = Shape::Sphere(32, 32).generate::<Vec<PosNormTex>>(None);
        let sphere_handle: MeshHandle = data.world.read_resource::<Loader>().load_from_data(
            sphere_shape,
            (),
            &data.world.read_resource(),
        );

        // Add Sphere (todo: add many, add rigidbodies and colliders)
        data.world
            .create_entity()
            .with(sphere_handle)
            .with(material)
            .with(Transform::from(Vector3::new(0.0, 0.0, -10.0)))
            .with(GlobalTransform::default())
            .with(DynamicBody::new_rigidbody_with_velocity(
                Velocity::linear(0.0, 10.0, 0.0),
                10.0,
                Matrix3::one(),
                Point::new(0.0, 0.0, 0.0),
            ))
            .build();
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
        vsync: true,
        multisampling: 0, // Must be multiple of 2, use 0 to disable
        visibility: true,
    };
    let pipe = Pipeline::build().with_stage(
        Stage::with_backbuffer()
            .clear_target([0.1, 0.1, 0.1, 1.0], 1.0)
            .with_pass(DrawShaded::<PosNormTex>::new()),
    );

    let game_data = GameDataBuilder::default()
        .with_bundle(TransformBundle::new())?
        .with_bundle(
            PhysicsBundle::<DefaultForceGenerators>::new().with_dep(&["transform_system"]),
        )?
        .with_bundle(RenderBundle::new(pipe, Some(display_config)))?;

    let application = Application::new("./", GameState, game_data);

    assert_eq!(application.is_ok(), true);

    application.ok().unwrap().run();

    Ok(())
}
