use amethyst::{
    assets::Loader,
    core::*,
    ecs::prelude::*,
    prelude::*,
    renderer::{
        formats::texture::TextureGenerator,
        light::{Light, SunLight},
        mtl::{Material, TextureOffset},
        palette::rgb::Srgb,
        pass::DrawShadedDesc,
        rendy::{
            factory::Factory,
            graph::{
                render::{RenderGroupDesc, SubpassBuilder},
                GraphBuilder,
            },
            hal::{format::Format, image},
            mesh::{Normal, Position, TexCoord},
        },
        shape::{Shape as AmethystShape, ShapeUpload},
        types::DefaultBackend,
        Camera, GraphCreator, RenderingSystem, Texture,
    },
    utils::application_root_dir,
    window::{DisplayConfig, ScreenDimensions, Window, WindowBundle},
};

use specs_physics::{
    colliders::Shape,
    nalgebra::Vector3,
    nphysics::{algebra::Velocity3, object::BodyStatus},
    parameters::Gravity,
    PhysicsBodyBuilder, PhysicsColliderBuilder,
};

#[derive(Default)]
struct State<'a, 'b> {
    fixed_dispatcher: Option<Dispatcher<'a, 'b>>,
}

impl<'a, 'b> SimpleState for State<'a, 'b> {
    fn on_start(&mut self, data: StateData<GameData>) {
        let mut builder =
            DispatcherBuilder::new().with_pool(data.world.res.fetch::<ArcThreadPool>().clone());

        TransformBundle::new().build(&mut builder).unwrap();
        specs_physics::register_physics_systems::<Float, Transform>(&mut builder);

        let mut dispatcher = builder.build();
        dispatcher.setup(&mut data.world.res);
        self.fixed_dispatcher = Some(dispatcher);

        data.world
            .add_resource(Gravity::<Float>(Vector3::<Float>::new(
                0.0.into(),
                Float::from_f32(-9.8),
                0.0.into(),
            )));

        let (x, y) = {
            let resolution = data.world.res.fetch::<ScreenDimensions>();
            (resolution.width(), resolution.height())
        };

        data.world
            .create_entity()
            .with(Camera::standard_3d(x, y))
            .with({
                let mut transform = Transform::default();
                transform.append_translation(Vector3::<Float>::new(
                    0.0.into(),
                    0.0.into(),
                    Float::from_f32(-4.0),
                ));
                transform
            })
            .build();

        data.world
            .create_entity()
            .with(Light::Sun(SunLight {
                angle: 0.01,
                color: Srgb::new(1.0, 1.0, 1.0),
                direction: Vector3::new(0.09, -0.95, 0.29),
                intensity: 10_000.0,
            }))
            .with({
                let mut transform = Transform::default();
                transform.append_translation(Vector3::<Float>::new(
                    0.0.into(),
                    100.0.into(),
                    0.0.into(),
                ));
                transform
            })
            .build();

        let material = {
            let loader = &data.world.read_resource::<Loader>();
            let orange_texture = loader.load_from_data::<Texture, ()>(
                TextureGenerator::Srgba(255.0, 64.0, 0.0, 1.0).data(),
                (),
                &data.world.read_resource(),
            );
            let black_texture = loader.load_from_data::<Texture, ()>(
                TextureGenerator::Srgba(0.0, 0.0, 0.0, 1.0).data(),
                (),
                &data.world.read_resource(),
            );
            loader.load_from_data::<Material, ()>(
                Material {
                    alpha_cutoff: 0.0,
                    albedo: orange_texture.clone(),
                    emission: black_texture.clone(),
                    normal: black_texture.clone(),
                    metallic_roughness: black_texture.clone(),
                    ambient_occlusion: black_texture.clone(),
                    cavity: black_texture.clone(),
                    uv_offset: TextureOffset::default(),
                },
                (),
                &data.world.read_resource(),
            )
        };

        let cube = AmethystShape::Cube.upload::<(Vec<Position>, Vec<Normal>, Vec<TexCoord>), _>(
            Some((1.0, 1.0, 1.0)),
            ShapeUpload::fetch(&data.world.res),
            (),
        );

        data.world
            .create_entity()
            .with({
                let mut transform = Transform::default();
                transform.append_translation(Vector3::<Float>::new(
                    0.1.into(),
                    1.3.into(),
                    0.1.into(),
                ));
                transform
            })
            .with(
                PhysicsBodyBuilder::<Float>::from(BodyStatus::Dynamic)
                    .velocity(Velocity3::linear(0.0.into(), 1.0.into(), 0.0.into()))
                    .build(),
            )
            .with(
                PhysicsColliderBuilder::<Float>::from(Shape::<Float>::Cuboid {
                    half_extents: Vector3::<Float>::new(0.5.into(), 0.5.into(), 0.5.into()),
                })
                .build(),
            )
            .with(material.clone())
            .with(cube.clone())
            .build();

        let sphere = AmethystShape::Sphere(32, 32)
            .upload::<(Vec<Position>, Vec<Normal>, Vec<TexCoord>), _>(
                Some((1.0, 1.0, 1.0)),
                ShapeUpload::fetch(&data.world.res),
                (),
            );

        data.world
            .create_entity()
            .with(Transform::default())
            .with(PhysicsBodyBuilder::<Float>::from(BodyStatus::Static).build())
            .with(
                PhysicsColliderBuilder::<Float>::from(Shape::<Float>::Ball { radius: 0.5.into() })
                    .build(),
            )
            .with(material.clone())
            .with(sphere.clone())
            .build();

        let plane = AmethystShape::Cube.upload::<(Vec<Position>, Vec<Normal>, Vec<TexCoord>), _>(
            Some((1000.0, 0.0001, 1000.0)),
            ShapeUpload::fetch(&data.world.res),
            (),
        );

        data.world
            .create_entity()
            .with({
                let mut transform = Transform::default();
                transform.append_translation(Vector3::<Float>::new(
                    0.0.into(),
                    Float::from_f32(-2.0),
                    0.0.into(),
                ));
                transform
            })
            .with(
                PhysicsColliderBuilder::<Float>::from(Shape::<Float>::Plane {
                    normal: Vector3::<Float>::y_axis(),
                })
                .build(),
            )
            .with(material.clone())
            .with(plane.clone())
            .build();
    }

    fn fixed_update(&mut self, data: StateData<GameData>) -> SimpleTrans {
        if let Some(dispatcher) = &mut self.fixed_dispatcher {
            dispatcher.dispatch(&data.world.res);
        }
        Trans::None
    }
}

fn main() -> amethyst::Result<()> {
    amethyst::start_logger(Default::default());

    let game_data = GameDataBuilder::default()
        .with_bundle(WindowBundle::from_config(DisplayConfig::default()))?
        .with_thread_local(RenderingSystem::<DefaultBackend, _>::new(
            ExampleGraph::default(),
        ));
    let mut game = Application::new(application_root_dir()?, State::default(), game_data)?;
    game.run();
    Ok(())
}

#[derive(Default)]
struct ExampleGraph {
    dimensions: Option<ScreenDimensions>,
    surface_format: Option<Format>,
    dirty: bool,
}

impl GraphCreator<DefaultBackend> for ExampleGraph {
    fn rebuild(&mut self, res: &Resources) -> bool {
        // Rebuild when dimensions change, but wait until at least two frames have the same.
        let new_dimensions = res.try_fetch::<ScreenDimensions>();
        use std::ops::Deref;
        if self.dimensions.as_ref() != new_dimensions.as_ref().map(|d| d.deref()) {
            self.dirty = true;
            self.dimensions = new_dimensions.map(|d| d.clone());
            return false;
        }
        return self.dirty;
    }

    fn builder(
        &mut self,
        factory: &mut Factory<DefaultBackend>,
        res: &Resources,
    ) -> GraphBuilder<DefaultBackend, Resources> {
        use amethyst::renderer::rendy::{
            graph::present::PresentNode,
            hal::command::{ClearDepthStencil, ClearValue},
        };

        self.dirty = false;
        let window = <ReadExpect<'_, Window>>::fetch(res);
        let surface = factory.create_surface(&window);
        // cache surface format to speed things up
        let surface_format = *self
            .surface_format
            .get_or_insert_with(|| factory.get_surface_format(&surface));
        let dimensions = self.dimensions.as_ref().unwrap();
        let window_kind =
            image::Kind::D2(dimensions.width() as u32, dimensions.height() as u32, 1, 1);

        let mut graph_builder = GraphBuilder::new();
        let color = graph_builder.create_image(
            window_kind,
            1,
            surface_format,
            Some(ClearValue::Color([0.34, 0.36, 0.52, 1.0].into())),
        );

        let depth = graph_builder.create_image(
            window_kind,
            1,
            Format::D32Sfloat,
            Some(ClearValue::DepthStencil(ClearDepthStencil(1.0, 0))),
        );

        let opaque = graph_builder.add_node(
            SubpassBuilder::new()
                .with_group(DrawShadedDesc::new().builder())
                .with_color(color)
                .with_depth_stencil(depth)
                .into_pass(),
        );

        let _present = graph_builder
            .add_node(PresentNode::builder(factory, surface, color).with_dependency(opaque));

        graph_builder
    }
}
