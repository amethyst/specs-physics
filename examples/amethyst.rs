use amethyst::{
    assets::Loader,
    core::*,
    ecs::prelude::*,
    input::{Axis, Bindings, Button, InputBundle, InputHandler, StringBindings, VirtualKeyCode},
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
    PhysicsBody, PhysicsBodyBuilder, PhysicsColliderBuilder,
};

fn create_object(world: &mut World, status: BodyStatus, shape: Shape<Float>, translation: [f32; 3], velocity: [f32; 3]) -> EntityBuilder {
    let mut transform = Transform::default();

    transform.append_translation(Vector3::<Float>::new(
        translation[0].into(),
        translation[1].into(),
        translation[2].into(),
    ));

    world
        .create_entity()
        .with(transform)
        .with(PhysicsBodyBuilder::<Float>::from(status)
            .velocity(Velocity3::linear(velocity[0].into(), velocity[1].into(), velocity[2].into()))
            .build())
        .with(PhysicsColliderBuilder::<Float>::from(shape).build())
}

#[derive(Copy, Clone, Debug, Default)]
pub struct Toy;

impl Component for Toy {
    type Storage = NullStorage<Self>;
}

const TOY_ACCELERATION_MULTIPLIER: Float = Float::from_f64(1.0);

#[derive(Default)]
struct ToyControllerSystem;

impl<'a> System<'a> for ToyControllerSystem {
    type SystemData = (
        Read<'a, InputHandler<StringBindings>>,
        WriteStorage<'a, PhysicsBody<Float>>,
        ReadStorage<'a, Toy>,
    );

    fn run(&mut self, data: Self::SystemData) {
        let (input, mut physics_bodies, toys) = data;

        let delta = Velocity3::<Float>::new(
            Vector3::<Float>::new(
                Float::from(input.axis_value("x").unwrap()) * TOY_ACCELERATION_MULTIPLIER,
                Float::from(input.axis_value("y").unwrap()) * TOY_ACCELERATION_MULTIPLIER,
                0.0.into(),
            ),
            Vector3::<Float>::zeros(),
        );

        for (body, _) in (&mut physics_bodies, &toys).join() {
            body.velocity += delta;
        }
    }
}

#[derive(Default)]
struct State<'a, 'b> {
    fixed_dispatcher: Option<Dispatcher<'a, 'b>>,
}

impl<'a, 'b> SimpleState for State<'a, 'b> {
    fn on_start(&mut self, data: StateData<GameData>) {
        let mut builder =
            DispatcherBuilder::new().with_pool(data.world.res.fetch::<ArcThreadPool>().clone());

        // Adds an input bundle with "x" being A/D and "y" being W/S
        InputBundle::new()
            .with_bindings(bindings())
            .build(&mut builder)
            .unwrap();
        builder.add(ToyControllerSystem::default(), "toy_controller_system", &[]);
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
                    Float::from_f32(0.0),
                    0.0.into(),
                    Float::from_f32(10.0),
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

        create_object(data.world, 
            BodyStatus::Kinematic, 
            Shape::<Float>::Cuboid {
                half_extents: Vector3::<Float>::new(0.5.into(), 0.5.into(), 0.5.into()),
            }, 
            [0.1, 1.3, 0.1], 
            [0.0, 1.0, 0.0])
            .with(material.clone())
            .with(cube.clone())
            .with(Toy::default())
            .build();

        let sphere = AmethystShape::Sphere(32, 32)
            .upload::<(Vec<Position>, Vec<Normal>, Vec<TexCoord>), _>(
                Some((1.0, 1.0, 1.0)),
                ShapeUpload::fetch(&data.world.res),
                (),
            );

        create_object(data.world, 
            BodyStatus::Dynamic, 
            Shape::<Float>::Ball { radius: 0.5.into() }, 
            [-0.1, 3.3, -0.1], 
            [0.0, 0.0, 0.0])
            .with(material.clone())
            .with(sphere.clone())
            .build();

        create_object(data.world, 
            BodyStatus::Static, 
            Shape::<Float>::Ball { radius: 0.5.into() }, 
            [0.0, 0.0, 0.0], 
            [0.0, 0.0, 0.0])
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

    // Our systems for simulation are being added to a fixed dispatcher in our State's on_start.
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

fn bindings() -> Bindings<StringBindings> {
    let mut bindings = Bindings::<StringBindings>::new();

    bindings
        .insert_axis(
            "x",
            Axis::Emulated {
                pos: Button::Key(VirtualKeyCode::D),
                neg: Button::Key(VirtualKeyCode::A),
            },
        )
        .unwrap();

    bindings
        .insert_axis(
            "y",
            Axis::Emulated {
                pos: Button::Key(VirtualKeyCode::W),
                neg: Button::Key(VirtualKeyCode::S),
            },
        )
        .unwrap();

    bindings
}
