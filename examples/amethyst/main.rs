//! Demonstrates renderable objects with physics properties
use amethyst::{
    assets::{
        Completion, Handle, HotReloadBundle, Prefab, PrefabLoader, PrefabLoaderSystemDesc,
        ProgressCounter, RonFormat,
    },
    controls::{CursorHideSystemDesc, MouseFocusUpdateSystemDesc},
    core::transform::{Transform, TransformBundle},
    input::{
        is_close_requested, is_key_down, InputBundle, StringBindings,
        VirtualKeyCode,
    },
    prelude::*,
    renderer::{
        plugins::{RenderShaded3D, RenderToWindow},
        rendy::mesh::PosNormTex,
        types::DefaultBackend,
        RenderingBundle,
    },
    ui::{RenderUi, UiBundle, UiCreator, UiFinder},
    utils::application_root_dir,
    Error,
};
use specs_physics::{
    nphysics::math::Vector,
    systems::PhysicsBundle,
};

use crate::{
    prefab::CustomScenePrefab,
    systems::{CameraMovementSystemDesc, CameraRotationSystemDesc, CollisionDetectionSystemDesc},
};

type MyPrefabData = CustomScenePrefab<Vec<PosNormTex>>;

#[derive(Default)]
struct Loading {
    progress: ProgressCounter,
    prefab: Option<Handle<Prefab<MyPrefabData>>>,
}

struct Example {
    scene: Handle<Prefab<MyPrefabData>>,
}

impl SimpleState for Loading {
    fn on_start(&mut self, data: StateData<'_, GameData<'_, '_>>) {
        self.prefab = Some(data.world.exec(|loader: PrefabLoader<'_, MyPrefabData>| {
            loader.load("prefab/renderable.ron", RonFormat, &mut self.progress)
        }));

        data.world.exec(|mut creator: UiCreator<'_>| {
            creator.create("ui/loading.ron", &mut self.progress);
        });
    }

    fn update(&mut self, data: &mut StateData<'_, GameData<'_, '_>>) -> SimpleTrans {
        match self.progress.complete() {
            Completion::Failed => {
                println!("Failed loading assets: {:?}", self.progress.errors());
                Trans::Quit
            }
            Completion::Complete => {
                println!("Assets loaded, swapping state");
                if let Some(entity) = data
                    .world
                    .exec(|finder: UiFinder<'_>| finder.find("loading"))
                {
                    let _ = data.world.delete_entity(entity);
                }
                Trans::Switch(Box::new(Example {
                    scene: self.prefab.as_ref().unwrap().clone(),
                }))
            }
            Completion::Loading => Trans::None,
        }
    }
}

impl SimpleState for Example {
    fn on_start(&mut self, data: StateData<'_, GameData<'_, '_>>) {
        let StateData { world, .. } = data;

        world.create_entity().with(self.scene.clone()).build();
    }

    fn handle_event(
        &mut self,
        _data: StateData<'_, GameData<'_, '_>>,
        event: StateEvent,
    ) -> SimpleTrans {
        if let StateEvent::Window(event) = &event {
            // Exit if user hits Escape or closes the window
            if is_close_requested(&event) || is_key_down(&event, VirtualKeyCode::Escape) {
                return Trans::Quit;
            }
        }
        Trans::None
    }
}

fn main() -> Result<(), Error> {
    amethyst::start_logger(Default::default());

    let app_root = application_root_dir()?;

    // Add our meshes directory to the asset loader.
    let assets_dir = app_root.join("examples").join("amethyst").join("assets");

    let display_config_path = assets_dir
        .join("display.ron");

    let input_config_path = assets_dir
        .join("input_config.ron");

    let game_data = GameDataBuilder::default()
        .with_system_desc(PrefabLoaderSystemDesc::<MyPrefabData>::default(), "", &[])
        .with_bundle(TransformBundle::new())?
        .with_bundle(UiBundle::<StringBindings>::new())?
        .with_bundle(HotReloadBundle::default())?
        .with_bundle(InputBundle::<StringBindings>::new().with_bindings_from_file(&input_config_path)?)?
        .with_system_desc(CameraMovementSystemDesc::new(5.), "", &[])
        .with_system_desc(CameraRotationSystemDesc::new(0.25, 0.25), "", &[])
        .with_system_desc(CursorHideSystemDesc::default(), "", &[])
        .with_system_desc(MouseFocusUpdateSystemDesc::default(), "", &[])
        .with_bundle(PhysicsBundle::<f32, Transform>::new(Vector::y() * -9.81))?
        .with_system_desc(CollisionDetectionSystemDesc::default(), "", &[])
        .with_bundle(
            RenderingBundle::<DefaultBackend>::new()
                .with_plugin(
                    RenderToWindow::from_config_path(display_config_path)?
                        .with_clear([0.34, 0.36, 0.52, 1.0]),
                )
                .with_plugin(RenderShaded3D::default())
                .with_plugin(RenderUi::default()),
        )?;
    let mut game = Application::build(assets_dir, Loading::default())?.build(game_data)?;
    game.run();
    Ok(())
}

mod prefab;
mod systems;