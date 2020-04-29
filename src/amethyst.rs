use amethyst_core::{SystemBundle, Transform};
use amethyst_error::Error;
use specs::{DispatcherBuilder, World};

use crate::{nalgebra::Isometry3, register_physics_systems, Position};

impl Position<f32> for Transform {
    fn isometry(&self) -> &Isometry3<f32> {
        self.isometry()
    }

    fn isometry_mut(&mut self) -> &mut Isometry3<f32> {
        self.isometry_mut()
    }

    fn set_isometry(&mut self, isometry: &Isometry3<f32>) -> &mut Self {
        self.set_isometry(*isometry)
    }
}

pub struct PhysicsBundle;

impl<'a, 'b> SystemBundle<'a, 'b> for PhysicsBundle {
    fn build(
        self,
        _world: &mut World,
        dispatcher: &mut DispatcherBuilder<'a, 'b>,
    ) -> Result<(), Error> {
        register_physics_systems::<f32, Transform>(dispatcher);
        Ok(())
    }
}
