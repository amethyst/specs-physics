use crate::{Gravity, PhysicsWorld};
use amethyst::ecs::{ReadExpect, Resources, System, SystemData, WriteExpect};

#[derive(Default)]
pub struct SyncGravityToPhysicsSystem;

impl SyncGravityToPhysicsSystem {
    pub fn new() -> Self {
        Default::default()
    }
}

impl<'a> System<'a> for SyncGravityToPhysicsSystem {
    type SystemData = (WriteExpect<'a, PhysicsWorld>, ReadExpect<'a, Gravity>);

    fn run(&mut self, (mut world, gravity): Self::SystemData) {
        world.set_gravity(*gravity);
    }

    fn setup(&mut self, res: &mut Resources) {
        Self::SystemData::setup(res);

        res.entry::<Gravity>()
            .or_insert_with(|| Gravity::new(0.0, -9.80665, 0.0));

        res.entry::<PhysicsWorld>().or_insert_with(PhysicsWorld::new);
    }
}
