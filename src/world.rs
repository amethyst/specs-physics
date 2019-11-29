//! Resource type aliases for accessing the geometrical and mechanical worlds in
//! *Specs*.

use crate::nphysics::{
    force_generator::DefaultForceGeneratorSet,
    world::{GeometricalWorld, MechanicalWorld},
};

use specs::Entity;

/**
The [`MechanicalWorld`] type stored as a Resource.

You can fetch this type in a system with `ReadExpect<'_, MechanicalWorldRes<N>>` or
`WriteExpect<'_, MechanicalWorldRes<N>>`.

# Usage

```
# use specs::{DispatcherBuilder, World, WorldExt};
# use specs_physics::{MechanicalWorldRes, PhysicsBundle, SimplePosition};
# let mut world = World::new();
# let mut builder = DispatcherBuilder::new();
# PhysicsBundle::<f32, SimplePosition<f32>>::default().register(&mut world, &mut builder);
# builder.build().setup(&mut world);
let mechanical_world = world.fetch::<MechanicalWorldRes<f32>>();

assert!(mechanical_world.timestep() > 0.0)
```
*/
pub type MechanicalWorldRes<N> = MechanicalWorld<N, Entity, Entity>;

/**
The [`GeometricalWorld`] type stored as a Resource.

You can fetch this type in a system with `ReadExpect<'_, GeometricalWorldRes<f32>>` or
`WriteExpect<'_, GeometricalWorldRes<f32>>`.

# Usage

```
# use specs::{DispatcherBuilder, World, WorldExt};
# use specs_physics::{GeometricalWorldRes, PhysicsBundle, SimplePosition};
# let mut world = World::new();
# let mut builder = DispatcherBuilder::new();
# PhysicsBundle::<f32, SimplePosition<f32>>::default().register(&mut world, &mut builder);
# builder.build().setup(&mut world);
let geometrical_world = world.fetch::<GeometricalWorldRes<f32>>();

assert!(geometrical_world.contact_events().len() >= 0)
```
*/
pub type GeometricalWorldRes<N> = GeometricalWorld<N, Entity, Entity>;

/**
The [`DefaultForceGeneratorSet`] type stored as a Resource. (Not Recommended)

Force generators are only sugar for executing a series of actions over the bodyset on each step,
and aren't executed on substeps. Therefor in specs usage it's probably better to just excute
force generation systems per step like your other physics systems instead of dealing with the
force generator set.

# Usage

```
# use specs::{DispatcherBuilder, World, WorldExt};
# use specs_physics::{ForceGeneratorSetRes, PhysicsBundle, SimplePosition};
# let mut world = World::new();
# let mut builder = DispatcherBuilder::new();
# PhysicsBundle::<f32, SimplePosition<f32>>::default().register(&mut world, &mut builder);
# builder.build().setup(&mut world);
let force_gen_set = world.fetch::<ForceGeneratorSetRes<f32>>();

assert!(force_gen_set.len() >= 0)
```
*/
pub type ForceGeneratorSetRes<N> = DefaultForceGeneratorSet<N, Entity>;
