# Nphysics - Amethyst connector

Don't use. Work in progress. Many things are incomplete!

Currently specific to 3d Nphysics and Amethyst due to Amethyst handling of Transforms and to keep iteration quick,
although I currently plan on allowing 3d and 2d nphysics implementations to be used to configuration settings, and
both amethyst and plain specs interfaces to be exposed, also behind configuration settings.

## System Sequence

I'll update this as I go along.

1.
    - `"sync_bodies_to_physics_system"` - Synchronize changes to dynamics bodies to physics world
    - `"sync_gravity_to_physics_system"` - Update gravity of physics world from resource
1. `"sync_colliders_to_physics_system"` - Synchronize collision items to physics world
1. `"physics_stepper_system"` - Step physics world simulation
1. `"sync_bodies_from_physics_system"` - Synchronize physics world changes back to components


## Current Roadmap

Full *TODO* sheet can be found in [this nphysics issue][todo]

Ongoing work:

- [x] RigidBody Components
- [x] Collider Components [#2]
- [x] Proximity and Contact EventChannels [#2]
- [x] External force property [#3]
- [x] `log` based logging [#4]
- [ ] Handling Body Activation & Sleeping [#9]
- [ ] Multibody-based Component Joints [#10]
- [ ] Force generator inversion of control [#11]
- [ ] Time scale and simulation pausing [#12]

Investigating:

- [ ] Proximity & Curve-based external force utility
- [ ] Constraint-based Joints
- [ ] Kinematics

[todo]: https://github.com/rustsim/nphysics/issues/149
[#2]: https://github.com/distransient/nphysics-ecs-dumb/pull/2
[#3]: https://github.com/distransient/nphysics-ecs-dumb/pull/3
[#4]: https://github.com/distransient/nphysics-ecs-dumb/pull/4
[#9]: https://github.com/distransient/nphysics-ecs-dumb/issues/9
[#10]: https://github.com/distransient/nphysics-ecs-dumb/issues/10
[#11]: https://github.com/distransient/nphysics-ecs-dumb/issues/11
[#12]: https://github.com/distransient/nphysics-ecs-dumb/issues/12
