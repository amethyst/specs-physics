# Nphysics - Amethyst connector

Don't use. Work in progress. Many things are incomplete!

Currently specific to 3d Nphysics and Amethyst due to Amethyst handling of Transforms and to keep iteration quick,
although I currently plan on allowing 3d and 2d nphysics implementations to be used to configuration settings, and
both amethyst and plain specs interfaces to be exposed, also behind configuration settings.

## System Sequence

I'll update this as I go along.

1.
    - `"sync_force_generators_to_physics_system"` - Synchronize changes to force generators to physics world
    - `"sync_bodies_to_physics_system"` - Synchronize changes to dynamics bodies to physics world
    - `"sync_gravity_to_physics_system"` - Update gravity of physics world from resource
1. `"physics_stepper_system"` - Step physics world simulation
1. `"sync_bodies_from_physics_system"` - Synchronize physics world changes back to components


## Current Roadmap

Full *TODO* sheet can be found in [this nphysics issue](https://github.com/rustsim/nphysics/issues/149)

Ongoing work:

- [x] RigidBody Components
- [ ] Force Generator Components
- [ ] Collider Components
- [ ] Proximity and Contact EventChannels

Investigating:

- [ ] Multibody-based Component Joints
- [ ] Constraint-based Joints
- [ ] Kinematics
- [ ] Body Activation & Sleeping.
