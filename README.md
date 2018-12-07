# Nphysics - Amethyst connector

Don't use. Work in progress. Many things are incomplete!

Currently specific to 3d Nphysics and Amethyst due to Amethyst handling of Transforms and to keep iteration quick,
although I currently plan on allowing 3d and 2d nphysics implementations to be used to configuration settings, and
both amethyst and plain specs interfaces to be exposed, also behind configuration settings.

## System Sequence

I'll update this as I go along.

1. `SyncBodiestoPhysicsSystem` - Apply FlaggedStorage changes to nphysics world
1. `PhysicsStepperSystem` - Step physics simulation
1. `SyncSynchronize changes back to components from nphysics world


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
