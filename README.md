# specs-physics

[![Build Status][bi]][bl] [![Crates.io][ci]][cl] ![MIT/Apache][li] [![Docs.rs][di]][dl]

[bi]: https://travis-ci.com/bamling/specs-physics.svg?branch=master
[bl]: https://travis-ci.com/bamling/specs-physics

[ci]: https://img.shields.io/crates/v/specs-physics.svg
[cl]: https://crates.io/crates/specs-physics/

[li]: https://img.shields.io/crates/l/specs-physics.svg

[di]: https://docs.rs/specs-physics/badge.svg
[dl]: https://docs.rs/specs-physics/

**specs-physics** aims to be an easily usable and extendable [nphysics](https://www.nphysics.org/) physics engine integration for applications and games that utilise the [Specs ECS](https://slide-rs.github.io/specs/).

The dream is to *simply* create `Entity`s with a set of configurable `Component`s and have most of your physics covered, be it collision/proximity detection, velocity and acceleration or gravity.


### Examples

Full examples can be found under [src/examples](https://github.com/bamling/specs-physics/tree/master/examples). If anything is missing or unclear, feel free to open an issue or give me a poke!

## Contributing 

I'd appreciate any kind of contribution to this project, be it feature requests, bugs/issues, pull requests, documentation, tests or examples! 

Please just try to format any code changes according to the [rustfmt.toml](https://github.com/bamling/specs-physics/blob/master/rustfmt.toml) rules. They're not exactly set in stone and I'm open for suggestions, but let's try to keep things tidy!

## Current Roadmap

Full *TODO* sheet can be found in [this nphysics issue][todo]

- [x] RigidBody Components
- [x] Collider Components
- [x] Proximity and Contact EventChannels
- [x] External force property
- [x] `log` based logging
- [ ] Handling Body Activation & Sleeping
- [ ] Multibody-based Component Joints
- [ ] Force generator inversion of control
- [ ] Time scale and simulation pausing

Investigating:

- [ ] Proximity & Curve-based external force utility
- [ ] Constraint-based Joints
- [ ] Kinematics

[todo]: https://github.com/rustsim/nphysics/issues/149

## License

Distributed under the MIT License. See [LICENSE](https://github.com/bamling/specs-physics/blob/master/LICENSE) for more information.

## Acknowledgments

This project is heavily inspired by [nphysics-ecs-dumb](https://github.com/distransient/nphysics-ecs-dumb); they did most of the heavy lifting, I'm just building up on what they have started!

**Special thanks to:**
- [distransient](https://github.com/distransient)
- [jojolepro](https://github.com/jojolepro)
