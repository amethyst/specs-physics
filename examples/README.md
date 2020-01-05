# Specs-Physics Examples

Read the source code of these files to get a feeling 
of how to perform common operations through the interface of specs-physics. 

It's important to also read through the [nphysics guide][] 
so you know what's happening in nphysics as well. 
Wherever there is a BodySet or a ColliderSet, instead think about how you're using 
[`BodyComponent`][] and [`ColliderComponent`][] Storages 

[nphysics guide]: https://nphysics.org/
[`BodyComponent`]: https://
[`ColliderComponent`]: https://

## Basic Usage (`basic.rs`)

Explains basic general use of specs-physics, based on the nphysics [balls3.rs][] example
which you can view in the nphysics testbed [here][balls3 testbed].


To run this example, execute the following command from the specs-physics directory:

```bash
cargo run --example basic
```

[balls3.rs]: https://github.com/rustsim/nphysics/blob/master/examples3d/balls3.rs
[balls3 testbed]: https://www.nphysics.org/demo_all_examples3/

## Batch Dispatching (`batch.rs`)

Demonstrates usage of the fixed batch dispatcher, 
which executes your physics code that depends on a fixed timestep, in that timestep.
Read the documentation of [`PhysicsBatchSystem`][] and the [`stepper`][] module 
to better understand how this works.

To run this example, execute the following command from the specs-physics directory:

```bash
cargo run --example batch
```

[`PhysicsBatchSystem`]: https://
[`stepper`]: https://