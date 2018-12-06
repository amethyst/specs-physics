extern crate amethyst;
pub extern crate ncollide3d as ncollide;
pub extern crate nphysics3d as nphysics;
extern crate num_traits;

pub mod bodies;
pub mod forces;
pub mod systems;

pub type World = self::nphysics::world::World<f32>;
pub type Gravity = self::nphysics::math::Vector<f32>;
