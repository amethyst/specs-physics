use crate::bodies::DynamicsBodyRelations;
use amethyst::core::approx::AbsDiffEq;
use amethyst::ecs::{BitSet, Component, FlaggedStorage};
use nalgebra::Unit;
use nphysics3d::force_generator::ForceGenerator;
use nphysics3d::math::{Force, Point, Vector, Velocity};
use nphysics3d::object::{BodyHandle, BodySet};
use nphysics3d::solver::IntegrationParameters;

pub trait ForceGenerators {
    type LocalForceGenerators: LocalForceGenerators;
    type LinkedForceGenerators: LinkedForceGenerators;
}

pub trait LocalForceGenerators:
    ForceGenerator<f32> + Component<Storage = FlaggedStorage<Self>>
{
}

pub trait LinkedForceGenerators:
    ForceGenerator<f32> + Component<Storage = FlaggedStorage<Self>>
{
    fn affected_bodies(&self) -> DynamicsBodyRelations; // TODO this signature is definitely wrong.
}

pub struct DefaultForceGenerators;

impl ForceGenerators for DefaultForceGenerators {
    type LocalForceGenerators = DefaultLocalForceGenerators;
    type LinkedForceGenerators = DefaultLinkedForceGenerators;
}

pub enum DefaultLocalForceGenerators {
    ConstantAcceleration(ConstantAcceleration),
}

pub struct ConstantAcceleration {
    pub acceleration: Velocity<f32>,
}

impl Component for DefaultLocalForceGenerators {
    type Storage = FlaggedStorage<Self>;
}

impl ForceGenerator<f32> for DefaultLocalForceGenerators {
    fn apply(&mut self, _: &IntegrationParameters<f32>, _bodies: &mut BodySet<f32>) -> bool {
        false
    }
}

impl LocalForceGenerators for DefaultLocalForceGenerators {}

pub enum DefaultLinkedForceGenerators {
    Spring(Spring),
}

pub struct Spring {
    pub length: f32,
    pub stiffness: f32,
    pub anchor1: Point<f32>,
    pub anchor2: Point<f32>,
    pub handle1: BodyHandle,
    pub handle2: BodyHandle,
    pub links: BitSet,
}

impl Component for DefaultLinkedForceGenerators {
    type Storage = FlaggedStorage<Self>;
}

impl LinkedForceGenerators for DefaultLinkedForceGenerators {
    fn affected_bodies(&self) -> DynamicsBodyRelations {
        DynamicsBodyRelations::new()
        // match self {
        //    DefaultLinkedForceGenerators::Spring(x) => x.links
        // }
    }
}

impl ForceGenerator<f32> for DefaultLinkedForceGenerators {
    fn apply(&mut self, _: &IntegrationParameters<f32>, bodies: &mut BodySet<f32>) -> bool {
        match self {
            DefaultLinkedForceGenerators::Spring(spring) => {
                if !bodies.contains(spring.handle1) || !bodies.contains(spring.handle2) {
                    return false;
                }

                let anchor1 = bodies.body_part(spring.handle1).position() * spring.anchor1;
                let anchor2 = bodies.body_part(spring.handle2).position() * spring.anchor2;

                let force_dir;
                let delta_length;

                if let Some((dir, length)) =
                    Unit::try_new_and_get(anchor2 - anchor1, f32::default_epsilon())
                {
                    force_dir = dir;
                    delta_length = length - spring.length;
                } else {
                    force_dir = Vector::y_axis();
                    delta_length = -spring.length;
                }

                let force = Force::linear(force_dir.as_ref() * delta_length * spring.stiffness);
                bodies.body_part_mut(spring.handle1).apply_force(&force);
                bodies.body_part_mut(spring.handle2).apply_force(&-force);

                true
            }
        }
    }
}
