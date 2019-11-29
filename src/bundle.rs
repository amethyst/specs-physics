use crate::{
    nalgebra::{convert as na_convert, RealField},
    nphysics::math::Vector,
    pose::Pose,
    stepper::StepperRes,
    systems::{PhysicsPoseSystem, PhysicsStepperSystem},
    ForceGeneratorSetRes, GeometricalWorldRes, MechanicalWorldRes,
};
use specs::{DispatcherBuilder, World};
use std::marker::PhantomData;

/// Bundle used to construct and insert all specs-physics Systems and Resources
/// into a Dispatcher and its World.
#[must_use]
pub struct PhysicsBundle<N: RealField, P: Pose<N>> {
    mechanical_world: MechanicalWorldRes<N>,
    geometrical_world: GeometricalWorldRes<N>,
    stepper_res: Option<StepperRes>,
    // Exercising superfluous allocations at init-time
    // is better than figuring out the lifetimes
    // for the slice version of this at programming-time.
    stepper_deps: Vec<Box<str>>,
    marker: PhantomData<P>,
}

impl<N: RealField, P: Pose<N>> PhysicsBundle<N, P> {
    /// Constructs a new physics bundle with a given gravity vector and system
    /// dependencies for [`PhysicsStepperSystem`]. Omits data for the
    /// [`PhysicsBatchSystem`] stepper.
    ///
    /// [`PhysicsBatchSystem`]: ../systems/system.PhysicsBatchSystem.html
    pub fn new(gravity: Vector<N>, dep: &[&str]) -> Self {
        Self::from_parts(
            MechanicalWorldRes::<N>::new(gravity),
            GeometricalWorldRes::<N>::new(),
            None,
            dep,
        )
    }

    /// For fine-grained control over initialization, constructs a bundle given
    /// - `mechanical_world`, for configuring the mechanics of the simulation
    /// - `geometrical_world`, for configuring collision
    /// - optionally `stepper_res`, for configuring [`PhysicsBatchSystem`] if
    ///   you're using it.
    /// - `dep`, to configure which systems should execute before the
    ///   [`PhysicsStepperSystem`].
    ///
    /// [`PhysicsBatchSystem`]: ../systems/system.PhysicsBatchSystem.html
    pub fn from_parts(
        mechanical_world: MechanicalWorldRes<N>,
        geometrical_world: GeometricalWorldRes<N>,
        stepper_res: Option<StepperRes>,
        dep: &[&str],
    ) -> Self {
        Self {
            mechanical_world,
            geometrical_world,
            stepper_res,
            stepper_deps: dep.iter().map(|s| Box::from(*s)).collect(),
            marker: PhantomData,
        }
    }

    /// *Adds* to the dependency list configuring which systems should execute
    /// before the [`PhysicsStepperSystem`].
    ///
    /// [`PhysicsBatchSystem`]: ../systems/system.PhysicsBatchSystem.html
    pub fn with_deps(mut self, dep: &[&str]) -> Self {
        self.stepper_deps = [
            self.stepper_deps
                .iter()
                .map(|s| s.as_ref())
                .collect::<Vec<&str>>()
                .as_slice(),
            dep,
        ]
        .concat()
        .iter()
        .map(|s| Box::from(*s))
        .collect();
        self
    }

    /// Adds fixed stepper [`StepperRes`] data for [`PhysicsBatchSystem`] at
    /// `interval` hz
    ///
    /// [`PhysicsBatchSystem`]: ../systems/system.PhysicsBatchSystem.html
    pub fn with_fixed_stepper(mut self, interval: u32) -> Self {
        self.stepper_res = Some(StepperRes::new_fixed(interval));
        self
    }

    /// Adds fixed stepper [`StepperRes`] data for [`PhysicsBatchSystem`].
    ///
    /// [`PhysicsBatchSystem`]: ../systems/system.PhysicsBatchSystem.html
    pub fn with_stepper_instance(mut self, stepper: StepperRes) -> Self {
        self.stepper_res = Some(stepper);
        self
    }

    /// Registers this bundle data to a `world` and dispatcher `builder`.
    pub fn register(self, world: &mut World, builder: &mut DispatcherBuilder) {
        world.insert(self.mechanical_world);
        world.insert(self.geometrical_world);

        if let Some(stepper_res) = self.stepper_res {
            world.insert(stepper_res);
        }

        world.insert(ForceGeneratorSetRes::<N>::new());

        // Add PhysicsStepperSystem after all other Systems that write data to the
        // nphysics World and has to depend on them; this System is used to progress the
        // nphysics World for all existing objects.
        builder.add(
            PhysicsStepperSystem::<N>::default(),
            "physics_stepper_system",
            self.stepper_deps
                .iter()
                .map(|s| s.as_ref())
                .collect::<Vec<&str>>()
                .as_slice(),
        );

        // Add PhysicsPoseSystem last as it handles the
        // synchronisation between nphysics World bodies and the Position
        // components; this depends on the PhysicsStepperSystem.
        builder.add(
            PhysicsPoseSystem::<N, P>::default(),
            "physics_pose_system",
            &["physics_stepper_system"],
        );
    }
}

#[cfg(feature = "amethyst")]
impl<'a, 'b, N: RealField, P: Pose<N>> amethyst::core::SystemBundle<'a, 'b>
    for PhysicsBundle<N, P>
{
    fn build(
        self,
        world: &mut World,
        builder: &mut DispatcherBuilder,
    ) -> Result<(), amethyst::error::Error> {
        Ok(self.register(world, builder))
    }
}

impl<N: RealField, P: Pose<N>> Default for PhysicsBundle<N, P> {
    fn default() -> Self {
        Self::new(Vector::<N>::y() * na_convert::<f64, N>(-9.81), &[])
    }
}
