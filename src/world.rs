use crate::{
    nphysics::{
        DefaultGeometricalWorld, DefaultMechanicalWorld,
    }
};

#[derive(Shrinkwrap)]
#[shrinkwrap(mutable)]
pub struct MechanicalWorld<N: RealField>(pub DefaultMechanicalWorld<N>);

#[derive(Shrinkwrap)]
#[shrinkwrap(mutable)]
pub struct GeometricalWorld<N: RealField>(pub DefaultGeometricalWorld<N>);

/// Resource holding the internal fields where physics computation occurs.
/// Some inspection methods are exposed to allow debugging.
pub struct PhysicsWorld<N: RealField> {
    /// Core structure where physics computation and synchronization occurs.
    /// Also contains ColliderWorld.
    pub(crate) mechanical_world: DefaultMechanicalWorld<N>,
    pub(crate) geometrical_world: DefaultGeometricalWorld<N>

    /// Hashmap of Entities to internal Physics bodies.
    /// Necessary for reacting to removed Components.
    pub(crate) body_handles: HashMap<Index, BodyHandle>,
    /// Hashmap of Entities to internal Collider handles.
    /// Necessary for reacting to removed Components.
    pub(crate) collider_handles: HashMap<Index, ColliderHandle>,
}

// Some non-mutating methods for diagnostics and testing
impl<N: RealField> PhysicsData<N> {
    /// Creates a new instance of the physics structure.
    pub fn new() -> Self {
        Self::default()
    }

    /// Reports the internal value for the timestep.
    /// See also `TimeStep` for setting this value.
    pub fn timestep(&self) -> N {
        self.mechanical_world.timestep()
    }

    /// Reports the internal value for the gravity.
    /// See also `Gravity` for setting this value.
    pub fn gravity(&self) -> &Vector<N> {
        self.mechanical_world.gravity()
    }

    /// Reports the internal value for prediction distance in collision
    /// detection. This cannot change and will normally be `0.002m`
    pub fn prediction(&self) -> N {
        self.mechanical_world.prediction()
    }

    /// Retrieves the performance statistics for the last simulated timestep.
    /// Profiling is disabled by default.
    /// See also `PhysicsProfilingEnabled` for enabling performance counters.
    pub fn performance_counters(&self) -> &Counters {
        self.mechanical_world.performance_counters()
    }

    /// Retrieves the internal parameters for integration.
    /// See also `PhysicsIntegrationParameters` for setting these parameters.
    pub fn integration_parameters(&self) -> &IntegrationParameters<N> {
        self.mechanical_world.integration_parameters()
    }

    /// Retrieves the internal lookup table for friction and restitution
    /// constants. Exposing this for modification is TODO.
    pub fn materials_coefficients_table(&self) -> &MaterialsCoefficientsTable<N> {
        self.mechanical_world.materials_coefficients_table()
    }
}

impl<N: RealField> Default for Physics<N> {
    fn default() -> Self {
        Self {
            mechanical_world: DefaultMechanicalWorld::new(Vector::zeros()),
            body_handles: HashMap::new(),
            collider_handles: HashMap::new(),
        }
    }
}