/*!
Markers used to simplify join operations
*/

use crate::{
    bodies::BodyComponent,
    nalgebra::RealField,
    nphysics::object::{Body, Ground, Multibody, RigidBody},
};

use specs::{
    hibitset::BitSetAnd,
    shred::{Fetch, FetchMut, MetaTable, ResourceId},
    storage::{AnyStorage, MaskedStorage, TryDefault, UnprotectedStorage},
    BitSet, Component, Join, NullStorage, ReadStorage, Storage, SystemData, World, WorldExt,
};

use std::{
    marker::PhantomData,
    ops::{Deref, DerefMut},
};

/// Component that marks the Body on this Entity as a RigidBody.
/// Do not attach more than one kind of Body Marker type to a Body entity.
#[derive(Default, Copy, Clone, Debug)]
pub struct RigidBodyMarker;

impl Component for RigidBodyMarker {
    type Storage = NullStorage<Self>;
}

/// Put this type in your SystemData to get a storage which joins immutably over
/// RigidBodies.
pub type ReadRigidBodies<'f, N> = BodyMarkerStorage<
    'f,
    N,
    RigidBody<N>,
    RigidBodyMarker,
    Fetch<'f, MaskedStorage<BodyComponent<N>>>,
>;

/// Put this type in your SystemData to get a storage which joins mutably over
/// RigidBodies.
pub type WriteRigidBodies<'f, N> = BodyMarkerStorage<
    'f,
    N,
    RigidBody<N>,
    RigidBodyMarker,
    FetchMut<'f, MaskedStorage<BodyComponent<N>>>,
>;

/// Component that marks the Body on this Entity as a Multibody.
/// Do not attach more than one kind of Body Marker type to a Body entity.
#[derive(Default, Copy, Clone, Debug)]
pub struct MultibodyMarker;

impl Component for MultibodyMarker {
    type Storage = NullStorage<Self>;
}

/// Put this type in your SystemData to get a storage which joins immutably over
/// Multibodies.
pub type ReadMultiBodies<'f, N> = BodyMarkerStorage<
    'f,
    N,
    Multibody<N>,
    MultibodyMarker,
    Fetch<'f, MaskedStorage<BodyComponent<N>>>,
>;

/// Put this type in your SystemData to get a storage which joins mutably over
/// Multibodies.
pub type WriteMultiBodies<'f, N> = BodyMarkerStorage<
    'f,
    N,
    Multibody<N>,
    MultibodyMarker,
    FetchMut<'f, MaskedStorage<BodyComponent<N>>>,
>;

/// Component that marks the Body on this Entity as Ground.
/// Do not attach more than one kind of Body Marker type to a Body entity.
#[derive(Default, Copy, Clone, Debug)]
pub struct GroundMarker;

impl Component for GroundMarker {
    type Storage = NullStorage<Self>;
}

/// Put this type in your SystemData to get a storage which joins immutably over
/// Ground type bodies.
pub type ReadGroundBodies<'f, N> =
    BodyMarkerStorage<'f, N, Ground<N>, GroundMarker, Fetch<'f, MaskedStorage<BodyComponent<N>>>>;

/// Put this type in your SystemData to get a storage which joins mutably over
/// Ground type bodies.
pub type WriteGroundBodies<'f, N> = BodyMarkerStorage<
    'f,
    N,
    Ground<N>,
    GroundMarker,
    FetchMut<'f, MaskedStorage<BodyComponent<N>>>,
>;

/// Used by the marker types in this module as a sort of Specs `Storage`
/// meta-type.
pub struct BodyMarkerStorage<'f, N: RealField, B: Body<N>, M: Component, D> {
    body_storage: Storage<'f, BodyComponent<N>, D>,
    marker_storage: ReadStorage<'f, M>,
    phantom: PhantomData<(N, B)>,
}

impl<'a, 'f, N, B, M, D> Join for &'a BodyMarkerStorage<'f, N, B, M, D>
where
    N: RealField,
    B: Body<N>,
    M: Component,
    D: Deref<Target = MaskedStorage<BodyComponent<N>>>,
{
    type Mask = BitSetAnd<&'a BitSet, &'a BitSet>;
    type Type = &'a B;
    type Value = &'a <BodyComponent<N> as Component>::Storage;

    // SAFETY: No unsafe code and no invariants to fulfill.
    unsafe fn open(self) -> (Self::Mask, Self::Value) {
        (
            self.body_storage.mask() & self.marker_storage.mask(),
            self.body_storage.unprotected_storage(),
        )
    }

    // SAFETY: Since we require that the mask was checked, an element for `id` must
    // have been inserted without being removed.
    unsafe fn get(value: &mut Self::Value, id: u32) -> Self::Type {
        value
            .get(id)
            .downcast_ref()
            .expect("Incorrect marker on Body.")
    }
}

impl<'a, 'f, N, B, M, D> Join for &'a mut BodyMarkerStorage<'f, N, B, M, D>
where
    N: RealField,
    B: Body<N>,
    M: Component,
    D: DerefMut<Target = MaskedStorage<BodyComponent<N>>>,
{
    type Mask = BitSetAnd<&'a BitSet, &'a BitSet>;
    type Type = &'a mut B;
    type Value = &'a mut <BodyComponent<N> as Component>::Storage;

    // SAFETY: No unsafe code and no invariants to fulfill.
    unsafe fn open(self) -> (Self::Mask, Self::Value) {
        let bodies = Join::open(&mut self.body_storage);
        (bodies.0 & self.marker_storage.mask(), bodies.1)
    }

    // TODO: audit unsafe
    unsafe fn get(value: &mut Self::Value, id: u32) -> Self::Type {
        // From Specs:
        // This is horribly unsafe. Unfortunately, Rust doesn't provide a way
        // to abstract mutable/immutable state at the moment, so we have to hack
        // our way through it.
        let value: *mut Self::Value = value as *mut Self::Value;

        (*value)
            .get_mut(id)
            .downcast_mut()
            .expect("Incorrect marker on Body.")
    }
}

impl<'f, N: RealField, B: Body<N>, M: Component> SystemData<'f>
    for BodyMarkerStorage<'f, N, B, M, Fetch<'f, MaskedStorage<BodyComponent<N>>>>
{
    fn setup(world: &mut World) {
        setup_component::<BodyComponent<N>>(world);
        setup_component::<M>(world);
    }

    fn fetch(world: &'f World) -> Self {
        Self {
            body_storage: world.read_storage(),
            marker_storage: world.read_storage(),
            phantom: PhantomData,
        }
    }

    fn reads() -> Vec<ResourceId> {
        vec![
            ResourceId::new::<MaskedStorage<BodyComponent<N>>>(),
            ResourceId::new::<MaskedStorage<M>>(),
        ]
    }

    fn writes() -> Vec<ResourceId> {
        vec![]
    }
}

impl<'f, N: RealField, B: Body<N>, M: Component> SystemData<'f>
    for BodyMarkerStorage<'f, N, B, M, FetchMut<'f, MaskedStorage<BodyComponent<N>>>>
{
    fn setup(world: &mut World) {
        setup_component::<BodyComponent<N>>(world);
        setup_component::<M>(world);
    }

    fn fetch(world: &'f World) -> Self {
        Self {
            body_storage: world.write_storage(),
            marker_storage: world.read_storage(),
            phantom: PhantomData,
        }
    }

    fn reads() -> Vec<ResourceId> {
        vec![ResourceId::new::<MaskedStorage<M>>()]
    }

    fn writes() -> Vec<ResourceId> {
        vec![ResourceId::new::<MaskedStorage<BodyComponent<N>>>()]
    }
}

fn setup_component<T: Component>(world: &mut World) {
    world.entry::<MaskedStorage<T>>().or_insert_with(|| {
        MaskedStorage::new(<<T as Component>::Storage as TryDefault>::unwrap_default())
    });
    world
        .fetch_mut::<MetaTable<dyn AnyStorage>>()
        .register(&*world.fetch::<MaskedStorage<T>>());
}
