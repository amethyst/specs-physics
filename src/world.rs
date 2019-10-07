use crate::{
    nalgebra::RealField,
    nphysics::{
        force_generator::DefaultForceGeneratorSet,
        joint::DefaultJointConstraintSet,
        object::{Body, BodySet as NBodySet, DefaultBodySet, DefaultColliderSet},
        world::{GeometricalWorld, MechanicalWorld},
    },
};

use specs::{
    shred::{Fetch, FetchMut, MetaTable, ResourceId},
    storage::{AntiStorage, AnyStorage, MaskedStorage, TryDefault, UnprotectedStorage},
    world::{EntitiesRes, Index},
    BitSet, Component, DenseVecStorage, Entity, FlaggedStorage, Join, Storage, SystemData, World,
};

use std::ops::{Deref, DerefMut, Not};

pub type BodyHandleType = Entity;
pub type ColliderHandleType = Entity;

pub type MechanicalWorldRes<'a, N> = MechanicalWorld<N, BodySet<'a, N>, ColliderHandleType>;
pub type GeometricalWorldRes<N> = GeometricalWorld<N, BodyHandleType, ColliderHandleType>;

// TODO
pub type ColliderSetRes<N> = DefaultColliderSet<N, BodyHandleType>;

// TODO: Could likely turn these into storages?
pub type JointConstraintSetRes<'a, N> = DefaultJointConstraintSet<N, BodySet<'a, N>>;
pub type ForceGeneratorSetRes<'a, N> = DefaultForceGeneratorSet<N, BodySet<'a, N>>;

#[derive(Shrinkwrap)]
#[shrinkwrap(mutable)]
pub struct BodyRemovals(pub Vec<BodyHandleType>);

pub struct BodySet<'a, N: RealField> {
    entities: Fetch<'a, EntitiesRes>,
    storage: WriteBodyStorage<'a, N>,
    removals: FetchMut<'a, BodyRemovals>,
}

impl<'a, N: RealField> SystemData<'a> for BodySet<'a, N> {
    fn setup(world: &mut World) {
        world
            .entry::<MaskedStorage<BodyComponent<N>>>()
            .or_insert_with(|| {
                MaskedStorage::new(
                    <<BodyComponent<N> as Component>::Storage as TryDefault>::unwrap_default(),
                )
            });
        world
            .fetch_mut::<MetaTable<dyn AnyStorage>>()
            .register(&*world.fetch::<MaskedStorage<BodyComponent<N>>>());
    }

    fn fetch(world: &'a World) -> Self {
        BodySet {
            entities: world.fetch(),
            storage: BodySetStorage(Storage::new(world.fetch(), world.fetch_mut())),
            removals: world.fetch_mut(),
        }
    }

    fn reads() -> Vec<ResourceId> {
        vec![ResourceId::new::<EntitiesRes>()]
    }

    fn writes() -> Vec<ResourceId> {
        vec![
            ResourceId::new::<MaskedStorage<BodyComponent<N>>>(),
            ResourceId::new::<BodyRemovals>(),
        ]
    }
}

impl<'a, N: RealField> NBodySet<N> for BodySet<'a, N> {
    type Body = dyn Body<N>;
    type Handle = BodyHandleType;

    fn get(&self, handle: Self::Handle) -> Option<&Self::Body> {
        self.storage.0.get(handle).map(|x| x.0.as_ref())
    }

    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Self::Body> {
        self.storage.0.get_mut(handle).map(|x| x.0.as_mut())
    }

    fn get_pair_mut(
        &mut self,
        handle1: Self::Handle,
        handle2: Self::Handle,
    ) -> (Option<&mut Self::Body>, Option<&mut Self::Body>) {
        let b1 = self.get_mut(handle1).map(|b| b as *mut dyn Body<N>);
        let b2 = self.get_mut(handle2).map(|b| b as *mut dyn Body<N>);
        unsafe {
            use std::mem;
            (
                b1.map(|b| mem::transmute(b)),
                b2.map(|b| mem::transmute(b))
            )
        }
    }

    fn contains(&self, handle: Self::Handle) -> bool {
        self.storage.0.contains(handle)
    }

    fn foreach(&self, mut f: impl FnMut(Self::Handle, &Self::Body)) {
        for (handle, body) in (&self.entities, &self.storage.0).join() {
            f(handle, body.0.as_ref());
        }
    }

    fn foreach_mut(&mut self, mut f: impl FnMut(Self::Handle, &mut Self::Body)) {
        for (handle, body) in (&self.entities, &mut self.storage.0).join() {
            f(handle, body.0.as_mut());
        }
    }

    fn pop_removal_event(&mut self) -> Option<Self::Handle> {
        self.removals.pop()
    }
}

pub type BodySetRes<N> = DefaultBodySet<N>;

pub type ReadBodyStorage<'a, N> = BodySetStorage<'a, N, Fetch<'a, MaskedStorage<BodyComponent<N>>>>;

impl<'a, N: RealField> SystemData<'a> for ReadBodyStorage<'a, N> {
    fn setup(res: &mut World) {
        res.entry::<MaskedStorage<BodyComponent<N>>>()
            .or_insert_with(|| {
                MaskedStorage::new(
                    <<BodyComponent<N> as Component>::Storage as TryDefault>::unwrap_default(),
                )
            });
        res.fetch_mut::<MetaTable<dyn AnyStorage>>()
            .register(&*res.fetch::<MaskedStorage<BodyComponent<N>>>());
    }

    fn fetch(res: &'a World) -> Self {
        BodySetStorage(Storage::new(res.fetch(), res.fetch()))
    }

    fn reads() -> Vec<ResourceId> {
        vec![ResourceId::new::<EntitiesRes>(), ResourceId::new::<MaskedStorage<BodyComponent<N>>>()]
    }

    fn writes() -> Vec<ResourceId> {
        vec![]
    }
}

pub type WriteBodyStorage<'a, N> =
    BodySetStorage<'a, N, FetchMut<'a, MaskedStorage<BodyComponent<N>>>>;

impl<'a, N: RealField> SystemData<'a> for WriteBodyStorage<'a, N> {
    fn setup(res: &mut World) {
        res.entry::<MaskedStorage<BodyComponent<N>>>()
            .or_insert_with(|| {
                MaskedStorage::new(
                    <<BodyComponent<N> as Component>::Storage as TryDefault>::unwrap_default(),
                )
            });
        res.fetch_mut::<MetaTable<dyn AnyStorage>>()
            .register(&*res.fetch::<MaskedStorage<BodyComponent<N>>>());
    }

    fn fetch(res: &'a World) -> Self {
        BodySetStorage(Storage::new(res.fetch(), res.fetch_mut()))
    }

    fn reads() -> Vec<ResourceId> {
        vec![ResourceId::new::<EntitiesRes>()]
    }

    fn writes() -> Vec<ResourceId> {
        vec![ResourceId::new::<MaskedStorage<BodyComponent<N>>>()]
    }
}

#[derive(Shrinkwrap)]
#[shrinkwrap(mutable)]
// Yuck! Bad allocation story here. Hard, if not impossible to avoid, however.
pub struct BodyComponent<N: RealField>(pub Box<dyn Body<N>>);

impl<N: RealField> Component for BodyComponent<N> {
    type Storage = FlaggedStorage<Self, DenseVecStorage<Self>>;
}

pub struct BodySetStorage<'e, N: RealField, D>(pub Storage<'e, BodyComponent<N>, D>);

impl<'a, 'e, N, D> Join for &'a BodySetStorage<'e, N, D>
where
    N: RealField,
    D: Deref<Target = MaskedStorage<BodyComponent<N>>>,
{
    type Mask = &'a BitSet;
    type Type = &'a dyn Body<N>;
    type Value = &'a <BodyComponent<N> as Component>::Storage;

    unsafe fn open(self) -> (Self::Mask, Self::Value) {
        Join::open(&self.0)
    }

    unsafe fn get(v: &mut Self::Value, i: Index) -> &'a dyn Body<N> {
        v.get(i).0.as_ref()
    }
}

impl<'a, 'e, N, D> Join for &'a mut BodySetStorage<'e, N, D>
where
    N: RealField,
    D: DerefMut<Target = MaskedStorage<BodyComponent<N>>>,
{
    type Mask = &'a BitSet;
    type Type = &'a mut dyn Body<N>;
    type Value = &'a mut <BodyComponent<N> as Component>::Storage;

    unsafe fn open(self) -> (Self::Mask, Self::Value) {
        Join::open(&mut self.0)
    }

    unsafe fn get(v: &mut Self::Value, i: Index) -> &'a mut dyn Body<N> {
        let value: *mut Self::Value = v as *mut Self::Value;
        (*value).get_mut(i).0.as_mut()
    }
}

impl<'a, 'e, N, D> Not for &'a BodySetStorage<'e, N, D>
where
    N: RealField,
    D: Deref<Target = MaskedStorage<BodyComponent<N>>>,
{
    type Output = AntiStorage<'a>;

    fn not(self) -> Self::Output {
        Not::not(&self.0)
    }
}
