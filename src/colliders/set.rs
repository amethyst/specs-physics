use crate::{
    colliders::ColliderComponent,
    nalgebra::RealField,
    ncollide::pipeline::CollisionObjectSet,
    nphysics::object::{Collider, ColliderRemovalData, ColliderSet as NColliderSet},
};

use specs::{
    shred::{Fetch, FetchMut, MetaTable, ResourceId},
    storage::{AnyStorage, ComponentEvent, MaskedStorage, TryDefault, UnprotectedStorage},
    world::EntitiesRes,
    Component, Entity, Join, ReaderId, SystemData, World, WorldExt, WriteStorage,
};

struct ColliderInsertionRes(Vec<Entity>);

struct ColliderRemovalRes<N: RealField>(Vec<(Entity, ColliderRemovalData<N, Entity>)>);

struct ColliderReaderRes(ReaderId<ComponentEvent>);

/// The `set` type needed by nphysics for colliders.
pub struct ColliderSet<'f, N: RealField> {
    pub storage: WriteStorage<'f, ColliderComponent<N>>,
    entities: Fetch<'f, EntitiesRes>,
    insertions: FetchMut<'f, ColliderInsertionRes>,
    removals: FetchMut<'f, ColliderRemovalRes<N>>,
}

impl<'f, N: RealField> SystemData<'f> for ColliderSet<'f, N> {
    fn setup(world: &mut World) {
        // Setup storage for collider component.
        world
            .entry::<MaskedStorage<ColliderComponent<N>>>()
            .or_insert_with(|| {
                MaskedStorage::new(
                    <<ColliderComponent<N> as Component>::Storage as TryDefault>::unwrap_default(),
                )
            });
        world
            .fetch_mut::<MetaTable<dyn AnyStorage>>()
            .register(&*world.fetch::<MaskedStorage<ColliderComponent<N>>>());

        // Setup resources for insertion and removal buffers.
        world
            .entry::<ColliderInsertionRes>()
            .or_insert_with(|| ColliderInsertionRes(Vec::default()));
        world
            .entry::<ColliderRemovalRes<N>>()
            .or_insert_with(|| ColliderRemovalRes::<N>(Vec::default()));

        // Setup ComponentEvent reader resource.
        // No worries about race condition here due to mut exclusive World reference.
        // Entry cannot be used since mut reference isn't passed to closure.
        if !world.has_value::<ColliderReaderRes>() {
            let reader = world
                .write_storage::<ColliderComponent<N>>()
                .register_reader();
            world.insert(ColliderReaderRes(reader));
        }
    }

    fn fetch(world: &'f World) -> Self {
        let entities = world.read_resource::<EntitiesRes>();
        let storage = world.write_storage::<ColliderComponent<N>>();
        let mut reader = world.write_resource::<ColliderReaderRes>();
        let mut insertions = world.write_resource::<ColliderInsertionRes>();
        let mut removals = world.write_resource::<ColliderRemovalRes<N>>();

        for event in storage.channel().read(&mut reader.0) {
            match event {
                ComponentEvent::Removed(index) => {
                    // Don't panic please! (I'm asking the computer)
                    unsafe {
                        // Ok this can be even more incredibly wrong than what we did in body_set.
                        // Only one way to find out
                        // (that isn't actually investigating the code underneath)
                        if let Some(removal_data) = UnprotectedStorage::<ColliderComponent<N>>::get(
                            storage.unprotected_storage(),
                            *index,
                        )
                        .removal_data()
                        {
                            removals.0.push((entities.entity(*index), removal_data));
                        }
                    }
                }
                ComponentEvent::Inserted(index) => {
                    insertions.0.push(entities.entity(*index));
                }
                _ => {}
            }
        }

        Self {
            entities,
            storage,
            insertions,
            removals,
        }
    }

    fn reads() -> Vec<ResourceId> {
        vec![ResourceId::new::<EntitiesRes>()]
    }

    fn writes() -> Vec<ResourceId> {
        vec![
            ResourceId::new::<MaskedStorage<ColliderComponent<N>>>(),
            ResourceId::new::<ColliderReaderRes>(),
            ResourceId::new::<ColliderInsertionRes>(),
            ResourceId::new::<ColliderRemovalRes<N>>(),
        ]
    }
}

impl<'f, N: RealField> CollisionObjectSet<N> for ColliderSet<'f, N> {
    type CollisionObject = Collider<N, Entity>;
    type CollisionObjectHandle = Entity;

    fn collision_object(
        &self,
        handle: Self::CollisionObjectHandle,
    ) -> Option<&Self::CollisionObject> {
        self.storage.get(handle).map(|x| &x.0)
    }

    fn foreach(&self, mut f: impl FnMut(Self::CollisionObjectHandle, &Self::CollisionObject)) {
        for (handle, collider) in (&self.entities, &self.storage).join() {
            f(handle, &collider.0);
        }
    }
}

impl<'f, N: RealField> NColliderSet<N, Entity> for ColliderSet<'f, N> {
    type Handle = Entity;

    fn get(&self, handle: Self::Handle) -> Option<&Collider<N, Entity>> {
        self.storage.get(handle).map(|x| &x.0)
    }

    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut Collider<N, Entity>> {
        self.storage.get_mut(handle).map(|x| &mut x.0)
    }

    fn contains(&self, handle: Self::Handle) -> bool {
        self.storage.contains(handle)
    }

    fn foreach(&self, mut f: impl FnMut(Self::Handle, &Collider<N, Entity>)) {
        for (handle, collider) in (&self.entities, &self.storage).join() {
            f(handle, &collider.0);
        }
    }

    fn foreach_mut(&mut self, mut f: impl FnMut(Self::Handle, &mut Collider<N, Entity>)) {
        for (handle, collider) in (&self.entities, &mut self.storage).join() {
            f(handle, &mut collider.0);
        }
    }

    fn pop_insertion_event(&mut self) -> Option<Self::Handle> {
        self.insertions.0.pop()
    }

    fn pop_removal_event(&mut self) -> Option<(Self::Handle, ColliderRemovalData<N, Entity>)> {
        self.removals.0.pop()
    }

    fn remove(&mut self, to_remove: Self::Handle) -> Option<&mut ColliderRemovalData<N, Entity>> {
        let collider = self.storage.remove(to_remove)?;
        if let Some(removal_data) = collider.removal_data() {
            self.removals.0.push((to_remove, removal_data));
            self.removals.0.last_mut().map(|r| &mut r.1)
        } else {
            None
        }
    }
}
