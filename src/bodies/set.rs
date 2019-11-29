use crate::{
    bodies::{marker::ReadMultiBodies, BodyComponent, ReadGroundBodies, ReadRigidBodies},
    nalgebra::RealField,
    nphysics::object::{Body, BodySet as NBodySet},
};

use specs::{
    shred::{Fetch, FetchMut, MetaTable, ResourceId},
    storage::{AnyStorage, ComponentEvent, MaskedStorage, TryDefault},
    world::EntitiesRes,
    Component, Entity, Join, ReaderId, SystemData, World, WorldExt, WriteStorage,
};

// List of removals used by `BodySet` so that nphysics may `pop` single removal
// events.
struct BodyRemovalRes(Vec<Entity>);

// Reader resource used by `BodySet` during fetching to populate
// `BodyRemovalRes` with removal events.
struct BodyReaderRes(ReaderId<ComponentEvent>);

/// This structure is only used to pass the BodyComponent storage to nphysics
/// API's. You probably don't want to use it. unless you're using your own
/// system for stepping.
pub struct BodySet<'f, N: RealField> {
    pub storage: WriteStorage<'f, BodyComponent<N>>,

    entities: Fetch<'f, EntitiesRes>,
    removals: FetchMut<'f, BodyRemovalRes>,
}

impl<'f, N: RealField> SystemData<'f> for BodySet<'f, N> {
    fn setup(world: &mut World) {
        // Setup storage for body component.
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

        // Setup resource for removal buffer.
        world
            .entry::<BodyRemovalRes>()
            .or_insert_with(|| BodyRemovalRes(Vec::default()));

        // Setup ComponentEvent reader resource.
        // No worries about race condition here due to mut exclusive World reference.
        // Entry cannot be used since mut reference isn't passed to closure.
        if !world.has_value::<BodyReaderRes>() {
            let id = world.write_storage::<BodyComponent<N>>().register_reader();
            world.insert(BodyReaderRes(id));
        }

        // Setup marker component storages.
        ReadRigidBodies::<N>::setup(world);
        ReadMultiBodies::<N>::setup(world);
        ReadGroundBodies::<N>::setup(world);
    }

    fn fetch(world: &'f World) -> Self {
        let entities = world.read_resource::<EntitiesRes>();
        let storage = world.write_storage::<BodyComponent<N>>();
        let mut reader = world.write_resource::<BodyReaderRes>();
        let mut removals = world.write_resource::<BodyRemovalRes>();

        for event in storage.channel().read(&mut reader.0) {
            if let ComponentEvent::Removed(index) = event {
                // Is grabbing the current entity for this index logically wrong? Maybe.
                // Is doing this in SystemData::fetch morally wrong? Yes.
                removals.0.push(entities.entity(*index));
            }
        }

        Self {
            entities,
            storage,
            removals,
        }
    }

    fn reads() -> Vec<ResourceId> {
        vec![ResourceId::new::<EntitiesRes>()]
    }

    fn writes() -> Vec<ResourceId> {
        vec![
            ResourceId::new::<MaskedStorage<BodyComponent<N>>>(),
            ResourceId::new::<BodyReaderRes>(),
            ResourceId::new::<BodyRemovalRes>(),
        ]
    }
}

impl<'f, N: RealField> NBodySet<N> for BodySet<'f, N> {
    type Handle = Entity;

    fn get(&self, handle: Self::Handle) -> Option<&dyn Body<N>> {
        self.storage.get(handle).map(|x| x.0.as_ref())
    }

    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut dyn Body<N>> {
        self.storage.get_mut(handle).map(|x| x.0.as_mut())
    }

    fn contains(&self, handle: Self::Handle) -> bool {
        self.storage.contains(handle)
    }

    fn foreach(&self, f: &mut dyn FnMut(Self::Handle, &dyn Body<N>)) {
        for (handle, body) in (&self.entities, &self.storage).join() {
            f(handle, body.0.as_ref());
        }
    }

    fn foreach_mut(&mut self, f: &mut dyn FnMut(Self::Handle, &mut dyn Body<N>)) {
        for (handle, body) in (&self.entities, &mut self.storage).join() {
            f(handle, body.0.as_mut());
        }
    }

    fn pop_removal_event(&mut self) -> Option<Self::Handle> {
        self.removals.0.pop()
    }
}
