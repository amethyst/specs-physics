use crate::{
    joints::JointComponent,
    nalgebra::RealField,
    nphysics::{
        joint::{JointConstraint, JointConstraintSet as NJointConstraintSet},
        object::BodyPartHandle,
    },
};

use specs::{
    shred::{Fetch, FetchMut, MetaTable, ResourceId},
    storage::{AnyStorage, ComponentEvent, MaskedStorage, TryDefault},
    world::EntitiesRes,
    Component, Entity, Join, ReaderId, SystemData, World, WorldExt, WriteStorage,
};

struct JointEvent {
    handle: Entity,
    part_one: BodyPartHandle<Entity>,
    part_two: BodyPartHandle<Entity>,
}

// Reader resource used by `BodySet` during fetching to populate
// `BodyRemovalRes` with removal events.
struct JointReaderRes(ReaderId<ComponentEvent>);

struct JointInsertionRes(Vec<JointEvent>);

struct JointRemovalRes(Vec<JointEvent>);

/// The `set` type needed by nphysics for constraint joints.
pub struct JointConstraintSet<'f, N: RealField> {
    pub storage: WriteStorage<'f, JointComponent<N>>,
    entities: Fetch<'f, EntitiesRes>,
    insertions: FetchMut<'f, JointInsertionRes>,
    removals: FetchMut<'f, JointRemovalRes>,
}

impl<'f, N: RealField> SystemData<'f> for JointConstraintSet<'f, N> {
    fn setup(world: &mut World) {
        // Setup storage for joint component.
        world
            .entry::<MaskedStorage<JointComponent<N>>>()
            .or_insert_with(|| {
                MaskedStorage::new(
                    <<JointComponent<N> as Component>::Storage as TryDefault>::unwrap_default(),
                )
            });
        world
            .fetch_mut::<MetaTable<dyn AnyStorage>>()
            .register(&*world.fetch::<MaskedStorage<JointComponent<N>>>());

        // Setup resource for insertion/removal buffers.
        world
            .entry::<JointInsertionRes>()
            .or_insert_with(|| JointInsertionRes(Vec::default()));
        world
            .entry::<JointRemovalRes>()
            .or_insert_with(|| JointRemovalRes(Vec::default()));

        // Setup ComponentEvent reader resource.
        // No worries about race condition here due to mut exclusive World reference.
        // Entry cannot be used since mut reference isn't passed to closure.
        if !world.has_value::<JointReaderRes>() {
            let id = world.write_storage::<JointComponent<N>>().register_reader();
            world.insert(JointReaderRes(id));
        }
    }

    fn fetch(world: &'f World) -> Self {
        let entities = world.read_resource::<EntitiesRes>();
        let storage = world.write_storage::<JointComponent<N>>();

        let mut reader = world.write_resource::<JointReaderRes>();
        let mut insertions = world.write_resource::<JointInsertionRes>();
        let mut removals = world.write_resource::<JointRemovalRes>();

        for event in storage.channel().read(&mut reader.0) {
            match event {
                ComponentEvent::Removed(index) => {
                    let entity = entities.entity(*index);
                    if let Some(joint) = storage.get(entity) {
                        let anchors = joint.0.anchors();
                        removals.0.push(JointEvent {
                            handle: entities.entity(*index),
                            part_one: anchors.0,
                            part_two: anchors.1,
                        });
                    } else {
                        error!("Failed to record anchors of removed Joint {:?}", entity);
                    }
                }
                ComponentEvent::Inserted(index) => {
                    let entity = entities.entity(*index);
                    if let Some(joint) = storage.get(entity) {
                        let anchors = joint.0.anchors();
                        insertions.0.push(JointEvent {
                            handle: entities.entity(*index),
                            part_one: anchors.0,
                            part_two: anchors.1,
                        });
                    } else {
                        error!("Failed to record anchors of inserted Joint {:?}", entity);
                    }
                }
                // No need for modified events.
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
            ResourceId::new::<MaskedStorage<JointComponent<N>>>(),
            ResourceId::new::<JointReaderRes>(),
            ResourceId::new::<JointInsertionRes>(),
            ResourceId::new::<JointRemovalRes>(),
        ]
    }
}

impl<'f, N: RealField> NJointConstraintSet<N, Entity> for JointConstraintSet<'f, N> {
    type Handle = Entity;
    type JointConstraint = dyn JointConstraint<N, Entity>;

    fn get(&self, handle: Entity) -> Option<&dyn JointConstraint<N, Entity>> {
        self.storage.get(handle).map(|x| x.0.as_ref())
    }

    fn get_mut(&mut self, handle: Entity) -> Option<&mut dyn JointConstraint<N, Entity>> {
        self.storage.get_mut(handle).map(|x| x.0.as_mut())
    }

    fn contains(&self, handle: Entity) -> bool {
        self.storage.contains(handle)
    }

    fn foreach(&self, mut f: impl FnMut(Entity, &dyn JointConstraint<N, Entity>)) {
        for (entity, joint) in (&self.entities, &self.storage).join() {
            f(entity, joint.0.as_ref())
        }
    }

    fn foreach_mut(&mut self, mut f: impl FnMut(Entity, &mut dyn JointConstraint<N, Entity>)) {
        for (entity, joint) in (&self.entities, &mut self.storage).join() {
            f(entity, joint.0.as_mut())
        }
    }

    fn pop_insertion_event(
        &mut self,
    ) -> Option<(Self::Handle, BodyPartHandle<Entity>, BodyPartHandle<Entity>)> {
        self.insertions
            .0
            .pop()
            .map(|e| (e.handle, e.part_one, e.part_two))
    }

    fn pop_removal_event(
        &mut self,
    ) -> Option<(Entity, BodyPartHandle<Entity>, BodyPartHandle<Entity>)> {
        self.removals
            .0
            .pop()
            .map(|e| (e.handle, e.part_one, e.part_two))
    }

    fn remove(&mut self, to_remove: Entity) {
        let _ = self.storage.remove(to_remove);
    }
}
