use crate::{
    nalgebra::RealField,
    nphysics::object::{Body, BodySet as NBodySet, Ground, Multibody, RigidBody},
};

use specs::{
    shred::{Fetch, FetchMut, MetaTable, ResourceId},
    storage::{AnyStorage, ComponentEvent, MaskedStorage, TryDefault},
    world::EntitiesRes,
    BitSet, Component, DenseVecStorage, Entity, FlaggedStorage, Join, ReaderId, SystemData, World,
    WorldExt, WriteStorage,
};

/// Handle component to a generic BodyPart, consisting of two members.
/// The first item, Entity, points at the BodyComponent for this Part.
/// The second item, usize, is the index of the Part in that Body.
pub struct BodyPartHandle(Entity, usize);

impl Component for BodyPartHandle {
    type Storage = DenseVecStorage<Self>;
}

/// Component that marks the Body on this Entity as a RigidBody.
/// Do not attach more than one kind of Body Marker type to a Body entity.
pub struct RigidBodyMarker;

impl Component for RigidBodyMarker {
    type Storage = NullStorage;
}

/// Component that marks the Body on this Entity as a Multibody.
/// Do not attach more than one kind of Body Marker type to a Body entity.
pub struct MultibodyMarker;

impl Component for RigidBodyMarker {
    type Storage = NullStorage;
}

/// Component that marks the Body on this Entity as Ground.
/// Do not attach more than one kind of Body Marker type to a Body entity.
pub struct GroundMarker;

impl Component for RigidBodyMarker {
    type Storage = NullStorage;
}

/// The handle type of Bodies passed to nphysics API type parameters receiving
/// "BodyHandle".
pub type BodyHandleType = Entity;

/// The component type of all physics bodies. Attaching this component to your
/// entity with a `Position` component will make the syncing system synchronize
/// the isometry of the first part in that Body to your `Position` (or simply
/// the position of the body if it is a single part body). This relationship is
/// one-way, however. If you'd like to update the position of a Body, **do so
/// via this component, and not from the `Position` component**.
///
/// If you'd like to synchronize individual parts of a body to a `Position`, you
/// should not attach a `Position` to the entity with this Component, and should
/// instead attach a `BodyPartHandle`, which points to the multipart body, to
/// the entity with the `Position` for a single part.
// Ouch! Bad allocation story here with DenseVecStorage<Box<dyn Body>>.
// However, this is hard if not impossible to avoid due to nphysics API limitations.
#[derive(Shrinkwrap)]
#[shrinkwrap(mutable)]
pub struct BodyComponent<N: RealField> {
    #[shrinkwrap(main_field)]
    pub body: Box<dyn Body<N>>,
    // Bitset of entities with a BodyPartHandle to this Body
    handles: BitSet,
}

impl<N: RealField> Component for BodyComponent<N> {
    type Storage = FlaggedStorage<Self, DenseVecStorage<Self>>;
}

impl<N: RealField> BodyComponent<N> {
    /// Creates a new Body Component.
    /// This is made useful by inserting the returned struct
    /// into the BodyComponent's Storage.
    pub fn new<B: Body<N>>(body: B) -> Self {
        Self {
            body: Box::new(body),
            handles: BitSet::new(),
        }
    }

    /// Attempts to cast this Body to a RigidBody.
    /// Just sugar for `Body::downcast_ref()`.
    pub fn as_rigid_body(&self) -> Option<&RigidBody<N>> {
        self.body.downcast_ref()
    }

    /// Attempts to mutably cast this Body to a RigidBody.
    /// Just sugar for `Body::downcast_mut()`.
    pub fn as_rigid_body_mut(&mut self) -> Option<&mut RigidBody<N>> {
        self.body.downcast_mut()
    }

    /// Attempts to cast this Body to a Multibody.
    /// Just sugar for `Body::downcast_ref()`.
    pub fn as_multi_body(&self) -> Option<&Multibody<N>> {
        self.body.downcast_ref()
    }

    /// Attempts to mutably cast this Body to a Multibody.
    /// Just sugar for `Body::downcast_mut()`.
    pub fn as_multi_body_mut(&mut self) -> Option<&mut Multibody<N>> {
        self.body.downcast_mut()
    }

    /// Attempts to cast this Body to Ground.
    /// Just sugar for `Body::downcast_ref()`.
    pub fn as_ground(&self) -> Option<&Ground<N>> {
        self.body.downcast_ref()
    }

    /// Attempts to mutably cast this Body to Ground.
    /// Just sugar for `Body::downcast_mut()`.
    pub fn as_ground_mut(&mut self) -> Option<&mut Ground<N>> {
        self.body.downcast_mut()
    }
}

/// List of removals used by `BodySet` so that nphysics may `pop` single removal
/// events.
#[derive(Clone, Debug, Default, Eq, Hash, Ord, PartialEq, PartialOrd, Shrinkwrap)]
#[shrinkwrap(mutable)]
pub(crate) struct BodyRemovalRes(pub Vec<BodyHandleType>);

/// Reader resource used by `BodySet` during fetching to populate
/// `BodyRemovalRes` with removal events.
#[derive(Debug, Shrinkwrap)]
#[shrinkwrap(mutable)]
pub(crate) struct BodyReaderRes(pub ReaderId<ComponentEvent>);

/// This structure is only used to pass the BodyComponent storage to nphysics
/// API's. You probably don't want to use it. unless you're using your own
/// system for stepping.
pub struct BodySet<'f, N: RealField> {
    entities: Fetch<'f, EntitiesRes>,
    storage: WriteStorage<'f, BodyComponent<N>>,
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
            .or_insert_with(|| BodyRemovalRes::default());

        // Setup ComponentEvent reader resource.
        if !world.has_value::<BodyReaderRes>() {
            let reader = world.write_storage::<BodyComponent<N>>().register_reader();
            world.insert(BodyReaderRes(reader));
        }
    }

    fn fetch(world: &'f World) -> Self {
        let entities = world.read_resource::<EntitiesRes>();
        let storage = world.write_storage::<BodyComponent<N>>();
        let mut reader = world.write_resource::<BodyReaderRes>();
        let mut removals = world.write_resource::<BodyRemovalRes>();

        for event in storage.channel().read(&mut reader) {
            if let ComponentEvent::Removed(index) = event {
                // Is grabbing the current entity for this index logically wrong? Maybe.
                // Is doing this in SystemData::fetch morally wrong? Yes.
                removals.push(entities.entity(*index));
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
    type Handle = BodyHandleType;

    fn get(&self, handle: Self::Handle) -> Option<&dyn Body<N>> {
        self.storage.get(handle).map(|x| x.body.as_ref())
    }

    fn get_mut(&mut self, handle: Self::Handle) -> Option<&mut dyn Body<N>> {
        self.storage.get_mut(handle).map(|x| x.body.as_mut())
    }

    fn contains(&self, handle: Self::Handle) -> bool {
        self.storage.contains(handle)
    }

    fn foreach(&self, f: &mut dyn FnMut(Self::Handle, &dyn Body<N>)) {
        for (handle, body) in (&self.entities, &self.storage).join() {
            f(handle, body.body.as_ref());
        }
    }

    fn foreach_mut(&mut self, f: &mut dyn FnMut(Self::Handle, &mut dyn Body<N>)) {
        for (handle, body) in (&self.entities, &mut self.storage).join() {
            f(handle, body.body.as_mut());
        }
    }

    fn pop_removal_event(&mut self) -> Option<Self::Handle> {
        self.removals.pop()
    }
}
