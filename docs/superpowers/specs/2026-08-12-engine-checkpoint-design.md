# Engine checkpointing

In-memory snapshot of the engine state — every entity, its components, and the
physics simulation — plus a restore that resets the engine back to that state.
No disk format, no save games: a checkpoint is a live C++ object and is only
valid for the run that produced it.

## Why this is cheap

Two properties of the existing engine do most of the work:

- **GPU handles are never unloaded.** `Mesh`, `Material`, `ShadowMap`,
  `TextureCubemap` and `Skybox` are raylib/tacoRender PODs holding GL ids. Copying
  one is aliasing an object that outlives every checkpoint, so components carrying
  them are copyable as-is.
- **Jolt has its own rollback support.** `PhysicsSystem::SaveState` /
  `RestoreState` with a `StateRecorderImpl` capture global state, bodies, contacts
  and constraints. `CharacterBase::SaveState` covers the character ground state.
  We do not reimplement any of it.

What is left is copying the ECS components and re-syncing the two.

## API

```cpp
// Engine.h
Checkpoint Save();
void Restore(Checkpoint &cp);   // non-const: the recorders need Rewind()

void RequestRestore(Checkpoint &cp);  // queue a restore from inside a System hook
void ApplyPendingRestore();           // drain the queue; Run() calls it after Update()

template<class T> void Ignore(); // opt a type out of checkpointing
```

There is deliberately no `Track<T>()`. `Entity::Add<T>` is the only path a
component has into the registry — `Engine::registry_` is private, and the engine
itself only ever emplaces the two ignored physics types — so `Add` registers the
type's capture the first time one is added, exactly as it already registers a
`System` subclass's dispatch hooks. Tracking cannot be forgotten because nobody
has to remember it.

A `System` phase hook is the only place game code holds an `Engine *`, but
`Restore` destroys entities and rewrites the very component storages the system
dispatch is iterating right then. So a system calls `RequestRestore`, and `Run`
applies it once `Update` has returned. The queued `Checkpoint` must be owned by
something `Restore` cannot destroy — not by a `System` and not by an entity.

`Checkpoint` (`src/taco/Checkpoint.h`) is move-only, because
`JPH::StateRecorderImpl` is:

```cpp
class Checkpoint {
    friend class Engine;

    std::vector<entt::entity> entities_;                          // alive at capture
    std::vector<std::function<void(entt::registry &)>> restore_;  // one per tracked type
    JPH::StateRecorderImpl physics_;
    std::set<entt::entity> colliders_;                            // owned a body at capture
    std::map<entt::entity, JPH::StateRecorderImpl> characters_;

public:
    Checkpoint() = default;
    Checkpoint(Checkpoint &&) = default;
};
```

A checkpoint is reusable: `Restore` never consumes it, so the same object can
reset the engine any number of times.

`Save` and `Restore` are `Engine` members because they need
`physics_->system_`, and `Engine` is already a `friend` of `PhysicsEngine`.

## Which components are tracked

`Sunlight` is the only type the engine registers in its constructor, because it
needs a hand-written capture and the first registration wins. Everything else —
`Transform`, `Link`, `Camera`, `Mesh`, `Material`, `BoundingBox`, `Environment`,
`Sky` and every game component — is picked up by `Add`.

Every copyable component type is registered by `Entity::Add<T>`. Two are opted
back out with `Ignore<T>()`: both are copyable, but copying them would duplicate a
Jolt handle rather than its state, and the state has a better source:

| Type | Why not copied | How its state survives |
|---|---|---|
| `Collider` | a `JPH::BodyID`; copying it copies an id, not a body | Jolt `SaveState` + `Transform` |
| `Character` | a `JPH::Ref<JPH::Character>`; likewise | Jolt `SaveState` + `Transform` |

`entt::entity` is ignored too: the entity storage is the registry's own, and
`entities_` already covers it.

The registration has to happen in `Add<T>`, where `T` is still a static type.
EnTT 3.15's type-erased storage API is `value(entity)` (opaque read) and
`push(entity, const void *)` (copy-construct), both of which need a destination
storage of the same *static* type, and `basic_sparse_set` exposes no clone-factory
virtual — so a checkpoint cannot be built by walking `registry.storage()` after
the fact.

`Add` cannot register a **move-only** component: there is no way to capture one by
value. That is the only remaining silent hole, so `Save` and `Restore` walk
`registry.storage()` and warn once per non-empty storage that is neither tracked
nor ignored, naming `pool.type().name()`.

## System state

Systems keep state outside the ECS — `LookSystem::yaw` / `pitch`, for instance,
which has to rewind with the world.

Since the engine API refactor a `System` subclass is stored **by value in its own
typed storage**, exactly like any other component, so it needs no mechanism of its
own: `Add<LookSystem>` registers it through the same generic path as everything
else, and `engine.Ignore<LookSystem>()` is how a system opts out and keeps its
running state across a restore.

## Capture

1. For each registered type `T`: its capture thunk builds a
   `std::map<entt::entity, T>` from `registry.view<T>()` and returns the restore
   closure holding it, which `Save` pushes into `restore_`. The thunk returns the
   closure rather than writing into the `Checkpoint` so that `Entity.h` — where the
   thunks are instantiated — needs no `Checkpoint`, and no Jolt headers.
2. Record every live entity into `entities_`.
3. `physics_->system_.SaveState(cp.physics_)` — defaults to
   `EStateRecorderState::All`, validation off.
4. Record which entities hold a `Collider` into `colliders_`, so `Restore` only
   hands a parked body back to its own entity.
5. For each `Character`, `character_->SaveState(cp.characters_[entity])` — one
   recorder per entity, so a change in iteration order between save and restore
   cannot cross-apply state.
6. Warn about untracked storages.

## Restore

Order matters:

1. **Destroy entities not in `entities_`.** That fires `Engine`'s `on_destroy`
   hooks, which take the Jolt bodies out of the broad phase, so the body set
   matches what the recorder holds before Jolt sees it.
2. **Resurrect entities in `entities_` that are no longer valid** — see
   "Resurrection" below.
3. **Run the component restore closures.** Each one `emplace_or_replace`s its
   saved values and removes `T` from any entity that gained it after the
   checkpoint. Collect-then-remove; do not erase while iterating the view.
   Systems ride this path like any other tracked component.
4. **Hand back the parked bodies** (`Engine::HandBackRetired`): re-add each one
   whose entity this checkpoint knows, destroy the rest.
5. **Restore physics.** `physics_.Rewind()` then
   `physics_->system_.RestoreState(physics_)`; log an error if it returns false.
   Then per character: `Rewind()`, `character_->RestoreState(rec)`.

The next `Update()` pushes the restored `Transform`s into the bodies as usual.
Both halves were captured at the same instant, so it writes back the values Jolt
already has and nothing drifts.

## Resurrection

A destroyed entity is not rebuilt, it is *retained*. Three pieces:

- **The handle.** `registry.create(hint)` returns exactly the hint when its index
  is free, and after `Restore` destroys everything spawned since the capture, every
  captured index is free — anything that recycled one was spawned after the capture.
- **The components.** They are already in the checkpoint by value, and land on the
  revived handle like on any other.
- **The body.** A Jolt body cannot be recreated from a copy: the id, the shape and
  the mass overrides die with it. So it never dies. `Engine`'s `on_destroy` hooks
  for `Collider` and `Character` take the body out of the broad phase and park the
  component in `retired_colliders_` / `retired_characters_` instead of destroying
  it. For a `Character` the parked copy's `JPH::Ref` is what holds the body open.
  `BodyManager::SaveState` only saves bodies passing `IsInBroadPhase()`, so a
  parked body is invisible to later checkpoints and costs no simulation time.

`Restore` re-adds each parked body whose entity the checkpoint knows, before
`RestoreState` so the stream can find it, and destroys the rest. `Save` and
`~Engine` destroy whatever is still parked; retention only starts at the first
`Save`, so a game that never checkpoints is unaffected.

The cost is memory: every physics entity destroyed since the last `Save` or
`Restore` still holds its body.

## Constraints and known ceilings

- **An entity destroyed after the checkpoint comes back** — see "Resurrection"
  above — but only for the newest capture: `Save` and `Restore` both discard the
  parked handles, so an older `Checkpoint` restored afterwards finds those bodies
  destroyed.
- **A body added after the checkpoint keeps its current state.** It is simply
  absent from the recorder's stream. Documented, not solved.
- **`Save` / `Restore` must not be called during the physics step.** Any system
  phase or a point between frames is fine.
- **A checkpoint does not outlive the run.** It aliases GPU resources and Jolt
  body ids by value.
- **Systems that are neither `Track`ed nor `Ignore`d keep their state across a
  restore**, and warn once so the omission is visible.

## Test

`src/taco/checkpoint_test.cpp`, wired into `CMakeLists.txt` next to
`tacoLoaderTest` and `tacoInputTest`, in the same assert-`main` style:

1. Load a small scene with a transform and a sphere collider.
2. `Save()`.
3. Perturb both sides: overwrite the `Transform` and move the Jolt body directly
   through `Collider::SetPosition`.
4. `Restore()`.
5. Assert the `Transform` and `Collider::GetPosition()` are both back at the
   captured values.

Driving it this way exercises the component path and the Jolt path without
running the main loop or needing access to the private `Update()`, so no API is
widened for the sake of the test.
