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

template<class T> void Track();  // register a component type for checkpointing
template<class T> void Ignore(); // suppress the untracked-storage warning for T
```

`Checkpoint` (`src/taco/Checkpoint.h`) is move-only, because
`JPH::StateRecorderImpl` is:

```cpp
class Checkpoint {
    friend class Engine;

    std::vector<entt::entity> entities_;                          // alive at capture
    std::vector<std::function<void(entt::registry &)>> restore_;  // one per tracked type
    std::vector<std::tuple<entt::id_type,                         // system storage id
                           entt::entity,
                           std::shared_ptr<System>>> systems_;
    JPH::StateRecorderImpl physics_;
    std::map<entt::entity, JPH::StateRecorderImpl> characters_;

public:
    Checkpoint() = default;
    Checkpoint(Checkpoint &&) = default;
    Checkpoint &operator=(Checkpoint &&) = default;
};
```

A checkpoint is reusable: `Restore` never consumes it, so the same object can
reset the engine any number of times.

`Save` and `Restore` are `Engine` members because they need
`physics_->system_`, and `Engine` is already a `friend` of `PhysicsEngine`.

## Which components are tracked

The engine registers all of its own components in its constructor:

`Transform`, `Link`, `Camera`, `Mesh`, `Material`, `BoundingBox`, `Sunlight`,
`Environment`, `Sky`.

Three types are on the ignore list because they cannot be copied and do not need
to be:

| Type | Why not copied | How its state survives |
|---|---|---|
| `Collider` | move-only RAII handle around a `JPH::BodyID` | Jolt `SaveState` + `Transform` |
| `Character` | move-only, owns a `unique_ptr<JPH::Character>` | Jolt `SaveState` + `Transform` |
| `std::shared_ptr<System>` | polymorphic, shared identity | `System::Clone` (below) |

Game-side components need one `engine.Track<MyComponent>()` call. This cannot be
made automatic: EnTT 3.15's type-erased storage API is `value(entity)` (opaque
read) and `push(entity, const void *)` (copy-construct), both of which require a
destination storage of the same *static* type, and `basic_sparse_set` exposes no
clone-factory virtual. To keep a forgotten `Track` from being a silent hole,
`Save` walks `registry.storage()` and logs a warning naming
`pool.type().name()` for every storage that is neither tracked nor ignored.

## System state

Systems keep state outside the ECS — `LookSystem::yaw` / `pitch`, for instance.
Restoring the `shared_ptr` would restore the pointer, not the state, so the view
would not rewind with the world.

`System` grows one hook:

```cpp
virtual std::shared_ptr<System> Clone() const { return nullptr; }
```

Opt-in per subclass and one line each
(`return std::make_shared<LookSystem>(*this);`). Systems returning the default
`nullptr` are left alone by `Restore`, keeping their running state.

Systems live in **named** storages — `Loader::AttachSystems` uses
`registry.storage<std::shared_ptr<System>>(entt::hashed_string{name})` — so the
storage id is captured alongside the clone and restored into the same storage.

## Capture

1. For each tracked type `T`: build a `std::map<entt::entity, T>` from
   `registry.view<T>()` and move it into a restore closure stored in `restore_`.
2. Record every live entity into `entities_`.
3. Walk every storage whose type is `std::shared_ptr<System>` (the same scan
   `Engine::Update`'s `visit_systems` does, since there is one storage per system
   name), call `Clone()` on each, and keep the non-null results with their
   storage id and entity.
4. `physics_->system_.SaveState(cp.physics_)` — defaults to
   `EStateRecorderState::All`, validation off.
5. For each `Character`, `character_->SaveState(cp.characters_[entity])` — one
   recorder per entity, so a change in iteration order between save and restore
   cannot cross-apply state.
6. Warn about untracked storages.

## Restore

Order matters:

1. **Destroy entities not in `entities_`.** Their `Collider` / `Character`
   destructors remove the Jolt bodies, so the body set matches what the recorder
   holds before Jolt sees it.
2. **Run the component restore closures.** Each one `emplace_or_replace`s its
   saved values and removes `T` from any entity that gained it after the
   checkpoint. Collect-then-remove; do not erase while iterating the view.
3. **Restore the systems.** For each saved entry, get
   `registry.storage<std::shared_ptr<System>>(id)`, drop the current entry if
   present, and emplace a *fresh* `Clone()` of the saved system so the checkpoint
   stays reusable.
4. **Restore physics.** `physics_.Rewind()` then
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
  the mass overrides die with it. So it never dies. `Engine` connects `on_destroy`
  hooks for `Collider` and `Character` that take the body out of the broad phase and
  move the handle into `retired_colliders_` / `retired_characters_`. Moving empties
  the handle's `physics_`, which is what both destructors test, so the husk left on
  the entity destructs without touching the body. `BodyManager::SaveState` only saves
  bodies passing `IsInBroadPhase()`, so a parked body is invisible to later
  checkpoints and costs no simulation time.

`Restore` re-adds each parked body whose entity the checkpoint knows, before
`RestoreState` so the stream can find it, and destroys the rest. `Save` and
`~Engine` destroy whatever is still parked; retention only starts at the first
`Save`, so a game that never checkpoints is unaffected.

The cost is memory: every physics entity destroyed since the last `Save` or
`Restore` still holds its body.

## Constraints and known ceilings

- **An entity destroyed after the checkpoint comes back** — see "Resurrection"
  below — but only for the newest capture: `Save` and `Restore` both discard the
  parked handles, so an older `Checkpoint` restored afterwards finds those bodies
  destroyed.
- **A body added after the checkpoint keeps its current state.** It is simply
  absent from the recorder's stream. Documented, not solved.
- **`Save` / `Restore` must not be called during the physics step.** Any system
  phase or a point between frames is fine.
- **A checkpoint does not outlive the run.** It aliases GPU resources and Jolt
  body ids by value.
- **Systems without a `Clone()` override keep their state across a restore.**

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
