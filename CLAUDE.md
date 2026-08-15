# tacoEngine

A small C++20 game engine: **EnTT** (ECS) + **Jolt** (physics) + **tacoRender**
(deferred PBR renderer, a C library wrapping raylib). Author: Nikolas Wipper. MPL-2.0.

The engine itself is ~1000 lines in `src/taco`. Everything heavy lives in `ext/`
submodules. For the renderer internals see **[docs/tacoRender.md](docs/tacoRender.md)**.

## Layout

```
src/taco/
  Engine.{h,cpp}      # main loop, deferred render orchestration, system dispatch
  Entity.h            # the entity handle: destroy/add/get/remove
  Physics.{h,cpp}     # Jolt wrapper: PhysicsEngine, Collider, Character, layers
  Checkpoint.{h,cpp}  # the snapshot itself: capture/restore of its own ECS + Jolt data
  Config.h            # render/quality settings struct (hot-swappable)
  Graphics.h          # just re-exports <tacoRender.h>
  comp/               # ECS components (plain structs unless noted)
    Transform.h       # Transform{position, rotation, velocity}, Link
    Camera.h          # Camera{fov}
    Lights.{h,cpp}    # Sunlight, Environment (IBL), Sky
    System.{h,cpp}    # the behaviour component; five phase hooks, added like any component
  misc/
    Rotation.{h,cpp}  # euler-angle rotation with cached quaternion
    Debug.{h,cpp}     # RaylibDebugRenderer: Jolt debug draw -> raylib
    Log.{h,cpp}       # routes Jolt Trace/Assert into yal logger
game/                 # a consumer app (not part of the library); adds `..` as subdir
```

**Build:** CMake, `-fno-rtti`, C++20. `tacoEngine` links `EnTT`, `tacoRender`,
`yal` (logging, `#include <log/log.h>` → `logging::Logger`), `Jolt`. `src` is a public
include dir, so downstream uses `#include "taco/..."`. Submodules: entt, JoltPhysics,
rapidjson (linked at top level but unused by the core lib), tacoRender (→ raylib), yal.

## Core model

Everything is an **entity**. Create one with `Engine::Create()`, which returns a
`taco::Entity` (`Entity.h`) — a cheap, copyable handle (`Valid`, `Destroy`, `Add<T>`,
`Get<T>`, `Has<T>`, `Remove<T>`, `id`, `engine`) you can store in a component. You attach:
- data components (`Transform`, `Camera`, `Mesh`, `Material`, `BoundingBox`, `Sunlight`,
  `Environment`, `Sky`, `Collider`, `Character`, `Link`) with `entity.Add<T>(args...)`, and
- behaviour as a subclass of **`taco::System`** (comp/System.h) with five virtual phase
  hooks. A system is stored **by value** like any other component; `Entity::Add` detects the
  `System` base class and registers the type's dispatch automatically, so consumer code
  never calls `RegisterSystem` itself. See `game/src/*System.*` for real subclasses (Movement,
  Look, Aim, ThirdPerson, …).

`Entity::Add<T>` is the **only** path a component has into the registry, which makes it the
one place that sees every type in the game. It uses that twice: to register a `System`'s
dispatch hooks, and to register the type's checkpoint capture (see Checkpointing). Both are
`if constexpr` on `T`, so they cost nothing for types they don't apply to.

`Engine::registry_` is **private** — consumers never touch it. Iterate entities with
`Engine::Each<Ts...>(fn)`, where `fn` is `(Entity, Ts &...)`.

`Mesh`/`Material`/`BoundingBox`/`Camera3D` etc. are **raylib** types used directly.
`Rotation` is stored as **euler radians** (subclass of `Vector3`) and lazily caches a
quaternion; `GetDirection()` = `(0,0,-1)` rotated by it (camera/forward convention).

## The frame (`Engine::Run` → loop)

Order is **Render() first, then Update()**. Delta time is measured with
`steady_clock` in nanoseconds between iterations (`GetDeltaTime()` → seconds).
Note: `accumulator_` is a dead field; `delta_time_` is int64 nanoseconds.

### `Update()` — fixed order

1. Systems `UpdateEarly` → `UpdatePrePhysics` (dispatched over `Engine::system_hooks_`, an
   ordered table of five function pointers per registered system type, called in
   first-attach order; each hook iterates that system type's storage).
2. **ECS → physics:** push each `Collider`/`Character` entity's transform
   (position, rotation-as-quaternion, velocity) into the Jolt body.
3. `physics_->Update(dt)`.
4. **physics → ECS:** pull position/rotation/velocity back onto the transform.
   Characters also call `PostSimulation(0.01f)`.
5. **`Link`** system: copy chosen position/velocity axes from a linked entity's transform
   (per-axis bool flags — used to constrain one entity to another).
6. Systems `UpdatePostPhysics` → `UpdateLate`.

(`UpdateUI` runs later, inside `Render()`'s `BeginDrawing` block.)

### `Render()` — deferred PBR pipeline

Instruments 7 GPU timers (Geometry, Shadow, Shadow-PP, SSAO, Lighting, PP, Blit),
drawn as an on-screen overlay with FPS and drawn/total mesh counts. On window resize →
`ReloadGBuffers()`.

1. **Geometry:** `BeginGBufferMode` → for each `(Transform, Camera)` build a `Camera3D`
   (target = pos + forward), build a `Frustum`, `DrawAllMeshes` into the G-buffer with the
   gbuffer shader (CPU frustum-culls entities that have a `BoundingBox`); optional physics
   debug draw; draw `Sky` skyboxes. `DrawAllMeshes` builds the model matrix as
   `rotate * translate` and **overwrites `material.shader`** with the pass shader.
2. **Shadows:** for each `Sunlight`: 3 cascades (hardcoded). (Re)allocate the shadow map if
   `config_.shadow_map_size` changed; per cascade `BeginShadowMap(lightDir)` + draw all
   meshes (shadow draw passes an **empty frustum = no culling**, marked `fixme`), then
   `FilterShadowMap` (gaussian, `config_.shadow_map_quality` iterations).
3. **SSAO** (if `config_.ssao`): `ApplySSAO`.
4. **Lighting:** `ClearPresenter` → `BeginLightingPass` (additive blend into `back[0]`) →
   `LightSun` per `Sunlight` (dir from rotation, intensity, colour, shadow map) →
   `LightIBL` per `Environment` → `CopyBackground` → `EndLightingPass`.
5. **Post:** `ApplyToneMapping` (Reinhard) → `ApplyGammaCorrection` (`config_.gamma_correction`).
6. **Blit:** `BeginDrawing`, draw `back[0]` to screen, FPS + timing overlay, systems'
   `UpdateUI`, `EndDrawing`. `running_ = !WindowShouldClose()`.

`Config` (Config.h) is hot-swappable via `SwapConfig` (returns the old one): `debug_physics`,
`ssao`, `gamma_correction`, `tone_mapper`, `shadow_map_size` (2048), `cascade_dist` (100),
`shadow_map_quality` (0 = off, n = (5×5)ⁿ gauss).

## Physics (Jolt wrapper, `Physics.{h,cpp}`)

`PhysicsEngine` (owned by `Engine` via `unique_ptr`; Colliders/Characters hold a non-owning
`PhysicsEngine *` back to it) owns the `JPH::PhysicsSystem`, a 10 MB temp allocator, and a
thread pool (`hardware_concurrency - 1`). Two layers only: `NON_MOVING` / `MOVING` (standard Jolt
hello-world filter setup). `Update(dt)` runs `ceil((1/60)/dt)` collision substeps. Jolt
`Trace`/`Assert` are routed to the yal logger (`misc/Log.cpp`).

- `CreateSphereCollider(radius)` — dynamic sphere.
- `CreateMeshCollider(mesh, dynamic=true)` — indexed mesh → `MeshShape` from vertex+index
  lists; non-indexed → `TriangleList`. Static bodies go on `NON_MOVING`.
- `CreateCharacter(height, radius)` — capsule `JPH::Character`, 45° max slope.
- `Collider` / `Character` are **copyable value types with no destructor**, each holding a
  non-owning `PhysicsEngine *` (`Character` also holds a `JPH::Ref<JPH::Character>`). The Jolt
  body is freed by `Engine`'s `on_destroy` hooks when the component is removed or the entity
  destroyed — **not** by the handle; `~Engine` calls `registry_.clear()` first because EnTT
  does not fire `on_destroy` during registry destruction. Get/Set position, rotation
  (quaternion), velocity; `Character::OnGround()` = `IsSupported()`.

## Checkpointing (`Checkpoint.{h,cpp}` + `Engine.cpp`)

`Engine::Save()` returns a move-only `Checkpoint`; `Engine::Restore(cp)` resets the
engine back to it. In-memory only, valid for the current run, and reusable.

Split by ownership: `Checkpoint` captures and puts back its own data
(`CaptureECS`/`RestoreECS`, `CapturePhysics`/`RestorePhysics`, all private, `Engine` is a
friend). `Engine` owns the policy and the sequencing — which types are tracked, the parked
bodies — and lives in `Engine.cpp`. The two physics halves
are separate calls because `Engine::HandBackRetired` has to run between them.

- **There is no `Track<T>()` call.** `Entity::Add<T>` is the only way a component reaches
  the registry, so it registers the type's capture thunk the first time one is added — the
  same trick that auto-registers system dispatch. Nothing is opt-in and nothing can be
  forgotten. `Engine::Ignore<T>()` opts a type back out (order-independent: it also drops an
  already-registered entry).
- The one type `Add` cannot register is a **move-only** component, which cannot be captured
  by value at all. Those are silently left out; nothing warns about them.
- **Systems are ordinary value components, so `Add<MySystem>` checkpoints them like anything
  else** — there is no `System::Clone`. `Ignore<MySystem>()` is how a system keeps its
  running state across a restore.
- `Collider`/`Character` are not copied: Jolt's own `PhysicsSystem::SaveState` /
  `RestoreState` and `CharacterBase::SaveState` carry the simulation state.
- **A body attached since the capture is removed and destroyed by `Restore`.** The physics
  stream does not name it, so leaving it in would mean a body nobody rewinds; `HandBackRetired`
  drops the component first (which parks the body) and then destroys it, since the checkpoint
  never saw that entity holding one. Same rule as any other component gained since the capture.
- **Destroying a physics entity after the first `Save` does not destroy its Jolt body.**
  The `on_destroy` hooks park the component in `retired_colliders_`/`retired_characters_`
  and only take the body out of the broad phase — a body cannot be rebuilt (its id, shape
  and mass overrides die with it), so it is kept. For a `Character` the parked copy's
  `JPH::Ref` is what keeps it alive. `BodyManager::SaveState` skips bodies outside the broad
  phase, so a parked body is invisible to later checkpoints and unsimulated. `Restore`
  re-adds it; `Restore` and `~Engine` destroy whatever is still parked. The cost: every physics
  entity destroyed since the last `Restore` holds its body — `Save` deliberately keeps them,
  since an older `Checkpoint` may still be the one restored. Before the first `Save` nothing
  is retained.
- `Material` is tracked **by value**: only `params[4]` really rewinds. `maps` is a heap
  array the saved copy aliases (per-map texture/colour edits are not rewound) and `shader`
  is overwritten by `DrawAllMeshes` every frame anyway.
- `Sunlight` is the one component with a hand-written capture: only `intensity`, `color`
  and `shadow_casting` round-trip. Its `shadow_map_` owns a GL fbo and heap arrays that
  `Render` unloads on a config change, so restoring a saved copy would double-free them.
  The restore patches the live component in place (`get_or_emplace`, never `emplace_or_replace`),
  so the shadow map survives every restore — a restore per frame reallocates nothing.
- `Restore` destroys entities spawned since the capture, then **resurrects the ones
  destroyed since** under their original handles: `registry.create(hint)` returns the exact
  handle when its index is free, and after the first step it always is. Components and the
  parked body land back on the revived entity, so a `Link` pointing at one stays valid.
- Any number of checkpoints can be alive at once and any one of them can be restored, but
  **the first `Restore` invalidates the others**: it empties the parked-handle maps, freeing
  every body it could not itself revive, so another `Checkpoint` restored afterwards finds
  those bodies destroyed.
- Call `Restore` directly only from outside the loop (before `Run`, or after it returns).
  From inside a `System` phase hook use `RequestRestore(cp)`: it queues the checkpoint and
  `Run` applies it (via `ApplyPendingRestore`) at the end of that frame's `Update`. A direct
  `Restore` there would destroy entities and rewrite the very storages the dispatch is
  iterating. The queued `Checkpoint` must be owned by something `Restore` cannot destroy —
  not by a `System` and not by an entity, since either owner can be freed mid-`Restore`.
  Never call either during the physics step.

## Gotchas worth remembering

- **Render runs before Update** each frame.
- Shadow cascades are hardcoded to 3; shadow-map draw does no frustum culling (`fixme`).
- `DrawAllMeshes` mutates `material.shader` on every entity every draw (shared pass shader).
- Frustum culling only happens for entities that carry a `BoundingBox` component.
- `Rotation` is euler radians with a cached quaternion; `SetFromQuaternion` writes euler and
  lets the cache lazily recompute.
- `accumulator_` is unused; `delta_time_`'s `0.0f` initialiser is cosmetic (it's int64 ns).
- Adding or destroying a component of the type currently being iterated (e.g. inside a system
  phase or an `Each` loop over that type) is UB.
- A `Checkpoint` aliases GPU handles by value — it is only valid for the run that made it.
- A stale `Entity` is normally caught by `Valid()` (entt tags each id with a version, so a
  recycled index fails validation) — **but** the version is a fixed-width counter (12 bits in
  the default 32-bit entt entity); recycle the same index enough times that it wraps and the
  stale handle silently names the new occupant.

## graphify

This project has a knowledge graph at graphify-out/ with god nodes, community structure, and cross-file relationships.

Rules:
- For codebase questions, first run `graphify query "<question>"` when graphify-out/graph.json exists. Use `graphify path "<A>" "<B>"` for relationships and `graphify explain "<concept>"` for focused concepts. These return a scoped subgraph, usually much smaller than GRAPH_REPORT.md or raw grep output.
- If graphify-out/wiki/index.md exists, use it for broad navigation instead of raw source browsing.
- Read graphify-out/GRAPH_REPORT.md only for broad architecture review or when query/path/explain do not surface enough context.
- After modifying code, run `graphify update .` to keep the graph current (AST-only, no API cost).
