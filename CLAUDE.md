# tacoEngine

A small C++20 game engine: **EnTT** (ECS) + **Jolt** (physics) + **tacoRender**
(deferred PBR renderer, a C library wrapping raylib). Author: Nikolas Wipper. MPL-2.0.

The engine itself is ~1000 lines in `src/taco`. Everything heavy lives in `ext/`
submodules. For the renderer internals see **[docs/tacoRender.md](docs/tacoRender.md)**.

## Layout

```
src/taco/
  Engine.{h,cpp}      # main loop, deferred render orchestration, system dispatch
  Physics.{h,cpp}     # Jolt wrapper: PhysicsEngine, Collider, Character, layers
  Config.h            # render/quality settings struct (hot-swappable)
  Graphics.h          # just re-exports <tacoRender.h>
  comp/               # ECS components (plain structs unless noted)
    Transform.h       # Transform{position, rotation, velocity}, Link
    Camera.h          # Camera{fov}
    Lights.{h,cpp}    # Sunlight, Environment (IBL), Sky
    System.h          # System base class — the behaviour/"script" component
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

Everything is an **entity** in the public `Engine::registry`. You attach:
- data components (`Transform`, `Camera`, `Mesh`, `Material`, `BoundingBox`, `Sunlight`,
  `Environment`, `Sky`, `Collider`, `Character`, `Link`), and
- behaviour via a **`std::shared_ptr<System>`** component. `System` (comp/System.h) has
  five virtual phase hooks; subclass it and store the shared_ptr on an entity. See
  `game/src/*System.*` for real subclasses (Movement, Look, Aim, ThirdPerson, …).

`Mesh`/`Material`/`BoundingBox`/`Camera3D` etc. are **raylib** types used directly.
`Rotation` is stored as **euler radians** (subclass of `Vector3`) and lazily caches a
quaternion; `GetDirection()` = `(0,0,-1)` rotated by it (camera/forward convention).

## The frame (`Engine::Run` → loop)

Order is **Render() first, then Update()**. Delta time is measured with
`steady_clock` in nanoseconds between iterations (`GetDeltaTime()` → seconds).
Note: `accumulator_` is a dead field; `delta_time_` is int64 nanoseconds.

### `Update()` — fixed order

1. Systems `UpdateEarly` → `UpdatePrePhysics` (dispatched over every entity holding a
   `shared_ptr<System>`; see `visit_systems` — it scans all storages for that type).
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

`PhysicsEngine` (a `shared_from_this` object; Colliders/Characters hold a shared_ptr back
to it for RAII cleanup) owns the `JPH::PhysicsSystem`, a 10 MB temp allocator, and a thread
pool (`hardware_concurrency - 1`). Two layers only: `NON_MOVING` / `MOVING` (standard Jolt
hello-world filter setup). `Update(dt)` runs `ceil((1/60)/dt)` collision substeps. Jolt
`Trace`/`Assert` are routed to the yal logger (`misc/Log.cpp`).

- `CreateSphereCollider(radius)` — dynamic sphere.
- `CreateMeshCollider(mesh, dynamic=true)` — indexed mesh → `MeshShape` from vertex+index
  lists; non-indexed → `TriangleList`. Static bodies go on `NON_MOVING`.
- `CreateCharacter(height, radius)` — capsule `JPH::Character`, 45° max slope.
- `Collider` / `Character` are **RAII move-only handles** around a Jolt body; the destructor
  removes+destroys the body. Get/Set position, rotation (quaternion), velocity;
  `Character::OnGround()` = `IsSupported()`.

## Gotchas worth remembering

- **Render runs before Update** each frame.
- Shadow cascades are hardcoded to 3; shadow-map draw does no frustum culling (`fixme`).
- `DrawAllMeshes` mutates `material.shader` on every entity every draw (shared pass shader).
- Frustum culling only happens for entities that carry a `BoundingBox` component.
- `Rotation` is euler radians with a cached quaternion; `SetFromQuaternion` writes euler and
  lets the cache lazily recompute.
- `accumulator_` is unused; `delta_time_`'s `0.0f` initialiser is cosmetic (it's int64 ns).
