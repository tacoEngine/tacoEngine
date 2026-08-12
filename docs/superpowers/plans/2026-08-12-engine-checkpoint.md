# Engine Checkpointing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let `taco::Engine` capture an in-memory snapshot of every entity, its components and the Jolt simulation, and reset back to it.

**Architecture:** A move-only `Checkpoint` object holds one type-erased restore closure per registered component type, clones of the systems that opt in, and Jolt `StateRecorderImpl` streams. `Engine::Save()` fills it; `Engine::Restore()` destroys entities spawned since, replays the closures, swaps in fresh system clones, then hands the recorders back to Jolt. GPU handles are copied by value because nothing in the engine ever unloads them.

**Tech Stack:** C++20, EnTT 3.15 (ECS), Jolt Physics (`PhysicsSystem::SaveState`/`RestoreState`, `StateRecorderImpl`), raylib types, yal logging (`logging::Logger`), CMake + Ninja.

## Global Constraints

- Spec: `docs/superpowers/specs/2026-08-12-engine-checkpoint-design.md`. Read it before starting.
- C++20, `-fno-rtti`. Never use `dynamic_cast` or `typeid` — use EnTT's `entt::type_id<T>()` / `pool.type()` for type identity.
- Every new file starts with the project header comment: `// tacoEngine (c) Nikolas Wipper 2026` followed by the MPL-2.0 block copied verbatim from `src/taco/Loader.h`.
- Header guards are bare uppercase macros matching the file name (`CHECKPOINT_H`), matching the rest of `src/taco`.
- Engine code lives in `namespace taco`.
- Logging: `#include <log/log.h>`, then `logging::Logger::Error(...)` / `logging::Logger::Warning(...)`, message prefixed `"[checkpoint]: "` (mirrors the `"[jolt]: "` prefix in `src/taco/misc/Log.cpp`).
- Build directory already exists and is configured: build with `cmake --build cmake-build-debug --target <target>`. Do not re-run `cmake -S . -B ...`.
- Tests are plain `assert`-based `main()` executables, one per file, registered in `CMakeLists.txt` next to `tacoLoaderTest`. No test framework.
- `Checkpoint` is move-only and must stay reusable: `Restore` may be called any number of times on the same object.

---

## File Structure

- **Create** `src/taco/Checkpoint.h` — the `Checkpoint` data holder. No logic; `Engine` is its only friend.
- **Modify** `src/taco/comp/System.h` — add the `Clone()` hook.
- **Modify** `src/taco/Engine.h` — `Save`, `Restore`, the `Track<T>`/`Ignore<T>` templates and their two member maps.
- **Create** `src/taco/Checkpoint.cpp` — `Engine::Save` / `Engine::Restore`. Kept out of `Engine.cpp` (already ~450 lines of render/update orchestration); this file owns checkpointing only.
- **Modify** `src/taco/Engine.cpp` — register the built-in tracked/ignored types in the constructor.
- **Create** `src/taco/checkpoint_test.cpp` — the self-check.
- **Modify** `CMakeLists.txt` — add `Checkpoint.h`/`Checkpoint.cpp` to `TACO_SOURCES`, add the `tacoCheckpointTest` target.
- **Modify** `CLAUDE.md` — document the feature (Task 4).

---

### Task 1: Checkpoint object, component capture and restore

Snapshot and restore plain component data. No physics, no systems yet.

**Files:**
- Create: `src/taco/Checkpoint.h`
- Create: `src/taco/Checkpoint.cpp`
- Modify: `src/taco/Engine.h`
- Modify: `src/taco/Engine.cpp:28-42` (constructor)
- Modify: `CMakeLists.txt:20-40` (`TACO_SOURCES`), `CMakeLists.txt:49-54` (test targets)
- Test: `src/taco/checkpoint_test.cpp`

**Interfaces:**
- Consumes: `taco::Engine::registry` (public `entt::registry`), `taco::Loader::LoadScene` / `Loader::Resolve` for test scene setup.
- Produces:
  - `class taco::Checkpoint` — move-only, default-constructible, `friend class Engine`.
  - `taco::Checkpoint taco::Engine::Save()`
  - `void taco::Engine::Restore(Checkpoint &cp)`
  - `template<class T> void taco::Engine::Track()`
  - `template<class T> void taco::Engine::Ignore()`

- [ ] **Step 1: Write the failing test**

Create `src/taco/checkpoint_test.cpp`:

```cpp
// tacoEngine (c) Nikolas Wipper 2026

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <cassert>
#include <cstdio>
#include <fstream>

#include "taco/Checkpoint.h"
#include "taco/Engine.h"
#include "taco/Loader.h"
#include "taco/comp/Transform.h"

int main() {
    const char *scene =
        "{ \"entities\": {"
        "  \"ball\": { \"Transform\": {\"position\":[0,10,0]},"
        "              \"Collider\": {\"sphere\": 1.0} }"
        "} }";
    { std::ofstream out("checkpoint_test_scene.json"); out << scene; }

    taco::Engine engine;
    taco::Loader loader(engine, ".");
    loader.LoadScene("checkpoint_test_scene.json");

    const entt::entity ball = loader.Resolve("ball");
    assert(ball != entt::null);

    taco::Checkpoint cp = engine.Save();

    // Component data changes, plus an entity that did not exist at capture.
    engine.registry.get<taco::Transform>(ball).position = {9, 9, 9};
    const entt::entity spawned = engine.registry.create();
    engine.registry.emplace<taco::Transform>(spawned, Vector3{1, 1, 1}, taco::Rotation(), Vector3{0, 0, 0});

    engine.Restore(cp);

    const taco::Transform &t = engine.registry.get<taco::Transform>(ball);
    assert(t.position.x == 0 && t.position.y == 10 && t.position.z == 0);
    assert(!engine.registry.valid(spawned));

    // A checkpoint is reusable.
    engine.registry.get<taco::Transform>(ball).position = {1, 1, 1};
    engine.Restore(cp);
    assert(engine.registry.get<taco::Transform>(ball).position.y == 10);

    std::remove("checkpoint_test_scene.json");
    printf("checkpoint test: ok\n");
}
```

Add to `CMakeLists.txt` directly after the `tacoInputTest` target:

```cmake
add_executable(tacoCheckpointTest src/taco/checkpoint_test.cpp)
target_link_libraries(tacoCheckpointTest tacoEngine)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build cmake-build-debug --target tacoCheckpointTest`
Expected: FAIL — `fatal error: 'taco/Checkpoint.h' file not found`.

- [ ] **Step 3: Create `src/taco/Checkpoint.h`**

```cpp
// tacoEngine (c) Nikolas Wipper 2026

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef CHECKPOINT_H
#define CHECKPOINT_H

#include <functional>
#include <map>
#include <memory>
#include <tuple>
#include <vector>

#include <entt/entt.hpp>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/StateRecorderImpl.h>

#include "comp/System.h"

namespace taco {
/// In-memory snapshot of the engine state. Move-only (JPH::StateRecorderImpl is),
/// reusable, and only valid for the run that produced it: it aliases GPU handles
/// and Jolt body ids by value.
class Checkpoint {
    friend class Engine;

    /// Entities alive at capture.
    std::vector<entt::entity> entities_;
    /// One closure per tracked component type, holding that type's saved values.
    std::vector<std::function<void(entt::registry &)>> restore_;
    /// Cloned systems: storage id (systems live in named storages), entity, clone.
    std::vector<std::tuple<entt::id_type, entt::entity, std::shared_ptr<System>>> systems_;
    JPH::StateRecorderImpl physics_;
    /// Keyed by entity so iteration order cannot cross-apply state between characters.
    std::map<entt::entity, JPH::StateRecorderImpl> characters_;

public:
    Checkpoint() = default;
    Checkpoint(Checkpoint &&) = default;
};
}

#endif //CHECKPOINT_H
```

Note: no move-assignment operator. `JPH::StateRecorderImpl` declares a move constructor and therefore has no move assignment, so `operator=(Checkpoint &&) = default` would be defined as deleted. `Save()` returns by value (NRVO, else the move constructor), which is all we need.

- [ ] **Step 4: Add the API to `src/taco/Engine.h`**

Add the include next to the existing ones:

```cpp
#include "Checkpoint.h"
```

Add these private members alongside `Config config_;`:

```cpp
    std::map<entt::id_type, std::function<void(const entt::registry &, Checkpoint &)>> tracked_;
    std::set<entt::id_type> ignored_;
```

and `#include <map>`, `#include <set>`, `#include <functional>` at the top.

Add to the public section, after `Config SwapConfig(Config con);`:

```cpp
    /// Capture the current engine state. See Restore.
    Checkpoint Save();
    /// Reset the engine back to cp. Non-const: the Jolt recorders need Rewind().
    void Restore(Checkpoint &cp);

    /// Register a component type for checkpointing. Engine components are
    /// registered in the constructor; game components need one call each.
    template<class T>
    void Track() {
        tracked_[entt::type_id<T>().hash()] = [](const entt::registry &registry, Checkpoint &cp) {
            std::map<entt::entity, T> data;
            for (auto [entity, component] : registry.view<const T>().each())
                data.emplace(entity, component);

            cp.restore_.emplace_back([data = std::move(data)](entt::registry &reg) {
                // Drop the component from entities that gained it after the capture.
                std::vector<entt::entity> stale;
                for (const entt::entity entity : reg.view<T>())
                    if (!data.count(entity)) stale.push_back(entity);
                for (const entt::entity entity : stale) reg.remove<T>(entity);

                for (const auto &[entity, component] : data)
                    reg.emplace_or_replace<T>(entity, component);
            });
        };
    }

    /// Exclude a type from checkpointing without tripping the untracked warning.
    template<class T>
    void Ignore() {
        ignored_.insert(entt::type_id<T>().hash());
    }
```

- [ ] **Step 5: Create `src/taco/Checkpoint.cpp`**

```cpp
// tacoEngine (c) Nikolas Wipper 2026

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "Checkpoint.h"

#include <set>
#include <vector>

#include <log/log.h>

#include "Engine.h"

namespace taco {
Checkpoint Engine::Save() {
    Checkpoint cp;

    for (auto &[_, capture] : tracked_)
        capture(registry, cp);

    for (const entt::entity entity : registry.view<entt::entity>())
        cp.entities_.push_back(entity);

    return cp;
}

void Engine::Restore(Checkpoint &cp) {
    // Entities spawned after the capture go first: their Collider/Character
    // destructors remove the Jolt bodies, so the body set matches the recorder.
    const std::set<entt::entity> alive(cp.entities_.begin(), cp.entities_.end());
    std::vector<entt::entity> spawned;
    for (const entt::entity entity : registry.view<entt::entity>())
        if (!alive.count(entity)) spawned.push_back(entity);
    for (const entt::entity entity : spawned)
        registry.destroy(entity);

    for (const entt::entity entity : cp.entities_)
        if (!registry.valid(entity))
            logging::Logger::Error("[checkpoint]: entity destroyed since the capture cannot be restored");

    for (auto &restore : cp.restore_)
        restore(registry);
}
}
```

- [ ] **Step 6: Register the built-in types in the `Engine` constructor**

In `src/taco/Engine.cpp`, at the end of the constructor body (after `ReloadGBuffers();`):

```cpp
    Track<Transform>();
    Track<Link>();
    Track<Camera>();
    Track<Mesh>();
    Track<Material>();
    Track<BoundingBox>();
    Track<Sunlight>();
    Track<Environment>();
    Track<Sky>();

    // Not copyable, and not needed: the entity storage is the registry's own,
    // Collider/Character state comes from Jolt, systems from System::Clone.
    Ignore<entt::entity>();
    Ignore<Collider>();
    Ignore<Character>();
    Ignore<std::shared_ptr<System>>();
```

`Engine.cpp` already includes `comp/Camera.h`, `comp/Lights.h` and `comp/System.h`, so every type is complete here — no new includes needed in `Engine.h`.

- [ ] **Step 7: Add the new sources to `CMakeLists.txt`**

Add to `TACO_SOURCES`, next to `src/taco/Engine.cpp`:

```cmake
        src/taco/Checkpoint.cpp
        src/taco/Checkpoint.h
```

- [ ] **Step 8: Run the test**

Run: `cmake --build cmake-build-debug --target tacoCheckpointTest && ./cmake-build-debug/tacoCheckpointTest`
Expected: PASS, printing `checkpoint test: ok`.

- [ ] **Step 9: Commit**

```bash
git add src/taco/Checkpoint.h src/taco/Checkpoint.cpp src/taco/Engine.h src/taco/Engine.cpp src/taco/checkpoint_test.cpp CMakeLists.txt
git commit -m "Add engine checkpoint with component capture and restore"
```

---

### Task 2: System cloning

Systems keep state outside the ECS (`LookSystem::yaw`/`pitch`). Restoring the `shared_ptr` restores the pointer, not the state, so systems opt in with a `Clone()` override.

**Files:**
- Modify: `src/taco/comp/System.h`
- Modify: `src/taco/Checkpoint.cpp` (`Engine::Save`, `Engine::Restore`)
- Test: `src/taco/checkpoint_test.cpp`

**Interfaces:**
- Consumes: `taco::Checkpoint::systems_` (declared in Task 1), `Engine::Save`, `Engine::Restore`.
- Produces: `virtual std::shared_ptr<System> taco::System::Clone() const` — returns `nullptr` by default, meaning "not checkpointed".

- [ ] **Step 1: Write the failing test**

In `src/taco/checkpoint_test.cpp`, add above `main()`:

```cpp
struct CountSystem : taco::System {
    int value = 0;

    std::shared_ptr<taco::System> Clone() const override {
        return std::make_shared<CountSystem>(*this);
    }
};
```

and `#include <entt/core/hashed_string.hpp>` plus `#include "taco/comp/System.h"` at the top.

Inside `main()`, immediately after `assert(ball != entt::null);`:

```cpp
    // Systems live in named storages, exactly like Loader::AttachSystems creates them.
    auto &system_storage =
        engine.registry.storage<std::shared_ptr<taco::System>>(entt::hashed_string{"CountSystem"});
    const auto counter = std::make_shared<CountSystem>();
    system_storage.emplace(ball, counter);
    counter->value = 5;
```

and after the first `engine.Restore(cp);`, before the reusability block:

```cpp
    // The clone is a fresh object; the original pointer is replaced, not mutated.
    const auto &restored =
        static_cast<CountSystem &>(*engine.registry.storage<std::shared_ptr<taco::System>>(
            entt::hashed_string{"CountSystem"}).get(ball));
    assert(restored.value == 5);
```

with the mutation between `Save()` and `Restore()` — add `counter->value = 99;` next to the existing `position = {9, 9, 9};` line.

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build cmake-build-debug --target tacoCheckpointTest`
Expected: FAIL — `'Clone' marked 'override' but does not override any member functions`.

- [ ] **Step 3: Add the hook to `src/taco/comp/System.h`**

Add `#include <memory>` and, inside `class System`:

```cpp
    /// Return a copy of this system to make its state part of a Checkpoint.
    /// The default (nullptr) means the system keeps its state across a restore.
    /// Override with: return std::make_shared<MySystem>(*this);
    virtual std::shared_ptr<System> Clone() const { return nullptr; }
```

- [ ] **Step 4: Capture the clones in `Engine::Save`**

In `src/taco/Checkpoint.cpp`, insert into `Engine::Save` between the `tracked_` loop and the `entities_` loop:

```cpp
    // Systems live in one named storage per system name, so scan every storage
    // holding shared_ptr<System> — the same walk Engine::Update's visit_systems does.
    for (auto [id, pool] : registry.storage()) {
        if (pool.type() != entt::type_id<std::shared_ptr<System>>())
            continue;

        auto system_view = entt::basic_view{registry.storage<std::shared_ptr<System>>(id)};
        for (auto [entity, system] : system_view.each())
            if (std::shared_ptr<System> clone = system->Clone())
                cp.systems_.emplace_back(id, entity, std::move(clone));
    }
```

- [ ] **Step 5: Restore the clones in `Engine::Restore`**

Append to `Engine::Restore`, after the `cp.restore_` loop:

```cpp
    for (const auto &[id, entity, system] : cp.systems_) {
        if (!registry.valid(entity)) continue;

        auto &storage = registry.storage<std::shared_ptr<System>>(id);
        if (storage.contains(entity)) storage.erase(entity);
        // Clone again so the checkpoint stays usable for the next restore.
        storage.emplace(entity, system->Clone());
    }
```

- [ ] **Step 6: Run the test**

Run: `cmake --build cmake-build-debug --target tacoCheckpointTest && ./cmake-build-debug/tacoCheckpointTest`
Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add src/taco/comp/System.h src/taco/Checkpoint.cpp src/taco/checkpoint_test.cpp
git commit -m "Checkpoint system state via System::Clone"
```

---

### Task 3: Jolt physics state

**Files:**
- Modify: `src/taco/Checkpoint.cpp` (`Engine::Save`, `Engine::Restore`)
- Test: `src/taco/checkpoint_test.cpp`

**Interfaces:**
- Consumes: `Checkpoint::physics_`, `Checkpoint::characters_` (Task 1), `PhysicsEngine::system_` and `Character::character_` — both private, both reachable because `PhysicsEngine` and `Character` declare `friend class Engine`.
- Produces: nothing new in the public API.

- [ ] **Step 1: Write the failing test**

In `src/taco/checkpoint_test.cpp`, add `#include <cmath>` and `#include "taco/Physics.h"`.

The Jolt body sits at the origin until `Engine::Update` pushes the transform into it, so place it explicitly before capturing. Immediately before `taco::Checkpoint cp = engine.Save();`:

```cpp
    engine.registry.get<taco::Collider>(ball).SetPosition({0, 10, 0});
```

Next to the other mutations after `Save()`:

```cpp
    engine.registry.get<taco::Collider>(ball).SetPosition({9, 9, 9});
```

And after the first `engine.Restore(cp);`:

```cpp
    const Vector3 body = engine.registry.get<taco::Collider>(ball).GetPosition();
    assert(std::fabs(body.x) < 0.001f);
    assert(std::fabs(body.y - 10.f) < 0.001f);
    assert(std::fabs(body.z) < 0.001f);
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build cmake-build-debug --target tacoCheckpointTest && ./cmake-build-debug/tacoCheckpointTest`
Expected: FAIL — assertion on `body.y - 10.f`, because the body is still at (9,9,9); nothing restores Jolt yet.

- [ ] **Step 3: Save the physics state**

In `src/taco/Checkpoint.cpp`, add `#include "Physics.h"` and append to `Engine::Save`, just before `return cp;`:

```cpp
    // Jolt's own rollback support: global state, bodies, contacts and constraints.
    physics_->system_.SaveState(cp.physics_);

    for (auto [entity, character] : registry.view<Character>().each())
        character.character_->SaveState(cp.characters_[entity]);
```

- [ ] **Step 4: Restore the physics state**

Append to `Engine::Restore`, after the system loop (it must run last — the bodies have to exist and the spawned entities have to be gone before Jolt reads the stream):

```cpp
    cp.physics_.Rewind();
    if (!physics_->system_.RestoreState(cp.physics_))
        logging::Logger::Error("[checkpoint]: failed to restore the physics state");

    for (auto &[entity, recorder] : cp.characters_) {
        if (!registry.valid(entity) || !registry.all_of<Character>(entity)) continue;

        recorder.Rewind();
        registry.get<Character>(entity).character_->RestoreState(recorder);
    }
```

`Rewind()` only moves the read head, so the recorders survive for the next `Restore`.

- [ ] **Step 5: Run the test**

Run: `cmake --build cmake-build-debug --target tacoCheckpointTest && ./cmake-build-debug/tacoCheckpointTest`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add src/taco/Checkpoint.cpp src/taco/checkpoint_test.cpp
git commit -m "Checkpoint Jolt physics state"
```

---

### Task 4: Untracked-component warning and docs

A game component that never got a `Track<T>()` call would silently not restore. Make it loud.

**Files:**
- Modify: `src/taco/Checkpoint.cpp` (`Engine::Save`)
- Modify: `CLAUDE.md`
- Test: `src/taco/checkpoint_test.cpp`

**Interfaces:**
- Consumes: `Engine::tracked_`, `Engine::ignored_` (Task 1).
- Produces: nothing new in the public API.

- [ ] **Step 1: Write the failing test**

In `src/taco/checkpoint_test.cpp`, add above `main()`:

```cpp
struct Untracked {
    int value;
};
```

In `main()`, after the `CountSystem` setup and before `engine.Save()`:

```cpp
    // Not registered with Track<>: Save must warn, and the value must not restore.
    engine.registry.emplace<Untracked>(ball, 1);
```

After the first `engine.Restore(cp);` (mutate it alongside the other post-capture edits with `engine.registry.get<Untracked>(ball).value = 2;`):

```cpp
    assert(engine.registry.get<Untracked>(ball).value == 2);
```

Also register a tracked game component to prove `Track<T>()` works end to end — add above `main()`:

```cpp
struct Tracked {
    int value;
};
```

and in `main()`, before `Save()`:

```cpp
    engine.Track<Tracked>();
    engine.registry.emplace<Tracked>(ball, 1);
```

with `engine.registry.get<Tracked>(ball).value = 2;` among the post-capture mutations and, after the restore:

```cpp
    assert(engine.registry.get<Tracked>(ball).value == 1);
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build cmake-build-debug --target tacoCheckpointTest && ./cmake-build-debug/tacoCheckpointTest`
Expected: PASS on the asserts but NO warning is logged for `Untracked`. Confirm by eye that the output contains no `[checkpoint]` line — that missing line is the failure this step drives out.

- [ ] **Step 3: Add the warning**

At the top of `Engine::Save` in `src/taco/Checkpoint.cpp`, before the `tracked_` loop:

```cpp
    // A component nobody registered would silently not restore. Say so.
    for (auto [id, pool] : registry.storage()) {
        const entt::id_type type = pool.type().hash();
        if (tracked_.count(type) || ignored_.count(type)) continue;

        logging::Logger::Warning("[checkpoint]: untracked component " + std::string(pool.type().name())
                                 + ", call Engine::Track<T>() to include it");
    }
```

`pool.type().hash()` is the type hash, not the storage id, so this covers the named system storages too (they are ignored via `Ignore<std::shared_ptr<System>>()`).

- [ ] **Step 4: Run the test**

Run: `cmake --build cmake-build-debug --target tacoCheckpointTest && ./cmake-build-debug/tacoCheckpointTest`
Expected: PASS, and the output now contains one `[checkpoint]: untracked component ...Untracked...` warning and no other `[checkpoint]` warnings.

- [ ] **Step 5: Document it in `CLAUDE.md`**

Add a section after the "Physics (Jolt wrapper...)" section:

```markdown
## Checkpointing (`Checkpoint.{h,cpp}`)

`Engine::Save()` returns a move-only `Checkpoint`; `Engine::Restore(cp)` resets the
engine back to it. In-memory only, valid for the current run, and reusable.

- Component data is copied per registered type. The engine registers its own nine
  components in its constructor; game components need one `engine.Track<T>()` call,
  and `Save` warns about any storage that is neither tracked nor `Ignore<T>()`d.
- `Collider`/`Character` are not copied: Jolt's own `PhysicsSystem::SaveState` /
  `RestoreState` and `CharacterBase::SaveState` carry the simulation state.
- Systems opt in by overriding `System::Clone()` (default `nullptr` = state kept).
- `Restore` destroys entities spawned since the capture; entities *destroyed* since
  cannot come back (their GPU/Jolt handles are gone) and are logged as an error.
- Call `Save`/`Restore` outside the physics step — any system phase, or between frames.
```

Also add `Checkpoint.{h,cpp}   # in-memory state snapshot + restore` to the `src/taco/` layout block, and add a line to "Gotchas worth remembering":

```markdown
- A `Checkpoint` aliases GPU handles by value — it is only valid for the run that made it.
```

- [ ] **Step 6: Commit**

```bash
git add src/taco/Checkpoint.cpp src/taco/checkpoint_test.cpp CLAUDE.md
git commit -m "Warn about untracked components and document checkpointing"
```

---

## Self-Review

**Spec coverage:**

| Spec section | Task |
|---|---|
| API (`Save`/`Restore`/`Track`/`Ignore`), `Checkpoint` shape | 1 |
| Tracked component list, ignore list | 1 |
| Untracked-storage warning | 4 |
| `System::Clone`, named system storages | 2 |
| Capture steps 1–3, 6 | 1, 2, 4 |
| Capture steps 4–5 (Jolt + characters) | 3 |
| Restore step 1 (destroy spawned) | 1 |
| Restore step 2 (components) | 1 |
| Restore step 3 (systems, re-clone) | 2 |
| Restore step 4 (Jolt, characters) | 3 |
| Ceilings: destroyed-entity logging | 1 |
| Test | 1–4 (grown across tasks) |

**Types:** `Checkpoint` members are declared once in Task 1 and only read by Tasks 2–4. `Clone()` has the same signature in `System.h` (Task 2 step 3), the test's `CountSystem` (Task 2 step 1) and both call sites (Task 2 steps 4–5). `Track`/`Ignore`/`Save`/`Restore` signatures match between `Engine.h` and `Checkpoint.cpp`.

**Ceiling not covered by a task, by design:** resurrecting an entity destroyed after the capture. `Restore` logs it (Task 1, step 5) and Jolt reports the body mismatch (Task 3, step 4); the spec explicitly leaves it unsolved.
