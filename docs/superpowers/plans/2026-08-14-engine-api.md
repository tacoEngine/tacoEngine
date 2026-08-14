# Engine API Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Put the ECS behind a `taco::Entity` handle, make systems ordinary value components detected through `std::derived_from`, and move Jolt body lifetime from component destructors to the engine.

**Architecture:** A new `Entity` handle carries `Engine *` + `entt::registry *` + `entt::entity` and defines every component operation inline, so it needs no complete `Engine`. `Entity::Add<T>` checks `std::derived_from<T, System>` and, for systems, registers five stateless dispatch thunks with the engine — replacing today's per-phase scan over every storage in the registry. `Collider`/`Character` become copyable value types whose bodies are freed by engine-owned `on_destroy` hooks.

**Tech Stack:** C++20, EnTT 3.15, Jolt Physics, tacoRender (raylib), yal logging, CMake, rapidjson.

**Spec:** `docs/superpowers/specs/2026-08-13-engine-api-design.md`

**Branch:** `feat/engine-api` (already created off `master`; the spec is already committed there).

## Global Constraints

- C++20, `-fno-rtti`. No RTTI-dependent constructs (`dynamic_cast`, `typeid` on polymorphic types). `std::derived_from` and `entt::type_id` are compile-time and fine.
- Every file keeps its existing licence header. New files in `src/taco/` use:
  ```
  // tacoEngine (c) Nikolas Wipper 2026

  /*
   * This Source Code Form is subject to the terms of the Mozilla Public
   * License, v. 2.0. If a copy of the MPL was not distributed with this
   * file, You can obtain one at https://mozilla.org/MPL/2.0/.
   */
  ```
- Every new `.h`/`.cpp` pair under `src/taco/` must be added to `TACO_SOURCES` in `CMakeLists.txt`.
- Header guards match the existing style: `#ifndef ENTITY_H` / `#define ENTITY_H` / `#endif //ENTITY_H`.
- Indentation is 4 spaces. Braces on the same line. `namespace taco { ... }` is **not** indented (see `Engine.cpp`).
- Build directory is `cmake-build-debug`. Always use absolute paths in build commands; the shell is fish and `cd` between calls is unreliable.
- Build with: `cmake --build /Users/nikolas/dev/local/C++/tacoEngine/cmake-build-debug --target <target>`
- `game/` is **gitignored in its entirety** (`.gitignore:86`). Changes under `game/` are working-tree-only and must never be `git add`ed.
- No checkpoint code lands on this branch. `Checkpoint.{h,cpp}` do not exist on `master` and must not be created here.
- Do not reorder or rename the five update phases: `UpdateEarly`, `UpdatePrePhysics`, `UpdatePostPhysics`, `UpdateLate`, `UpdateUI`.

## File Structure

| File | Responsibility |
| --- | --- |
| `src/taco/Entity.h` | **new.** The handle, `SystemHooks`, `detail::MakeHooks<T>`, `detail::RegisterSystem` declaration. All definitions inline. |
| `src/taco/comp/System.h` | Base class; five hook **declarations** taking `(Engine *, Entity)`. Forward-declares `Entity`. |
| `src/taco/comp/System.cpp` | **new.** The five empty hook bodies. |
| `src/taco/Engine.h` | Private registry, `Create()`, `Each<Ts...>()`, system hook table, physics as `unique_ptr`. |
| `src/taco/Engine.cpp` | `detail::RegisterSystem` definition, hook-table dispatch, `on_destroy` physics hooks. |
| `src/taco/Physics.h` | `Collider`/`Character` copyable, non-owning back-pointer, no destructors. `PhysicsEngine::BodyCount`. |
| `src/taco/Physics.cpp` | Factories pass `this`; destructor bodies deleted. |
| `src/taco/Loader.h` | One registration map. `ComponentLoader` takes `Entity`. `Register<T>` for systems. |
| `src/taco/Loader.cpp` | Builtins ported to `Entity`; `AttachSystems` folded into the component map. |
| `src/taco/comp/Transform.h` | `Link::entity` → `Link::target` of type `Entity`. |
| `src/taco/entity_test.cpp` | **new.** Entity ops, system dispatch, body lifetime. |
| `src/taco/loader_test.cpp` | Ported to `Entity`/`Each`. |
| `CMakeLists.txt` | New sources and the new test target. |
| `CLAUDE.md` | Core model, frame, physics sections. |
| `game/src/*` | Ported (working-tree-only). |

---

### Task 1: Entity handle and System re-layering

**Files:**
- Create: `src/taco/Entity.h`
- Create: `src/taco/comp/System.cpp`
- Create: `src/taco/entity_test.cpp`
- Modify: `src/taco/comp/System.h`
- Modify: `src/taco/Engine.h`
- Modify: `src/taco/Engine.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Produces:
  - `taco::Entity` with `Entity()`, `Entity(Engine *, entt::registry *, entt::entity)`, `entt::entity id() const`, `Engine *engine() const`, `bool Valid() const`, `void Destroy()`, `template<class T, class... Args> T &Add(Args &&...)`, `template<class T> T &Get() const`, `template<class T> bool Has() const`, `template<class T> void Remove()`, `bool operator==(const Entity &) const`.
  - `struct taco::SystemHooks { void (*early)(entt::registry &, Engine *); void (*pre_physics)(...); void (*post_physics)(...); void (*late)(...); void (*ui)(...); }`
  - `void taco::detail::RegisterSystem(Engine *, entt::id_type, SystemHooks)` — **declared only** in this task, and never called (Task 1 adds no system through `Add`). Its definition arrives in Task 2.
  - `taco::Entity taco::Engine::Create()`
  - `taco::System` hooks now take `(Engine *engine, Entity entity)`.
- Consumes: nothing.

**Context:** `Entity::Add<T>` evaluates `std::derived_from<T, System>` for *every* `T`, including `Transform`, so `System` must be a complete type in `Entity.h`. That forces `Entity.h` to include `comp/System.h`, and therefore `System.h` must not include `Entity.h`. A function *declaration* may take an incomplete type by value, so `System.h` forward-declares `class Entity;` and moves its five empty bodies into `System.cpp`.

`Engine::registry` stays **public** in this task. It is privatised in Task 4.

- [ ] **Step 1: Write the failing test**

Create `src/taco/entity_test.cpp`:

```cpp
// tacoEngine (c) Nikolas Wipper 2026

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <cassert>
#include <cstdio>

#include "taco/Engine.h"
#include "taco/Entity.h"
#include "taco/comp/Transform.h"

namespace {
void TestEntityOperations() {
    taco::Engine engine;

    // A default-constructed handle names nothing.
    const taco::Entity null_entity;
    assert(!null_entity.Valid());

    taco::Entity e = engine.Create();
    assert(e.Valid());
    assert(!e.Has<taco::Transform>());

    taco::Transform &t = e.Add<taco::Transform>(Vector3{1, 2, 3}, taco::Rotation(), Vector3{0, 0, 0});
    assert(e.Has<taco::Transform>());
    assert(t.position.x == 1);

    // Get returns a reference into the storage, not a copy.
    e.Get<taco::Transform>().position.y = 9;
    assert(e.Get<taco::Transform>().position.y == 9);

    e.Remove<taco::Transform>();
    assert(!e.Has<taco::Transform>());
    assert(e.Valid()); // removing the last component does not destroy the entity

    // Two handles to the same entity compare equal; different entities do not.
    const taco::Entity same(&engine, &engine.registry, e.id());
    assert(same == e);
    assert(!(engine.Create() == e));

    e.Destroy();
    assert(!e.Valid());
}
}

int main() {
    TestEntityOperations();
    std::printf("entity self-check passed\n");
    return 0;
}
```

- [ ] **Step 2: Register the test target and run it to verify it fails**

Add to `CMakeLists.txt`, after the `tacoInputTest` block:

```cmake
add_executable(tacoEntityTest src/taco/entity_test.cpp)
target_link_libraries(tacoEntityTest tacoEngine)
```

Run:
```
cmake --build /Users/nikolas/dev/local/C++/tacoEngine/cmake-build-debug --target tacoEntityTest
```
Expected: FAIL — `'taco/Entity.h' file not found`.

- [ ] **Step 3: Re-layer `System.h` and add `System.cpp`**

Replace the body of `src/taco/comp/System.h` (keep its existing licence header and guards):

```cpp
#ifndef SYSTEM_H
#define SYSTEM_H

namespace taco {
class Engine;
class Entity;

/// Behaviour attached to an entity. Subclass it, override the phases you need, and add it
/// like any other component: `entity.Add<MySystem>()`. Entity::Add spots the base class and
/// registers the type's dispatch with the engine, so there is nothing else to call.
/// Entity is incomplete here on purpose: Entity.h includes this header, not the other way
/// round. Declarations may take an incomplete type by value; the bodies live in System.cpp.
class System {
public:
    virtual ~System() = default;

    virtual void UpdateEarly(Engine *engine, Entity entity);
    virtual void UpdatePrePhysics(Engine *engine, Entity entity);
    virtual void UpdatePostPhysics(Engine *engine, Entity entity);
    virtual void UpdateLate(Engine *engine, Entity entity);
    virtual void UpdateUI(Engine *engine, Entity entity);
};
}

#endif //SYSTEM_H
```

Create `src/taco/comp/System.cpp` (with the standard licence header):

```cpp
#include "System.h"

#include "taco/Entity.h"

namespace taco {
void System::UpdateEarly(Engine *, Entity) {}
void System::UpdatePrePhysics(Engine *, Entity) {}
void System::UpdatePostPhysics(Engine *, Entity) {}
void System::UpdateLate(Engine *, Entity) {}
void System::UpdateUI(Engine *, Entity) {}
}
```

- [ ] **Step 4: Write `Entity.h`**

Create `src/taco/Entity.h` (with the standard licence header):

```cpp
#ifndef ENTITY_H
#define ENTITY_H

#include <concepts>
#include <utility>

#include <entt/entt.hpp>

#include "comp/System.h"

namespace taco {
class Engine;

/// Five stateless thunks, one per update phase. Each iterates one system type's storage and
/// calls that phase on every instance. Captureless lambdas, so they convert to plain function
/// pointers and SystemHooks stays trivially copyable.
struct SystemHooks {
    void (*early)(entt::registry &, Engine *);
    void (*pre_physics)(entt::registry &, Engine *);
    void (*post_physics)(entt::registry &, Engine *);
    void (*late)(entt::registry &, Engine *);
    void (*ui)(entt::registry &, Engine *);
};

namespace detail {
/// Defined in Engine.cpp. Idempotent: a type already registered is ignored, so calling it on
/// every Add costs one set lookup.
void RegisterSystem(Engine *engine, entt::id_type type, SystemHooks hooks);

template<class T>
SystemHooks MakeHooks();
}

/// Handle to one entity: an id plus the registry it lives in. Cheap to copy and safe to store
/// in a component. A default-constructed Entity is null and fails Valid().
/// Engine is only ever passed through, never dereferenced, so this header needs no more than
/// its forward declaration — which is what lets every member be defined inline.
class Entity {
    Engine *engine_ = nullptr;
    entt::registry *registry_ = nullptr;
    entt::entity entity_ = entt::null;

public:
    Entity() = default;

    /// Public because the system thunks in MakeHooks<T> build handles too.
    Entity(Engine *engine, entt::registry *registry, entt::entity entity)
        : engine_(engine), registry_(registry), entity_(entity) {}

    entt::entity id() const { return entity_; }
    Engine *engine() const { return engine_; }

    bool Valid() const { return registry_ != nullptr && registry_->valid(entity_); }

    /// Fires the registry's on_destroy hooks, which is how Jolt bodies are released.
    void Destroy() { registry_->destroy(entity_); }

    template<class T, class... Args>
    T &Add(Args &&...args) {
        // Dependent on T, so it is only instantiated for the types actually added.
        if constexpr (std::derived_from<T, System>)
            detail::RegisterSystem(engine_, entt::type_id<T>().hash(), detail::MakeHooks<T>());

        return registry_->emplace<T>(entity_, std::forward<Args>(args)...);
    }

    template<class T>
    T &Get() const { return registry_->get<T>(entity_); }

    template<class T>
    bool Has() const { return registry_->all_of<T>(entity_); }

    template<class T>
    void Remove() { registry_->remove<T>(entity_); }

    bool operator==(const Entity &other) const {
        return registry_ == other.registry_ && entity_ == other.entity_;
    }
};

/// Below Entity because the thunks construct handles.
template<class T>
SystemHooks detail::MakeHooks() {
    return {
        [](entt::registry &reg, Engine *engine) {
            for (auto [entity, system] : reg.view<T>().each())
                system.UpdateEarly(engine, Entity(engine, &reg, entity));
        },
        [](entt::registry &reg, Engine *engine) {
            for (auto [entity, system] : reg.view<T>().each())
                system.UpdatePrePhysics(engine, Entity(engine, &reg, entity));
        },
        [](entt::registry &reg, Engine *engine) {
            for (auto [entity, system] : reg.view<T>().each())
                system.UpdatePostPhysics(engine, Entity(engine, &reg, entity));
        },
        [](entt::registry &reg, Engine *engine) {
            for (auto [entity, system] : reg.view<T>().each())
                system.UpdateLate(engine, Entity(engine, &reg, entity));
        },
        [](entt::registry &reg, Engine *engine) {
            for (auto [entity, system] : reg.view<T>().each())
                system.UpdateUI(engine, Entity(engine, &reg, entity));
        },
    };
}
}

#endif //ENTITY_H
```

- [ ] **Step 5: Add `Engine::Create` and port the existing dispatch**

In `src/taco/Engine.h`, add the include and the factory:

```cpp
#include "Entity.h"
```

and in the `public:` section, directly above `void Run();`:

```cpp
    /// A fresh entity with no components.
    Entity Create();
```

In `src/taco/Engine.cpp`, add the definition next to the other small accessors at the bottom of the file (above `GetPhysics`):

```cpp
Entity Engine::Create() {
    return Entity(this, &registry, registry.create());
}
```

The four `visit_systems` call sites in `Engine::Update` currently pass `entity` straight through. Change each lambda's body to wrap it — the lambda parameter stays `entt::entity entity`:

```cpp
    visit_systems([&](std::shared_ptr<System> system, entt::entity entity) {
        system->UpdateEarly(this, Entity(this, &registry, entity));
    });
```

Apply the same wrapping to `UpdatePrePhysics`, `UpdatePostPhysics` and `UpdateLate`, and to the `UpdateUI` call inside `Engine::Render`'s `BeginDrawing` block:

```cpp
            system->UpdateUI(this, Entity(this, &registry, entity));
```

- [ ] **Step 6: Add the new sources to CMake**

In `CMakeLists.txt`, add to `TACO_SOURCES`, keeping the list's existing grouping (`comp/` entries first, then `misc/`, then the rest):

```cmake
        src/taco/comp/System.cpp
        src/taco/comp/System.h
```
and, alongside `src/taco/Engine.h`:
```cmake
        src/taco/Entity.h
```

Note `src/taco/comp/System.h` is not currently listed at all; add both lines.

- [ ] **Step 7: Run the test to verify it passes**

Run:
```
cmake --build /Users/nikolas/dev/local/C++/tacoEngine/cmake-build-debug --target tacoEntityTest tacoLoaderTest tacoInputTest
/Users/nikolas/dev/local/C++/tacoEngine/cmake-build-debug/tacoEntityTest
```
Expected: builds clean, prints `entity self-check passed`, exit 0.

Then confirm nothing regressed:
```
/Users/nikolas/dev/local/C++/tacoEngine/cmake-build-debug/tacoLoaderTest
```
Expected: prints `loader self-check passed`.

- [ ] **Step 8: Commit**

```bash
git add src/taco/Entity.h src/taco/comp/System.h src/taco/comp/System.cpp src/taco/entity_test.cpp src/taco/Engine.h src/taco/Engine.cpp CMakeLists.txt
git commit -m "Add the Entity handle and re-layer System"
```

---

### Task 2: Systems as value components

**Files:**
- Modify: `src/taco/Engine.h`
- Modify: `src/taco/Engine.cpp`
- Modify: `src/taco/Loader.h`
- Modify: `src/taco/Loader.cpp`
- Modify: `src/taco/comp/Transform.h`
- Modify: `src/taco/loader_test.cpp`
- Test: `src/taco/entity_test.cpp`

**Interfaces:**
- Consumes: `taco::Entity`, `taco::SystemHooks`, `taco::detail::MakeHooks<T>`, `taco::Engine::Create()` (Task 1).
- Produces:
  - `void taco::detail::RegisterSystem(Engine *, entt::id_type, SystemHooks)` — now defined.
  - `taco::Loader::ComponentLoader` = `std::function<void(Loader &, Entity, const Value &)>`
  - `taco::Loader::Resolve(const std::string &) -> Entity` (null `Entity` when unknown)
  - `template<class T> void taco::Loader::Register(const std::string &name)`
  - `taco::Link` field renamed: `entt::entity entity` → `Entity target`.
  - `taco::Loader::RegisterSystem` and `taco::Loader::SystemFactory` are **gone**.

**Context:** A system stops being `std::shared_ptr<System>` in a `hashed_string`-named storage and becomes a plain value component in its own typed storage. `Engine` keeps an ordered table of `SystemHooks` instead of scanning every storage in the registry five times a frame.

- [ ] **Step 1: Write the failing test**

Append to `src/taco/entity_test.cpp`, inside the anonymous namespace and above `main`:

```cpp
/// Records every hook call as "<tag><phase>", e.g. "Aearly", in call order.
std::vector<std::string> g_calls;

struct SystemA : taco::System {
    void UpdateEarly(taco::Engine *, taco::Entity) override { g_calls.push_back("Aearly"); }
    void UpdatePrePhysics(taco::Engine *, taco::Entity) override { g_calls.push_back("Apre"); }
    void UpdatePostPhysics(taco::Engine *, taco::Entity) override { g_calls.push_back("Apost"); }
    void UpdateLate(taco::Engine *, taco::Entity) override { g_calls.push_back("Alate"); }
};

/// Overrides one phase only: the other four must fall through to System's empty bodies.
struct SystemB : taco::System {
    void UpdateEarly(taco::Engine *, taco::Entity) override { g_calls.push_back("Bearly"); }
};

/// Asserts that the hook receives a handle to the entity the system is attached to.
struct SelfCheckSystem : taco::System {
    entt::entity seen = entt::null;
    void UpdateEarly(taco::Engine *, taco::Entity self) override { seen = self.id(); }
};

void TestSystemDispatch() {
    taco::Engine engine;
    g_calls.clear();

    taco::Entity a1 = engine.Create();
    a1.Add<SystemA>();
    taco::Entity b = engine.Create();
    b.Add<SystemB>();
    taco::Entity a2 = engine.Create();
    a2.Add<SystemA>();

    // No RegisterSystem call anywhere above: Add<T> spotted the base class itself.
    engine.RunSystemPhasesForTest();


    // The exact sequence pins down both orderings at once: phases run in declaration order,
    // and within a phase the system types run in first-attach order (SystemA before SystemB).
    // SystemB's four unoverridden phases fall through to System's empty bodies and record
    // nothing. a1 and a2 are indistinguishable here, which is why both read "Aearly".
    const std::vector<std::string> expected = {
        "Aearly", "Aearly", "Bearly",
        "Apre", "Apre",
        "Apost", "Apost",
        "Alate", "Alate",
    };
    assert(g_calls == expected);
}

void TestSystemReceivesOwnEntity() {
    taco::Engine engine;
    taco::Entity e = engine.Create();
    e.Add<SelfCheckSystem>();

    engine.RunSystemPhasesForTest();

    assert(e.Get<SelfCheckSystem>().seen == e.id());
}
```

Add the needed includes at the top of the file:

```cpp
#include <string>
#include <vector>
```

and call both from `main`:

```cpp
int main() {
    TestEntityOperations();
    TestSystemDispatch();
    TestSystemReceivesOwnEntity();
    std::printf("entity self-check passed\n");
    return 0;
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run:
```
cmake --build /Users/nikolas/dev/local/C++/tacoEngine/cmake-build-debug --target tacoEntityTest
```
Expected: FAIL — `no member named 'RunSystemPhasesForTest' in 'taco::Engine'`, plus an undefined symbol for `taco::detail::RegisterSystem` once that is resolved.

- [ ] **Step 3: Add the hook table to `Engine`**

In `src/taco/Engine.h`, add the includes:

```cpp
#include <set>
#include <vector>
```

In the private data section, next to `size_t mesh_count_ = 0;`:

```cpp
    /// One entry per system type, in first-attach order. Populated by detail::RegisterSystem.
    std::vector<SystemHooks> system_hooks_;
    std::set<entt::id_type> system_types_;
```

Give the free function access, immediately below `class Engine {`:

```cpp
    friend void detail::RegisterSystem(Engine *engine, entt::id_type type, SystemHooks hooks);
```

In the `public:` section, below `Entity Create();`:

```cpp
    /// Run all five system phases once. Exists so the dispatch table can be tested without
    /// a frame; Run()/Update() do not use it.
    void RunSystemPhasesForTest();
```

In the `private:` section, above `void Update();`:

```cpp
    /// Dispatch one phase over every registered system type. `phase` selects which of the
    /// five thunks to call — SystemHooks' members are function pointers, so the selector is a
    /// pointer to a member whose type is itself a function pointer.
    void DispatchSystems(void (*SystemHooks::*phase)(entt::registry &, Engine *));
```

- [ ] **Step 4: Define registration and dispatch in `Engine.cpp`**

Add at the top of `namespace taco {`, above `Engine::Engine()`:

```cpp
// Called from Entity::Add on every add of a System subclass, so it must stay cheap and
// idempotent. The vector keeps first-attach order, which is the dispatch order; the set is
// only there to make the second and later calls a no-op.
void detail::RegisterSystem(Engine *engine, const entt::id_type type, const SystemHooks hooks) {
    if (engine->system_types_.insert(type).second)
        engine->system_hooks_.push_back(hooks);
}
```

Add the dispatcher next to `Engine::Update`:

```cpp
// Indexed rather than range-based: a hook may Add a system type that is not registered yet,
// which appends to system_hooks_ and can reallocate it. Re-reading size() each iteration also
// means a type registered mid-phase still runs in that phase — what the old storage scan did.
void Engine::DispatchSystems(void (*SystemHooks::*phase)(entt::registry &, Engine *)) {
    for (size_t i = 0; i < system_hooks_.size(); i++)
        (system_hooks_[i].*phase)(registry, this);
}

void Engine::RunSystemPhasesForTest() {
    DispatchSystems(&SystemHooks::early);
    DispatchSystems(&SystemHooks::pre_physics);
    DispatchSystems(&SystemHooks::post_physics);
    DispatchSystems(&SystemHooks::late);
    DispatchSystems(&SystemHooks::ui);
}
```

- [ ] **Step 5: Replace `visit_systems` in `Engine::Update`**

Delete the `visit_systems` lambda at the top of `Engine::Update` entirely, and replace the four call sites:

```cpp
    DispatchSystems(&SystemHooks::early);
    DispatchSystems(&SystemHooks::pre_physics);
```
before the physics block, and
```cpp
    DispatchSystems(&SystemHooks::post_physics);
    DispatchSystems(&SystemHooks::late);
```
after it.

In `Engine::Render`, replace the storage-scanning `UpdateUI` block inside `BeginDrawing` with:

```cpp
    DispatchSystems(&SystemHooks::ui);
```

Remove `#include "comp/System.h"` from `Engine.cpp` — `Entity.h`, via `Engine.h`, already provides it. The named-storage machinery is now unused everywhere, so also remove `#include <entt/core/hashed_string.hpp>` and `using namespace entt::literals;` from `Engine.h`, and `#include <entt/core/hashed_string.hpp>` from `Loader.cpp`. If the build then fails on a missing `_hs` literal, that use was missed — port it rather than restoring the include.

- [ ] **Step 6: Port `Link` to `Entity`**

In `src/taco/comp/Transform.h`, add `#include "taco/Entity.h"` and rename the field:

```cpp
struct Link {
    Entity target;
    bool linkPosX, linkPosY, linkPosZ;
    bool linkRotX, linkRotY, linkRotZ;
    bool linkVelX, linkVelY, linkVelZ;
};
```

In `Engine::Update`'s `Link` loop, replace the lookup:

```cpp
    for (auto [_, link, transform] : link_view.each()) {
        auto &remote_transform = link.target.Get<Transform>();
```

leaving the nine per-axis copies unchanged.

- [ ] **Step 7: Collapse the Loader's two registration maps into one**

In `src/taco/Loader.h`: delete the `SystemFactory` alias, the `systems_` member, both `RegisterSystem` overloads and the `AttachSystems` declaration. Change `ComponentLoader` and `Resolve`, and add `Register`:

```cpp
    using ComponentLoader = std::function<void(Loader &, Entity, const Value &)>;

    void RegisterComponent(const std::string &name, ComponentLoader fn);

    /// Register a type that takes no scene parameters — systems, mostly. The scene's value
    /// for this key is ignored; write a RegisterComponent lambda if you need it.
    template<class T>
    void Register(const std::string &name) {
        RegisterComponent(name, [](Loader &, Entity e, const Value &) { e.Add<T>(); });
    }

    Entity Resolve(const std::string &name) const;
```

and change the `named_` map to `std::map<std::string, Entity>`. Replace `#include "comp/System.h"` with `#include "Entity.h"`.

In `src/taco/Loader.cpp`:

```cpp
Entity Loader::Resolve(const std::string &name) const {
    auto it = named_.find(name);
    return it == named_.end() ? Entity() : it->second;
}
```

In `LoadScene`, pass 1 becomes `named_[it->name.GetString()] = engine_.Create();`, and the `"systems"` key is handled through the same map. Replace the `AttachSystems` call with a call to a private helper that expands both JSON forms into `ApplyComponents`' machinery:

```cpp
// Both spellings the scene format has always accepted:
//   "systems": ["LookSystem", "AimSystem"]        - no parameters
//   "systems": {"LookSystem": {...}}              - the object is the component Value
// Either way the name is looked up in the one component map, so a system registered with
// Register<T> and a component registered with RegisterComponent are indistinguishable here.
void Loader::ApplySystems(Entity entity, const Value &list) {
    static const Value null_params;

    auto apply = [&](const std::string &name, const Value &params) {
        auto f = components_.find(name);
        if (f != components_.end()) f->second(*this, entity, params);
    };

    if (list.IsArray())
        for (const auto &s : list.GetArray()) apply(s.GetString(), null_params);
    else if (list.IsObject())
        for (auto it = list.MemberBegin(); it != list.MemberEnd(); ++it)
            apply(it->name.GetString(), it->value);
}
```

Declare `void ApplySystems(Entity entity, const Value &list);` in `Loader.h`'s private section where `AttachSystems` was.

- [ ] **Step 8: Port the Loader builtins to `Entity`**

Every builtin lambda's signature changes from `(Loader &l, entt::entity e, const Value &v)` to `(Loader &l, Entity e, const Value &v)`, and every `l.engine().registry.emplace<T>(e, ...)` becomes `e.Add<T>(...)`. For example:

```cpp
    RegisterComponent("Transform", [](Loader &l, Entity e, const Value &v) {
        const Vector3 pos = GetVec3(v, "position", {0, 0, 0});
        const Vector3 rot = GetVec3(v, "rotation", {0, 0, 0});
        const Vector3 vel = GetVec3(v, "velocity", {0, 0, 0});
        e.Add<Transform>(pos, Rotation(rot.x * DEG2RAD, rot.y * DEG2RAD, rot.z * DEG2RAD), vel);
    });
```

Three need more than a mechanical swap:

```cpp
    RegisterComponent("Link", [](Loader &l, Entity e, const Value &v) {
        const Entity target = l.Resolve(v["target"].GetString());
        bool px, py, pz, rx, ry, rz, vx, vy, vz;
        GetBool3(v, "pos", px, py, pz);
        GetBool3(v, "rot", rx, ry, rz);
        GetBool3(v, "vel", vx, vy, vz);
        e.Add<Link>(target, px, py, pz, rx, ry, rz, vx, vy, vz);
    });

    RegisterComponent("Collider", [](Loader &l, Entity e, const Value &v) {
        auto physics = l.engine().GetPhysics();
        if (v.HasMember("sphere")) {
            e.Add<Collider>(physics->CreateSphereCollider(v["sphere"].GetFloat()));
        } else if (GetBool(v, "mesh", false)) {
            e.Add<Collider>(physics->CreateMeshCollider(e.Get<Mesh>(), GetBool(v, "dynamic", true)));
        }
    });

    RegisterComponent("Mesh", [](Loader &l, Entity e, const Value &v) {
        // ... mesh generation unchanged ...
        e.Add<Mesh>(mesh);
        if (!e.Has<BoundingBox>())
            e.Add<BoundingBox>(GetMeshBoundingBox(mesh));
    });
```

`ApplyComponents` and `ExpandModel` change their `entt::entity entity` parameters to `Entity entity`; `ExpandModel`'s `engine_.registry.create()` becomes `engine_.Create()` and its four `emplace` calls become `Add`.

- [ ] **Step 9: Update `loader_test.cpp` for the `Link` rename**

The test still uses the public `registry` (privatised in Task 4). Only the `Link` field name changes:

```cpp
        const taco::Link &link = engine.registry.get<taco::Link>(e);
        const taco::Transform &t = link.target.Get<taco::Transform>();
```

- [ ] **Step 10: Run the tests to verify they pass**

Run:
```
cmake --build /Users/nikolas/dev/local/C++/tacoEngine/cmake-build-debug --target tacoEntityTest tacoLoaderTest tacoInputTest
/Users/nikolas/dev/local/C++/tacoEngine/cmake-build-debug/tacoEntityTest
/Users/nikolas/dev/local/C++/tacoEngine/cmake-build-debug/tacoLoaderTest
```
Expected: both print their `... self-check passed` line, exit 0.

- [ ] **Step 11: Commit**

```bash
git add src/taco/Engine.h src/taco/Engine.cpp src/taco/Loader.h src/taco/Loader.cpp src/taco/comp/Transform.h src/taco/entity_test.cpp src/taco/loader_test.cpp
git commit -m "Make systems ordinary value components"
```

---

### Task 3: Engine-owned physics bodies

**Files:**
- Modify: `src/taco/Physics.h`
- Modify: `src/taco/Physics.cpp`
- Modify: `src/taco/Engine.h`
- Modify: `src/taco/Engine.cpp`
- Test: `src/taco/entity_test.cpp`

**Interfaces:**
- Consumes: `taco::Entity` (Task 1).
- Produces:
  - `taco::Collider` and `taco::Character` are copyable, have no user-declared destructor, and hold a non-owning `PhysicsEngine *`.
  - `taco::Character::character_` is `JPH::Ref<JPH::Character>`.
  - `size_t taco::PhysicsEngine::BodyCount() const`
  - `taco::Engine::GetPhysics()` returns `PhysicsEngine *` (was `std::shared_ptr<PhysicsEngine>`).

**Context:** Today `~Collider` removes and destroys its Jolt body, so the body dies with the *component* — including when a temporary is destructed, which is why both types are move-only. Ownership moves to `Engine`, which connects two `on_destroy` hooks. `JPH::Character` is `NonCopyable` but `CharacterBase` derives from `RefTarget`, so `JPH::Ref` is the copyable owner.

- [ ] **Step 1: Write the failing test**

Append to `src/taco/entity_test.cpp`, inside the anonymous namespace:

```cpp
void TestBodyLifetime() {
    taco::Engine engine;
    const size_t empty = engine.GetPhysics()->BodyCount();

    taco::Entity e = engine.Create();
    e.Add<taco::Collider>(engine.GetPhysics()->CreateSphereCollider(1.0));
    assert(engine.GetPhysics()->BodyCount() == empty + 1);

    // A copy going out of scope must not take the body with it. This is the property the old
    // destructor-owned design could not offer, and the reason the components are copyable.
    {
        taco::Collider copy = e.Get<taco::Collider>();
        (void) copy.GetPosition();
    }
    assert(engine.GetPhysics()->BodyCount() == empty + 1);
    assert(e.Get<taco::Collider>().GetPosition().y == 0);

    // Removing the component releases the body, via Engine's on_destroy hook.
    e.Remove<taco::Collider>();
    assert(engine.GetPhysics()->BodyCount() == empty);

    // So does destroying the entity.
    taco::Entity e2 = engine.Create();
    e2.Add<taco::Collider>(engine.GetPhysics()->CreateSphereCollider(1.0));
    assert(engine.GetPhysics()->BodyCount() == empty + 1);
    e2.Destroy();
    assert(engine.GetPhysics()->BodyCount() == empty);

    // Characters take the same route.
    taco::Entity c = engine.Create();
    c.Add<taco::Character>(engine.GetPhysics()->CreateCharacter(1.8, 0.2));
    assert(engine.GetPhysics()->BodyCount() == empty + 1);
    c.Destroy();
    assert(engine.GetPhysics()->BodyCount() == empty);
}
```

Add `#include "taco/Physics.h"` to the test's includes and call `TestBodyLifetime();` from `main`.

- [ ] **Step 2: Run the test to verify it fails**

Run:
```
cmake --build /Users/nikolas/dev/local/C++/tacoEngine/cmake-build-debug --target tacoEntityTest
```
Expected: FAIL — `no member named 'BodyCount'`, and `call to implicitly-deleted copy constructor of 'taco::Collider'`.

- [ ] **Step 3: Make the physics components copyable value types**

In `src/taco/Physics.h`:

```cpp
class PhysicsEngine {
    friend class Engine;
    friend class Collider;
    friend class Character;
    // ... members unchanged except: delete `std::shared_ptr<PhysicsEngine> self_;`
    // and drop the `: public std::enable_shared_from_this<PhysicsEngine>` base.

public:
    PhysicsEngine();

    /// Bodies currently in the simulation. Used to check that a destroyed entity took its
    /// body with it.
    size_t BodyCount() const;
    // ... rest unchanged
};

class Collider {
    friend class PhysicsEngine;
    friend class Engine;

    JPH::BodyID body_id_;
    PhysicsEngine *physics_ = nullptr;  ///< non-owning; the Engine outlives every component
    Vector3 com_;

    Collider(PhysicsEngine *physics, JPH::BodyID body_id, Vector3 com);

public:
    // Copyable and destructor-free on purpose: the body's lifetime belongs to Engine's
    // on_destroy hook, not to this handle. Copying a live Collider and keeping the copy
    // around is not supported — see the spec's known ceilings.
    Collider(const Collider &) = default;
    Collider(Collider &&) = default;
    Collider &operator=(const Collider &) = default;
    Collider &operator=(Collider &&) = default;

    // ... the eight accessors unchanged
};

class Character {
    friend class PhysicsEngine;
    friend class Engine;

    /// JPH::Character is NonCopyable but refcounted, so Ref is what makes this component
    /// copyable. The last surviving Ref frees it, and ~JPH::Character destroys the body.
    JPH::Ref<JPH::Character> character_;
    PhysicsEngine *physics_ = nullptr;

    Character(PhysicsEngine *physics, JPH::Ref<JPH::Character> character);

public:
    Character(const Character &) = default;
    Character(Character &&) = default;
    Character &operator=(const Character &) = default;
    Character &operator=(Character &&) = default;

    bool OnGround() const;
    // ... the six accessors unchanged
};
```

In `src/taco/Physics.cpp`: delete both destructor definitions, update the two constructors, and change the three factories to pass `this` instead of `shared_from_this()`. `CreateCharacter` builds a `JPH::Ref`:

```cpp
size_t taco::PhysicsEngine::BodyCount() const {
    return system_.GetNumBodies();
}

taco::Character taco::PhysicsEngine::CreateCharacter(double height, double radius) {
    // ... settings unchanged ...
    JPH::Ref<JPH::Character> character = new JPH::Character(settings,
                                                            JPH::RVec3::sZero(),
                                                            JPH::Quat::sIdentity(),
                                                            0,
                                                            &system_);
    character->AddToPhysicsSystem(JPH::EActivation::Activate);

    return Character(this, std::move(character));
}
```

`Character`'s accessors already go through `character_->`, which `JPH::Ref` supports unchanged.

- [ ] **Step 4: Move body destruction into `Engine`**

In `src/taco/Engine.h`, change the physics member and getter:

```cpp
    std::unique_ptr<PhysicsEngine> physics_;
```
```cpp
    PhysicsEngine *GetPhysics() const;
```

and declare the hooks plus a destructor in the private section:

```cpp
    /// on_destroy hooks: the body belongs to the engine, not to the component.
    void DestroyColliderBody(entt::registry &reg, entt::entity entity);
    void DestroyCharacterBody(entt::registry &reg, entt::entity entity);
```

with `~Engine();` in the public section next to `Engine();`.

In `src/taco/Engine.cpp`, change the construction and connect the hooks at the end of the constructor:

```cpp
    physics_ = std::make_unique<PhysicsEngine>();
```
```cpp
    // A Jolt body outlives any single copy of its component; it dies with the entity.
    registry.on_destroy<Collider>().connect<&Engine::DestroyColliderBody>(this);
    registry.on_destroy<Character>().connect<&Engine::DestroyCharacterBody>(this);
}

// EnTT does not fire on_destroy when the registry itself is destroyed — ~basic_storage calls
// the non-virtual shrink_to_size(0), which never publishes. So clear it here, while physics_
// is still alive, or every body leaks.
Engine::~Engine() {
    registry.clear();
}

void Engine::DestroyColliderBody(entt::registry &reg, const entt::entity entity) {
    const Collider &collider = reg.get<Collider>(entity);
    physics_->body_interface_.RemoveBody(collider.body_id_);
    physics_->body_interface_.DestroyBody(collider.body_id_);
}

// Only removed, not destroyed: ~JPH::Character destroys its own body when the last Ref to it
// goes away, which is normally the component being erased right after this hook returns.
void Engine::DestroyCharacterBody(entt::registry &reg, const entt::entity entity) {
    const Character &character = reg.get<Character>(entity);
    character.character_->RemoveFromPhysicsSystem();
}
```

and the getter:

```cpp
PhysicsEngine *Engine::GetPhysics() const {
    return physics_.get();
}
```

- [ ] **Step 5: Run the tests to verify they pass**

Run:
```
cmake --build /Users/nikolas/dev/local/C++/tacoEngine/cmake-build-debug --target tacoEntityTest tacoLoaderTest tacoInputTest
/Users/nikolas/dev/local/C++/tacoEngine/cmake-build-debug/tacoEntityTest
/Users/nikolas/dev/local/C++/tacoEngine/cmake-build-debug/tacoLoaderTest
```
Expected: both print their `... self-check passed` line, exit 0.

- [ ] **Step 6: Commit**

```bash
git add src/taco/Physics.h src/taco/Physics.cpp src/taco/Engine.h src/taco/Engine.cpp src/taco/entity_test.cpp
git commit -m "Give the engine ownership of Jolt bodies"
```

---

### Task 4: Privatise the registry

**Files:**
- Modify: `src/taco/Engine.h`
- Modify: `src/taco/Engine.cpp`
- Modify: `src/taco/loader_test.cpp`
- Modify: `src/taco/entity_test.cpp`

**Interfaces:**
- Consumes: everything from Tasks 1–3.
- Produces: `taco::Engine::registry` is private and renamed `registry_`; `template<class... Ts, class Fn> void taco::Engine::Each(Fn &&fn)` calling `fn(Entity, Ts &...)`.

**Context:** By this point `Loader` and both tests are the only consumers left, and `Loader` already goes through `Entity`. The rename is mechanical inside `Engine.{h,cpp}`; `Each` is the one piece of new API.

- [ ] **Step 1: Write the failing test**

`TestEntityOperations` (Task 1) builds a second handle with the three-argument constructor and `engine.registry`. That is the last use of the public registry in the tests. Replace those two lines with a copy of the handle, which exercises `operator==` just as well:

```cpp
    // Two handles to the same entity compare equal; different entities do not.
    const taco::Entity same = e;
    assert(same == e);
    assert(!(engine.Create() == e));
```

Then add a test for `Each`:

```cpp
void TestEach() {
    taco::Engine engine;

    taco::Entity a = engine.Create();
    a.Add<taco::Transform>(Vector3{1, 0, 0}, taco::Rotation(), Vector3{0, 0, 0});
    taco::Entity b = engine.Create();
    b.Add<taco::Transform>(Vector3{2, 0, 0}, taco::Rotation(), Vector3{0, 0, 0});
    engine.Create().Add<taco::Camera>(72.f); // no Transform: must not be visited

    int seen = 0;
    float sum = 0;
    engine.Each<taco::Transform>([&](taco::Entity e, taco::Transform &t) {
        assert(e.Valid());
        seen++;
        sum += t.position.x;
        t.position.x *= 10; // the callback gets a reference into the storage
    });

    assert(seen == 2);
    assert(sum == 3);
    assert(a.Get<taco::Transform>().position.x == 10);
    assert(b.Get<taco::Transform>().position.x == 20);
}
```

Add `#include "taco/comp/Camera.h"` and call `TestEach();` from `main`.

- [ ] **Step 2: Run the test to verify it fails**

Run:
```
cmake --build /Users/nikolas/dev/local/C++/tacoEngine/cmake-build-debug --target tacoEntityTest
```
Expected: FAIL — `no member named 'Each' in 'taco::Engine'`.

- [ ] **Step 3: Add `Each` and privatise the registry**

In `src/taco/Engine.h`, delete `entt::registry registry;` from the public section and add it to the private data section as:

```cpp
    entt::registry registry_;
```

Add to the public section, below `Entity Create();`:

```cpp
    /// Visit every entity carrying all of Ts. `fn` is called as fn(Entity, Ts &...).
    /// Engine's own Render/Update use registry_.view directly; this is for consumers.
    template<class... Ts, class Fn>
    void Each(Fn &&fn) {
        for (auto tuple : registry_.view<Ts...>().each())
            std::apply([&](entt::entity entity, Ts &...components) {
                fn(Entity(this, &registry_, entity), components...);
            }, tuple);
    }
```

Add `#include <tuple>` to `Engine.h`.

`DrawAllMeshes`' declaration uses `decltype(registry.view<...>())` — update it to `registry_`.

In `src/taco/Engine.cpp`, rename every `registry` to `registry_`. There are roughly 30 occurrences; `Update`, `Render` and `DrawAllMeshes` hold almost all of them.

- [ ] **Step 4: Port `loader_test.cpp`**

Rewrite the two loops to use `Each`:

```cpp
    // Entity 'a': the one carrying a Mesh, positioned at (1,2,3).
    bool found_a = false;
    engine.Each<Mesh, taco::Transform>([&](taco::Entity e, Mesh &m, taco::Transform &t) {
        if (t.position.x == 1 && t.position.y == 2 && t.position.z == 3) {
            found_a = true;
            assert(e.Has<BoundingBox>());
            assert(m.tangents != nullptr);
        }
    });
    assert(found_a);

    // Entity 'b': its Link must resolve to 'a' with the right per-axis flags.
    bool found_b = false;
    engine.Each<taco::Link>([&](taco::Entity, taco::Link &link) {
        found_b = true;
        const taco::Transform &t = link.target.Get<taco::Transform>();
        assert(t.position.x == 1 && t.position.y == 2 && t.position.z == 3);
        assert(link.linkPosX && !link.linkPosY && link.linkPosZ);
    });
    assert(found_b);
```

- [ ] **Step 5: Run the tests to verify they pass**

Run:
```
cmake --build /Users/nikolas/dev/local/C++/tacoEngine/cmake-build-debug --target tacoEntityTest tacoLoaderTest tacoInputTest
/Users/nikolas/dev/local/C++/tacoEngine/cmake-build-debug/tacoEntityTest
/Users/nikolas/dev/local/C++/tacoEngine/cmake-build-debug/tacoLoaderTest
```
Expected: both print their `... self-check passed` line, exit 0.

- [ ] **Step 6: Verify nothing outside Engine names the registry**

Run:
```
grep -rnE "(\.|->)registry\b" /Users/nikolas/dev/local/C++/tacoEngine/src
```
Expected: no output. The `\b` matters — without it the pattern also matches `other.registry_` in `Entity.h`, which is that class's own member and perfectly legal.

- [ ] **Step 7: Commit**

```bash
git add src/taco/Engine.h src/taco/Engine.cpp src/taco/loader_test.cpp src/taco/entity_test.cpp
git commit -m "Make the registry private"
```

---

### Task 5: Port the game and update the docs

**Files:**
- Modify: `CLAUDE.md`
- Modify (working tree only, never committed): `game/src/AimSystem.{h,cpp}`, `game/src/FrictionSystem.{h,cpp}`, `game/src/LookSystem.{h,cpp}`, `game/src/MovementSystem.{h,cpp}`, `game/src/RotationSystem.{h,cpp}`, `game/src/ThirdPersonSystem.{h,cpp}`, `game/src/main.cpp`

**Interfaces:**
- Consumes: everything from Tasks 1–4.
- Produces: nothing the library depends on. This task proves the new API works for a real consumer and records it in the project docs.

**Context:** `game/` is gitignored (`.gitignore:86`), so its files are edited but never staged. It is a separate CMake project that adds `..` as a subdirectory; building it is the end-to-end check that the public API is complete.

- [ ] **Step 1: Port the six system headers**

Each header changes its hook signature and can drop the `entt` dependency. For example `game/src/AimSystem.h`:

```cpp
#include <taco/comp/System.h>

class AimSystem : public taco::System {
public:
    void UpdatePostPhysics(taco::Engine *engine, taco::Entity entity) override;
};
```

Apply the same change to `FrictionSystem.h` (`UpdatePrePhysics`), `RotationSystem.h` (`UpdatePostPhysics`), `ThirdPersonSystem.h` (`UpdateLate`), `LookSystem.h` (`UpdatePrePhysics`, `UpdatePostPhysics`) and `MovementSystem.h` (`UpdateEarly`, `UpdatePrePhysics`, `UpdateUI`).

`LookSystem.h` also carries a `Clone()` override left over from the checkpoint branch's working tree. Delete it — `Clone` does not exist on `taco::System` on this branch.

- [ ] **Step 2: Port the six system implementations**

Every `engine->registry.get<T>(entity)` becomes `entity.Get<T>()`. The signature in each `.cpp` matches its header. For example `game/src/ThirdPersonSystem.cpp`:

```cpp
void ThirdPersonSystem::UpdateLate(taco::Engine *engine, taco::Entity entity) {
    auto &transform = entity.Get<taco::Transform>();

    Vector3 direction = Vector3Normalize(transform.rotation.GetDirection());

    transform.position = Vector3Subtract(transform.position, Vector3Scale(direction, 4.0));
}
```

`RotationSystem.cpp` also loses its two-step link lookup:

```cpp
void RotationSystem::UpdatePostPhysics(taco::Engine *engine, taco::Entity entity) {
    auto link = entity.Get<taco::Link>();
    auto cam_transform = link.target.Get<taco::Transform>();
    auto &transform = entity.Get<taco::Transform>();
    // ... the rest unchanged
}
```

- [ ] **Step 3: Port `main.cpp`**

The six `loader.RegisterSystem<T>("T")` calls become:

```cpp
    loader.Register<LookSystem>("LookSystem");
    loader.Register<AimSystem>("AimSystem");
    loader.Register<ThirdPersonSystem>("ThirdPersonSystem");
    loader.Register<MovementSystem>("MovementSystem");
    loader.Register<FrictionSystem>("FrictionSystem");
    loader.Register<RotationSystem>("RotationSystem");
```

Nothing else in `main.cpp` changes — `engine.GetPhysics()->SetGravity(...)` still compiles against the raw pointer.

- [ ] **Step 4: Build the game**

Run:
```
cmake --build /Users/nikolas/dev/local/C++/tacoEngine/game/cmake-build-debug
```
Expected: builds clean. `game/assets/scene.json` needs no change — both `"systems"` forms still work.

- [ ] **Step 5: Update `CLAUDE.md`**

Four sections need rewriting:

1. **Layout** — add `Entity.h  # the entity handle: create/destroy/add/get/remove` under `src/taco/`, and change the `comp/System.h` line to `System.{h,cpp}  # the behaviour component; five phase hooks, added like any component`.
2. **Core model** — replace the `std::shared_ptr<System>` paragraph. Entities come from `Engine::Create()` and are handled through `taco::Entity`; components are added with `entity.Add<T>()`; a system is a subclass of `taco::System` stored **by value** like any other component, and `Add` registers its dispatch automatically, so there is no `RegisterSystem` call. `Engine::registry` is private; consumers iterate with `Engine::Each<Ts...>`.
3. **The frame** — replace the `visit_systems` description: phases now dispatch over `Engine::system_hooks_`, an ordered table of five function pointers per registered system type, in first-attach order. The old "scans all storages for that type" note goes away.
4. **Physics** — replace the "RAII move-only handles" bullet: `Collider`/`Character` are copyable value types with no destructor, holding a non-owning `PhysicsEngine *`; the body is freed by `Engine`'s `on_destroy` hooks when the component is removed or the entity destroyed, and `~Engine` clears the registry first because EnTT does not fire `on_destroy` during registry destruction. Note that `Character` holds a `JPH::Ref<JPH::Character>`.

Also add to **Gotchas**: adding or destroying a component of the type currently being iterated is UB; and a stale `Entity` whose index has been recycled silently names the new occupant.

- [ ] **Step 6: Verify the whole library and tests still build**

Run:
```
cmake --build /Users/nikolas/dev/local/C++/tacoEngine/cmake-build-debug --target tacoEngine tacoEntityTest tacoLoaderTest tacoInputTest
/Users/nikolas/dev/local/C++/tacoEngine/cmake-build-debug/tacoEntityTest
/Users/nikolas/dev/local/C++/tacoEngine/cmake-build-debug/tacoLoaderTest
```
Expected: clean build, both self-checks pass.

- [ ] **Step 7: Commit the docs**

```bash
git add CLAUDE.md
git commit -m "Document the Entity API and engine-owned bodies"
```

Confirm the game changes were not staged:
```
git status --short
```
Expected: clean (no `game/` entries — it is gitignored).

---

## After the plan

`feat/checkpoint` is **not** rebased by this plan. Once this branch is merged, the checkpoint
work is re-applied on top as fresh commits (the spec's chosen strategy), where it should
shrink: systems become trackable components, the storage-id scan disappears, and the parked-body
graveyard is largely subsumed by engine-owned bodies.
