# Engine API Design

**Goal:** Put the ECS behind a `taco::Entity` handle, make systems ordinary components,
and move Jolt body lifetime from component destructors to the engine.

**Why:** All three are quality-of-life on their own, and all three are prerequisites the
checkpoint feature (branch `feat/checkpoint`) had to work around. Checkpointing is
re-applied on top of this branch afterwards; no checkpoint code lands here.

## Motivation

Three specific pains in the current design:

- `entt` leaks into every consumer. Game systems call `engine->registry.get<T>(entity)`,
  the loader calls `engine_.registry.emplace<T>(...)`, tests call `registry.view<...>()`.
  There is no engine-level vocabulary for "an entity".
- A system is a `std::shared_ptr<System>` component stored in a named storage keyed by a
  `hashed_string` of the system's name. Every phase, `Engine::Update` scans **every storage
  in the registry** looking for that type. The name is a string chosen by the loader, the
  dispatch is virtual through a heap allocation, and the type is invisible to anything that
  wants to treat a system as data.
- `~Collider` and `~Character` remove and destroy their Jolt body. The body therefore dies
  when the *component* dies — including when a temporary is destructed — which makes the
  components move-only, uncopyable, and impossible to snapshot or hand around.

## Architecture

Four units, in dependency order:

```
Entity.h     handle: engine ptr + registry ptr + entity id; all component operations
  ^
System.h     base class; five hooks taking (Engine *, Entity)
  ^
Engine.h     owns the registry (now private), the system dispatch table, physics ownership
  ^
Loader.h     one registration map for components and systems alike
```

### Header layering

`Entity::Add<T>` tests `std::derived_from<T, System>`, which requires **`System` to be a
complete type** at every instantiation — including `Add<Transform>`, where nothing would
otherwise have pulled in the system header. So `Entity.h` includes `comp/System.h`, and
`System.h` must not include `Entity.h` back.

That is affordable because a function *declaration* may take an incomplete type by value;
only a definition and a call require completeness. `System.h` therefore forward-declares
`class Entity;` and **declares** its five hooks without bodies. The empty bodies move to a new
`comp/System.cpp`, which includes `Entity.h` and can see the complete type.

A subclass's `override` declaration is likewise just a declaration, so a game system's header
keeps including only `<taco/comp/System.h>`. Its `.cpp` needs `Entity` complete and already
includes `<taco/Engine.h>`.

`Engine` appears in `Entity.h` only as an opaque forward-declared pointer. Nothing in
`Entity.h` dereferences it.

## Entity

```cpp
// src/taco/Entity.h
namespace taco {
class Engine;
class System;

/// Five stateless thunks, one per phase. Each iterates one system type's storage.
struct SystemHooks {
    void (*early)(entt::registry &, Engine *);
    void (*pre_physics)(entt::registry &, Engine *);
    void (*post_physics)(entt::registry &, Engine *);
    void (*late)(entt::registry &, Engine *);
    void (*ui)(entt::registry &, Engine *);
};

namespace detail {
template<class T> SystemHooks MakeHooks();
/// Defined in Engine.cpp. Idempotent: a type already registered is ignored.
void RegisterSystem(Engine *engine, entt::id_type type, SystemHooks hooks);
}

/// Handle to one entity. Cheap to copy, safe to store in a component.
/// A default-constructed Entity is null and fails Valid().
class Entity {
    Engine *engine_ = nullptr;
    entt::registry *registry_ = nullptr;
    entt::entity entity_ = entt::null;

public:
    Entity() = default;
    /// Public because the system thunks in MakeHooks<T> build handles too.
    Entity(Engine *engine, entt::registry *registry, entt::entity entity);

    bool Valid() const;
    void Destroy();

    template<class T, class... Args> T &Add(Args &&...args);
    template<class T> T &Get() const;
    template<class T> bool Has() const;
    template<class T> void Remove();

    bool operator==(const Entity &other) const;
};
}
```

Every member is defined in `Entity.h`. Carrying `registry_` alongside `engine_` is what makes
that possible: the component operations need only `entt`, so none of them requires a complete
`Engine`.

`Destroy()` is `registry_->destroy(entity_)`. That fires the `on_destroy` hooks `Engine`
connected, so Jolt bodies are released without `Entity` knowing physics exists.

`Add<T>` registers the type if it is a system:

```cpp
template<class T, class... Args>
T &Add(Args &&...args) {
    if constexpr (std::derived_from<T, System>)
        detail::RegisterSystem(engine_, entt::type_id<T>().hash(), detail::MakeHooks<T>());
    return registry_->emplace<T>(entity_, std::forward<Args>(args)...);
}
```

`MakeHooks<T>` is defined below `Entity` in the same header (it constructs an `Entity` and so
needs the complete type). Each member is a captureless lambda, which converts to a function
pointer:

```cpp
template<class T>
SystemHooks detail::MakeHooks() {
    return {
        [](entt::registry &reg, Engine *engine) {
            for (auto [entity, system] : reg.view<T>().each())
                system.UpdateEarly(engine, Entity(engine, &reg, entity));
        },
        // ... four more, one per phase
    };
}
```

## System

`src/taco/comp/System.h` keeps the base class. The hook signature changes, and the bodies move
out of the header for the layering reason above:

```cpp
// System.h — declarations only; Entity is incomplete here
class Engine;
class Entity;

class System {
public:
    virtual void UpdateEarly(Engine *engine, Entity entity);
    virtual void UpdatePrePhysics(Engine *engine, Entity entity);
    virtual void UpdatePostPhysics(Engine *engine, Entity entity);
    virtual void UpdateLate(Engine *engine, Entity entity);
    virtual void UpdateUI(Engine *engine, Entity entity);
};

// System.cpp
void System::UpdateEarly(Engine *, Entity) {}
// ... four more
```

A system is now stored **by value** in its own typed storage, like any other component:
`entity.Add<MovementSystem>()`. There is no `shared_ptr`, no named storage, no
`RegisterSystem` call in game code or in the loader — `Add` detects the base class and
registers the type on first use.

The `Engine *` parameter is redundant with the one inside `Entity`, but keeping it means hooks
still read `engine->GetDeltaTime()` rather than reaching through the handle.

## Engine

```cpp
class Engine {
    entt::registry registry_;                 // was: public `registry`
    std::vector<SystemHooks> system_hooks_;   // dispatch order = first-attach order
    std::set<entt::id_type> system_types_;    // dedupe for RegisterSystem

public:
    Entity Create();
    template<class... Ts> void Each(auto &&fn);   // fn(Entity, Ts &...)
};
```

`Engine::Update` replaces the storage scan with a walk over `system_hooks_`:

```cpp
for (size_t i = 0; i < system_hooks_.size(); i++)
    system_hooks_[i].early(registry_, this);
```

Indexed rather than range-based because a hook may `Add` a component of a type not yet
registered, which appends to the vector and can reallocate it. Re-reading `size()` each
iteration means a type registered mid-phase runs in that same phase — the same behaviour as
today, where a newly emplaced system is picked up by the ongoing storage scan.

`Each<Ts...>` exists for consumers that need to iterate. `Engine`'s own `Render` and `Update`
keep using `registry_.view<...>()` directly — routing internals through a callback buys
nothing when the registry is right there.

`Engine::Create()` returns `Entity(this, &registry_, registry_.create())`.

## Physics ownership

`Collider` and `Character` become copyable value types:

- the `std::shared_ptr<PhysicsEngine>` becomes a raw, non-owning `PhysicsEngine *`
- both destructors are deleted outright
- `Character::character_` changes from `std::unique_ptr<JPH::Character>` to
  `JPH::Ref<JPH::Character>`. `JPH::Character` is `NonCopyable`, so a `unique_ptr` cannot be
  copied; `CharacterBase` derives from `RefTarget`, so Jolt's own intrusive refcount is the
  copyable owner. Copies share one `JPH::Character`; the last one to die frees it, and
  `~JPH::Character` destroys the Jolt body.
- `PhysicsEngine` drops `std::enable_shared_from_this` and its `self_` member; the `Create*`
  factories pass `this`. `Engine` holds it as a `std::unique_ptr<PhysicsEngine>` and
  `GetPhysics()` returns a raw `PhysicsEngine *`, so `engine.GetPhysics()->SetGravity(...)`
  is unchanged at the call site.
- `PhysicsEngine` gains `size_t BodyCount() const` returning `system_.GetNumBodies()`, so body
  lifetime is observable without reaching through a `friend`.

Creation stays on the physics engine, unchanged at the call site:

```cpp
entity.Add<Collider>(engine.GetPhysics()->CreateSphereCollider(1.0));
```

Destruction moves to `Engine`, which connects two hooks in its constructor:

```cpp
registry_.on_destroy<Collider>().connect<&Engine::DestroyColliderBody>(this);
registry_.on_destroy<Character>().connect<&Engine::DestroyCharacterBody>(this);
```

`DestroyColliderBody` removes and destroys the Jolt body; `DestroyCharacterBody` removes it
from the physics system (`~JPH::Character` destroys the body itself). Entity destroyed or
component removed → body freed. Component copied, moved, or destructed as a temporary →
nothing happens.

One consequence worth stating: EnTT does **not** fire `on_destroy` when the registry itself is
destroyed, so `~Engine` must clear the registry before the `PhysicsEngine` goes away.

## Loader

`SystemFactory`, the `systems_` map and `AttachSystems` are deleted. A system is registered
exactly like a component:

```cpp
template<class T>
void Loader::Register(const std::string &name) {
    RegisterComponent(name, [](Loader &, Entity e, const Value &) { e.Add<T>(); });
}
```

One map, one lookup path. `main.cpp`'s six `loader.RegisterSystem<T>("T")` calls become
`loader.Register<T>("T")`.

The `"systems"` key stays valid in scene JSON in **both** its current forms, expanded into the
same component map, so existing scenes are unchanged:

- `"systems": ["LookSystem", "AimSystem"]` — each name looked up and invoked with an empty
  `Value`, exactly as `AttachSystems` does today.
- `"systems": {"LookSystem": {...}}` — each name invoked with its object as the `Value`.

`Register<T>` ignores that `Value`, but a hand-written `RegisterComponent(name, fn)` receives
it, so the object form loses nothing relative to today's `SystemFactory`.

`ComponentLoader` changes from `void(Loader &, entt::entity, const Value &)` to
`void(Loader &, Entity, const Value &)`. `Loader::Resolve` returns `Entity` instead of
`entt::entity`, returning a null `Entity` for an unknown name.

The `Collider` builtin loses its `registry.get<Mesh>(e)` reach-around in favour of
`e.Get<Mesh>()`.

## Link

`Link::entity` becomes `Entity target`. `RotationSystem`'s two-step lookup collapses to
`link.target.Get<Transform>()`, and `entt` then appears nowhere in game code.

## Migration

| File | Change |
| --- | --- |
| `src/taco/Entity.h` | new |
| `src/taco/comp/System.h` | hook signature takes `Entity`; bodies move out |
| `src/taco/comp/System.cpp` | new: the five empty hook bodies |
| `src/taco/Engine.{h,cpp}` | private registry, `Create`/`Each`, hook table, physics hooks |
| `src/taco/Physics.{h,cpp}` | raw back-pointer, no destructors, no `shared_from_this` |
| `src/taco/Loader.{h,cpp}` | one registration map, `Entity` in signatures |
| `src/taco/comp/Transform.h` | `Link::entity` → `Entity target` |
| `src/taco/loader_test.cpp` | ported to `Each`/`Entity` |
| `src/taco/entity_test.cpp` | new |
| `CMakeLists.txt` | register the new test |
| `game/src/*System.{h,cpp}`, `game/src/main.cpp` | hook signature, `Get<T>()`, `Register<T>` |
| `CLAUDE.md` | core model, frame, physics sections |

## Testing

`entity_test.cpp` covers the three things that can silently break, as `assert`s in a single
binary:

1. **Entity operations** — `Add`/`Get`/`Has`/`Remove`, `Valid()` before and after `Destroy()`,
   a default-constructed `Entity` failing `Valid()`.
2. **System auto-detection** — two test systems, one attached to two entities, asserting that
   each phase fires once per entity and that dispatch order matches first-attach order. No
   registration call anywhere in the test.
3. **Body lifetime** — a `Collider` copied into a second variable that then goes out of scope,
   asserting the original still reports a valid position; then `entity.Destroy()`, asserting
   the body count in the physics system drops.

`loader_test.cpp` is ported to the new API and keeps its existing assertions.

## Out of scope

- Checkpointing. Re-applied on `feat/checkpoint` after this lands.
- Hiding `entt::entity` from `Entity`'s own interface — it is the identity, and `Entity`
  exposes it for hashing and debug output.
- Reordering or renaming the five update phases.
- The shadow-map and frustum-culling `fixme`s in `Engine::Render`.

## Known ceilings

- **Mutation during dispatch.** Adding or destroying a component of the type currently being
  iterated is undefined behaviour, as it is today. Not fixed, only documented.
- **Empty hooks still cost an iteration.** A system that overrides two of five hooks is still
  visited five times per frame, each visit iterating its storage. Identical to the current
  `visit_systems` behaviour.
- **A copied physics component can outlive its body.** `on_destroy` fires when the *first*
  copy's entity is destroyed. For `Collider` that destroys the body while any other copy still
  names its `BodyID`; for `Character` the refcount keeps the object alive but removed from the
  simulation. Copying a physics component and keeping the copy is therefore not supported —
  the copyability exists so the components can be held and moved by value, not aliased.
- **A dangling `Entity` is not detected.** `Entity` stores a raw entity id; if the entity is
  destroyed and its index recycled, the handle silently refers to the new occupant. EnTT's
  version counter makes `Valid()` correct, but a stale handle stored in a `Link` and never
  re-validated will point at whatever now lives there.
