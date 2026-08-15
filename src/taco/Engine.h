// tacoEngine (c) Nikolas Wipper 2024-2025

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef ENGINE_H
#define ENGINE_H

#include <functional>
#include <map>
#include <set>
#include <tuple>
#include <vector>

#include <entt/entt.hpp>
#include <tacoRender.h>

#include "Checkpoint.h"
#include "comp/Transform.h"
#include "Config.h"
#include "Entity.h"
#include "input/Input.h"
#include "misc/Debug.h"
#include "Physics.h"
#include "tr_math.h"

namespace taco {
class Engine {
    friend void detail::RegisterSystem(Engine *engine, SystemHooks hooks);
    friend void detail::TrackComponent(Engine *engine, entt::id_type type, detail::CaptureFn capture);

    bool running_ = false;
    int64_t delta_time_ = 0.0f;
    long long accumulator_ = 0.f;

    std::unique_ptr<PhysicsEngine> physics_;
    std::unique_ptr<RaylibDebugRenderer> debug_renderer_;
    Config config_;
    Input input_;

    GBuffers gbuffers_;
    GBufferPresenter presenter_;

    size_t mesh_count_ = 0;

    /// One entry per system type, in first-attach order. Populated by detail::RegisterSystem.
    std::vector<SystemHooks> system_hooks_;

    /// One capture thunk per component type, registered by Entity::Add.
    std::map<entt::id_type, detail::CaptureFn> tracked_;
    std::set<entt::id_type> ignored_;
    Checkpoint *pending_restore_ = nullptr;
    /// Bodies parked on destroy, waiting for a Restore to hand them back.
    std::map<entt::entity, Collider> retired_colliders_;
    std::map<entt::entity, Character> retired_characters_;
    bool checkpointed_ = false;

    entt::registry registry_;

public:
    Engine();
    ~Engine();

    /// A fresh entity with no components.
    Entity Create();

    /// Visit every entity carrying all of Ts. `fn` is called as fn(Entity, Ts &...).
    /// Engine's own Render/Update use registry_.view directly; this is for consumers.
    template<class... Ts, class Fn>
    void Each(Fn &&fn) {
        for (auto tuple : registry_.view<Ts...>().each())
            std::apply([&](entt::entity entity, Ts &...components) {
                fn(Entity(this, &registry_, entity), components...);
            }, tuple);
    }

    /// Run all five system phases once. Exists so the dispatch table can be tested without
    /// a frame; Run()/Update() do not use it.
    void RunSystemPhasesForTest();

    void Run();

    PhysicsEngine *GetPhysics() const;
    double GetDeltaTime() const;
    Input &GetInput();

    Config SwapConfig(Config con);

    /// Capture the engine state. Many checkpoints can be alive at once, but the first Restore
    /// invalidates the rest: it frees the parked bodies only it could have revived.
    Checkpoint Save();
    /// Reset the engine back to cp. Non-const: the Jolt recorders need Rewind().
    void Restore(Checkpoint &cp);
    /// Queue cp for the end of this frame's Update. Its owner has to survive Restore — not a
    /// System, not an entity: Restore can free either one, taking cp with it.
    void RequestRestore(Checkpoint &cp);
    /// Drain the queue. Run() calls it after Update; exposed for hand-rolled loops. Never call
    /// it from a System hook — Restore rewrites the storages Update is iterating.
    void ApplyPendingRestore();

    /// Opt a type out of checkpointing. There is no matching Track<T>(): Entity::Add registers
    /// every type the first time one is added, so this is the only knob — for a System that
    /// should keep its state across a restore, or a handle you manage yourself. Order does not
    /// matter; it also drops an existing entry.
    template<class T>
    void Ignore() {
        const entt::id_type type = entt::type_id<T>().hash();
        ignored_.insert(type);
        tracked_.erase(type);
    }

private:
    /// Sunlight's capture, minus its GPU-owning shadow_map_. Registered in the ctor so it beats
    /// the generic one Entity::Add would install.
    void TrackSunlight();
    /// Destroy every parked body.
    void ClearRetired();
    /// Give the parked bodies cp knows back to their entities; destroy the rest.
    void HandBackRetired(const Checkpoint &cp);

    /// Dispatch one phase over every registered system type. `phase` selects which of the
    /// five thunks to call — SystemHooks' members are function pointers, so the selector is a
    /// pointer to a member whose type is itself a function pointer.
    void DispatchSystems(void (*SystemHooks::*phase)(entt::registry &, Engine *));

    /// on_destroy hooks: the body belongs to the engine, not to the component.
    void DestroyColliderBody(entt::registry &reg, entt::entity entity);
    void DestroyCharacterBody(entt::registry &reg, entt::entity entity);

    void Update();
    void Render();
    size_t DrawAllMeshes(const decltype(registry_.view<const Transform, const Mesh, Material>()) &model_view,
                         Frustum frustum,
                         Shader shader = LoadMaterialDefault().shader);

    void ReloadGBuffers();
};
} // taco

#endif //ENGINE_H
