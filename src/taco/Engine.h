// tacoEngine (c) Nikolas Wipper 2024-2025

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef ENGINE_H
#define ENGINE_H

#include <set>
#include <vector>

#include <entt/entt.hpp>
#include <tacoRender.h>

#include "comp/Transform.h"
#include "Config.h"
#include "Entity.h"
#include "input/Input.h"
#include "misc/Debug.h"
#include "Physics.h"
#include "tr_math.h"

namespace taco {
class Engine {
    friend void detail::RegisterSystem(Engine *engine, entt::id_type type, SystemHooks hooks);

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
    std::set<entt::id_type> system_types_;

public:
    entt::registry registry;

    Engine();
    ~Engine();

    /// A fresh entity with no components.
    Entity Create();

    /// Run all five system phases once. Exists so the dispatch table can be tested without
    /// a frame; Run()/Update() do not use it.
    void RunSystemPhasesForTest();

    void Run();

    PhysicsEngine *GetPhysics() const;
    double GetDeltaTime() const;
    Input &GetInput();

    Config SwapConfig(Config con);

private:
    /// Dispatch one phase over every registered system type. `phase` selects which of the
    /// five thunks to call — SystemHooks' members are function pointers, so the selector is a
    /// pointer to a member whose type is itself a function pointer.
    void DispatchSystems(void (*SystemHooks::*phase)(entt::registry &, Engine *));

    /// on_destroy hooks: the body belongs to the engine, not to the component.
    void DestroyColliderBody(entt::registry &reg, entt::entity entity);
    void DestroyCharacterBody(entt::registry &reg, entt::entity entity);

    void Update();
    void Render();
    size_t DrawAllMeshes(const decltype(registry.view<const Transform, const Mesh, Material>()) &model_view,
                         Frustum frustum,
                         Shader shader = LoadMaterialDefault().shader);

    void ReloadGBuffers();
};
} // taco

#endif //ENGINE_H
