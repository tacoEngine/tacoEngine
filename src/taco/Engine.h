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

#include <entt/entt.hpp>
#include <entt/core/hashed_string.hpp>
#include <tacoRender.h>

#include "Checkpoint.h"
#include "comp/Transform.h"
#include "Config.h"
#include "input/Input.h"
#include "misc/Debug.h"
#include "Physics.h"
#include "tr_math.h"

using namespace entt::literals;

namespace taco {
class Engine {
    bool running_ = false;
    int64_t delta_time_ = 0.0f;
    long long accumulator_ = 0.f;

    std::shared_ptr<PhysicsEngine> physics_;
    std::unique_ptr<RaylibDebugRenderer> debug_renderer_;
    Config config_;
    Input input_;

    std::map<entt::id_type, std::function<void(const entt::registry &, Checkpoint &)>> tracked_;
    std::set<entt::id_type> ignored_;
    Checkpoint *pending_restore_ = nullptr;
    /// Handles taken off destroyed entities so Restore can hand them back. See Checkpoint.cpp.
    std::map<entt::entity, Collider> retired_colliders_;
    std::map<entt::entity, Character> retired_characters_;
    bool checkpointed_ = false;

    GBuffers gbuffers_;
    GBufferPresenter presenter_;

    size_t mesh_count_ = 0;

public:
    entt::registry registry;

    Engine();
    ~Engine();

    void Run();

    std::shared_ptr<PhysicsEngine> GetPhysics() const;
    double GetDeltaTime() const;
    Input &GetInput();

    Config SwapConfig(Config con);

    /// Capture the current engine state. See Restore.
    Checkpoint Save();
    /// Reset the engine back to cp. Non-const: the Jolt recorders need Rewind().
    void Restore(Checkpoint &cp);
    /// Queue cp to be restored at the end of this frame's Update. cp must be owned by
    /// something Restore cannot destroy — not by a System and not by an entity: Restore
    /// destroys entities spawned since the capture and erases system storage entries, so
    /// either owner can be freed mid-Restore, taking the Checkpoint with it.
    void RequestRestore(Checkpoint &cp);
    /// Apply a restore queued with RequestRestore, if any. Called by Run() after Update();
    /// exposed so a manually driven loop can do the same. Never call it from a System hook —
    /// it runs Restore, which rewrites the very storages Update is iterating. That is what
    /// RequestRestore is for.
    void ApplyPendingRestore();

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

                // Entities destroyed since the capture stay destroyed: their handle is
                // stale, and its index may already have been recycled into a different
                // live entity. Restore diagnoses this; here we just skip it.
                for (const auto &[entity, component] : data)
                    if (reg.valid(entity)) reg.emplace_or_replace<T>(entity, component);
            });
        };
    }

    /// Exclude a type from checkpointing without tripping the untracked warning.
    template<class T>
    void Ignore() {
        ignored_.insert(entt::type_id<T>().hash());
    }

private:
    /// Warn about every storage that is neither tracked nor ignored.
    void WarnUntracked() const;
    /// Track<T> for Sunlight, minus its GPU-owning shadow_map_. See Checkpoint.cpp.
    void TrackSunlight();
    /// on_destroy hooks: park the Jolt body instead of destroying it. See Checkpoint.cpp.
    void RetireCollider(entt::registry &reg, entt::entity entity);
    void RetireCharacter(entt::registry &reg, entt::entity entity);
    /// Destroy every parked body. Runs at Save and at teardown.
    void ClearRetired();

    void Update();
    void Render();
    size_t DrawAllMeshes(const decltype(registry.view<const Transform, const Mesh, Material>()) &model_view,
                         Frustum frustum,
                         Shader shader = LoadMaterialDefault().shader);

    void ReloadGBuffers();
};
} // taco

#endif //ENGINE_H
