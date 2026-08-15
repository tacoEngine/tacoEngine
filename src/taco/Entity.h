// tacoEngine (c) Nikolas Wipper 2026

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef ENTITY_H
#define ENTITY_H

#include <concepts>
#include <functional>
#include <map>
#include <utility>
#include <vector>

#include <entt/entt.hpp>

#include "comp/System.h"

namespace taco {
class Engine;

/// Five thunks, one per phase, each running that phase over one system type's storage.
/// Captureless lambdas, so they decay to function pointers and SystemHooks stays trivial.
struct SystemHooks {
    void (*early)(entt::registry &, Engine *);
    void (*pre_physics)(entt::registry &, Engine *);
    void (*post_physics)(entt::registry &, Engine *);
    void (*late)(entt::registry &, Engine *);
    void (*ui)(entt::registry &, Engine *);
};

namespace detail {

/// Copies one component type out of the registry, returns the closure that puts it back.
/// Returning a closure instead of filling a Checkpoint keeps Checkpoint.h — and Jolt — out
/// of this header.
using CaptureFn = std::function<void(entt::registry &)> (*)(const entt::registry &);


/// Idempotent: a type already registered is ignored.
void RegisterSystem(Engine *engine, SystemHooks hooks);
/// Idempotent, first registration wins: Engine's hand-written captures and Ignore<T>() hold.
void TrackComponent(Engine *engine, entt::id_type type, CaptureFn capture);

template<class T>
CaptureFn MakeCapture();
}

class Entity {
    friend class Engine;

    Engine *engine_ = nullptr;
    entt::registry *registry_ = nullptr;
    entt::entity entity_ = entt::null;

    template<class T>
    static SystemHooks MakeHooks();

    Entity(Engine *engine, entt::registry *registry, entt::entity entity)
        : engine_(engine), registry_(registry), entity_(entity) {}

public:
    Entity() = default;

    entt::entity id() const { return entity_; }
    Engine *engine() const { return engine_; }

    bool Valid() const { return registry_ != nullptr && registry_->valid(entity_); }

    /// Fires the registry's on_destroy hooks, which is how Jolt bodies are released.
    void Destroy() { registry_->destroy(entity_); }

    template<class T, class... Args>
    T &Add(Args &&...args) {
        if constexpr (std::derived_from<T, System>)
            detail::RegisterSystem(engine_, MakeHooks<T>());

        // Move-only can't be captured by value, so it isn't checkpointed at all.
        if constexpr (std::copy_constructible<T>)
            detail::TrackComponent(engine_, entt::type_id<T>().hash(), detail::MakeCapture<T>());

        return registry_->emplace<T>(entity_, std::forward<Args>(args)...);
    }

    template<class T>
    T &Get() const {
        return registry_->get<T>(entity_);
    }

    template<class T>
    bool Has() const { return registry_->all_of<T>(entity_); }

    template<class T>
    void Remove() { registry_->remove<T>(entity_); }

    bool operator==(const Entity &other) const {
        return registry_ == other.registry_ && entity_ == other.entity_;
    }
};

template<class T>
detail::CaptureFn detail::MakeCapture() {
    return [](const entt::registry &registry) -> std::function<void(entt::registry &)> {
        std::map<entt::entity, T> data;
        for (auto [entity, component] : registry.view<const T>().each())
            data.emplace(entity, component);

        return [data = std::move(data)](entt::registry &reg) {
            // Gained since the capture: drop it, don't just overwrite.
            std::vector<entt::entity> stale;
            for (const entt::entity entity : reg.view<T>())
                if (!data.count(entity)) stale.push_back(entity);
            for (const entt::entity entity : stale) reg.remove<T>(entity);

            // An entity that couldn't be revived stays destroyed.
            for (const auto &[entity, component] : data)
                if (reg.valid(entity)) reg.emplace_or_replace<T>(entity, component);
        };
    };
}

template<class T>
SystemHooks Entity::MakeHooks() {
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
