// tacoEngine (c) Nikolas Wipper 2026

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

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
