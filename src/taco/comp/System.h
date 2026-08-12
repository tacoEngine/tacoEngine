// tacoEngine (c) Nikolas Wipper 2025

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef SYSTEM_H
#define SYSTEM_H

#include <memory>

#include <entt/entt.hpp>

namespace taco {
class Engine;

class System {
public:
    virtual void UpdateEarly(Engine *engine, entt::entity entity) {}
    virtual void UpdatePrePhysics(Engine *engine, entt::entity entity) {}
    virtual void UpdatePostPhysics(Engine *engine, entt::entity entity) {}
    virtual void UpdateLate(Engine *engine, entt::entity entity) {}
    virtual void UpdateUI(Engine *engine, entt::entity entity) {}

    /// Return a copy of this system to make its state part of a Checkpoint.
    /// The default (nullptr) means the system keeps its state across a restore.
    /// Override with: return std::make_shared<MySystem>(*this);
    virtual std::shared_ptr<System> Clone() const { return nullptr; }
};
}

#endif //SYSTEM_H
