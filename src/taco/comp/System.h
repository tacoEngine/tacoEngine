// tacoEngine (c) Nikolas Wipper 2025

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef SYSTEM_H
#define SYSTEM_H

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
};
}

#endif //SYSTEM_H
