// tacoEngine (c) Nikolas Wipper 2025

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

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
