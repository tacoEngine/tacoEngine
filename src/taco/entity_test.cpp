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
