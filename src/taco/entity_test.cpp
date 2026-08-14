// tacoEngine (c) Nikolas Wipper 2026

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <cassert>
#include <cstdio>
#include <string>
#include <vector>

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

/// Records every hook call as "<tag><phase>", e.g. "Aearly", in call order.
std::vector<std::string> g_calls;

struct SystemA : taco::System {
    void UpdateEarly(taco::Engine *, taco::Entity) override { g_calls.push_back("Aearly"); }
    void UpdatePrePhysics(taco::Engine *, taco::Entity) override { g_calls.push_back("Apre"); }
    void UpdatePostPhysics(taco::Engine *, taco::Entity) override { g_calls.push_back("Apost"); }
    void UpdateLate(taco::Engine *, taco::Entity) override { g_calls.push_back("Alate"); }
};

/// Overrides one phase only: the other four must fall through to System's empty bodies.
struct SystemB : taco::System {
    void UpdateEarly(taco::Engine *, taco::Entity) override { g_calls.push_back("Bearly"); }
};

/// Asserts that the hook receives a handle to the entity the system is attached to.
struct SelfCheckSystem : taco::System {
    entt::entity seen = entt::null;
    void UpdateEarly(taco::Engine *, taco::Entity self) override { seen = self.id(); }
};

void TestSystemDispatch() {
    taco::Engine engine;
    g_calls.clear();

    taco::Entity a1 = engine.Create();
    a1.Add<SystemA>();
    taco::Entity b = engine.Create();
    b.Add<SystemB>();
    taco::Entity a2 = engine.Create();
    a2.Add<SystemA>();

    // No RegisterSystem call anywhere above: Add<T> spotted the base class itself.
    engine.RunSystemPhasesForTest();

    // The exact sequence pins down both orderings at once: phases run in declaration order,
    // and within a phase the system types run in first-attach order (SystemA before SystemB).
    // SystemB's four unoverridden phases fall through to System's empty bodies and record
    // nothing. a1 and a2 are indistinguishable here, which is why both read "Aearly".
    const std::vector<std::string> expected = {
        "Aearly", "Aearly", "Bearly",
        "Apre", "Apre",
        "Apost", "Apost",
        "Alate", "Alate",
    };
    assert(g_calls == expected);
}

void TestSystemReceivesOwnEntity() {
    taco::Engine engine;
    taco::Entity e = engine.Create();
    e.Add<SelfCheckSystem>();

    engine.RunSystemPhasesForTest();

    assert(e.Get<SelfCheckSystem>().seen == e.id());
}
}

int main() {
    TestEntityOperations();
    TestSystemDispatch();
    TestSystemReceivesOwnEntity();
    std::printf("entity self-check passed\n");
    return 0;
}
