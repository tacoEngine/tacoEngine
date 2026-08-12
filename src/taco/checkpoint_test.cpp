// tacoEngine (c) Nikolas Wipper 2026

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>

#include <entt/core/hashed_string.hpp>

#include "taco/Checkpoint.h"
#include "taco/Engine.h"
#include "taco/Loader.h"
#include "taco/Physics.h"
#include "taco/comp/System.h"
#include "taco/comp/Transform.h"

struct CountSystem : taco::System {
    int value = 0;

    std::shared_ptr<taco::System> Clone() const override {
        return std::make_shared<CountSystem>(*this);
    }
};

int main() {
    const char *scene =
        "{ \"entities\": {"
        "  \"ball\": { \"Transform\": {\"position\":[0,10,0]},"
        "              \"Collider\": {\"sphere\": 1.0} },"
        "  \"doomed\": { \"Transform\": {\"position\":[5,5,5]} }"
        "} }";
    { std::ofstream out("checkpoint_test_scene.json"); out << scene; }

    taco::Engine engine;
    taco::Loader loader(engine, ".");
    loader.LoadScene("checkpoint_test_scene.json");

    const entt::entity ball = loader.Resolve("ball");
    assert(ball != entt::null);

    // Systems live in named storages, exactly like Loader::AttachSystems creates them.
    auto &system_storage =
        engine.registry.storage<std::shared_ptr<taco::System>>(entt::hashed_string{"CountSystem"});
    const auto counter = std::make_shared<CountSystem>();
    system_storage.emplace(ball, counter);
    counter->value = 5;

    const entt::entity doomed = loader.Resolve("doomed");
    assert(doomed != entt::null && doomed != ball);

    engine.registry.get<taco::Collider>(ball).SetPosition({0, 10, 0});

    taco::Checkpoint cp = engine.Save();

    // Component data changes, plus an entity that did not exist at capture.
    engine.registry.get<taco::Transform>(ball).position = {9, 9, 9};
    engine.registry.get<taco::Collider>(ball).SetPosition({9, 9, 9});
    counter->value = 99;
    const entt::entity spawned = engine.registry.create();
    engine.registry.emplace<taco::Transform>(spawned, Vector3{1, 1, 1}, taco::Rotation(), Vector3{0, 0, 0});

    // A captured entity destroyed since the capture cannot come back. Restoring must
    // survive its stale handle (emplace_or_replace asserts on invalid entities, and a
    // recycled index would otherwise be written into) and still restore the rest.
    engine.registry.destroy(doomed);

    engine.Restore(cp);

    const taco::Transform &t = engine.registry.get<taco::Transform>(ball);
    assert(t.position.x == 0 && t.position.y == 10 && t.position.z == 0);
    assert(!engine.registry.valid(spawned));
    assert(!engine.registry.valid(doomed));

    const Vector3 body = engine.registry.get<taco::Collider>(ball).GetPosition();
    assert(std::fabs(body.x) < 0.001f);
    assert(std::fabs(body.y - 10.f) < 0.001f);
    assert(std::fabs(body.z) < 0.001f);

    // The clone is a fresh object; the original pointer is replaced, not mutated.
    const auto &restored =
        static_cast<CountSystem &>(*engine.registry.storage<std::shared_ptr<taco::System>>(
            entt::hashed_string{"CountSystem"}).get(ball));
    assert(restored.value == 5);

    // A checkpoint is reusable.
    engine.registry.get<taco::Transform>(ball).position = {1, 1, 1};
    engine.Restore(cp);
    assert(engine.registry.get<taco::Transform>(ball).position.y == 10);

    std::remove("checkpoint_test_scene.json");
    printf("checkpoint test: ok\n");
}
