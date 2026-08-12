// tacoEngine (c) Nikolas Wipper 2026

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <cassert>
#include <cstdio>
#include <fstream>

#include "taco/Checkpoint.h"
#include "taco/Engine.h"
#include "taco/Loader.h"
#include "taco/comp/Transform.h"

int main() {
    const char *scene =
        "{ \"entities\": {"
        "  \"ball\": { \"Transform\": {\"position\":[0,10,0]},"
        "              \"Collider\": {\"sphere\": 1.0} }"
        "} }";
    { std::ofstream out("checkpoint_test_scene.json"); out << scene; }

    taco::Engine engine;
    taco::Loader loader(engine, ".");
    loader.LoadScene("checkpoint_test_scene.json");

    const entt::entity ball = loader.Resolve("ball");
    assert(ball != entt::null);

    taco::Checkpoint cp = engine.Save();

    // Component data changes, plus an entity that did not exist at capture.
    engine.registry.get<taco::Transform>(ball).position = {9, 9, 9};
    const entt::entity spawned = engine.registry.create();
    engine.registry.emplace<taco::Transform>(spawned, Vector3{1, 1, 1}, taco::Rotation(), Vector3{0, 0, 0});

    engine.Restore(cp);

    const taco::Transform &t = engine.registry.get<taco::Transform>(ball);
    assert(t.position.x == 0 && t.position.y == 10 && t.position.z == 0);
    assert(!engine.registry.valid(spawned));

    // A checkpoint is reusable.
    engine.registry.get<taco::Transform>(ball).position = {1, 1, 1};
    engine.Restore(cp);
    assert(engine.registry.get<taco::Transform>(ball).position.y == 10);

    std::remove("checkpoint_test_scene.json");
    printf("checkpoint test: ok\n");
}
