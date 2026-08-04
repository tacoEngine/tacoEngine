// tacoEngine (c) Nikolas Wipper 2026

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <cassert>
#include <cstdio>
#include <fstream>

#include "taco/Engine.h"
#include "taco/Loader.h"
#include "taco/comp/Transform.h"

int main() {
    const char *scene =
        "{ \"entities\": {"
        "  \"a\": { \"Transform\": {\"position\":[1,2,3]},"
        "           \"Mesh\": {\"generate\":\"cube\",\"size\":[1,1,1]} },"
        "  \"b\": { \"Transform\": {},"
        "           \"Link\": {\"target\":\"a\",\"pos\":[true,false,true]} }"
        "} }";
    { std::ofstream out("loader_test_scene.json"); out << scene; }

    taco::Engine engine;
    taco::Loader loader(engine, ".");
    loader.LoadScene("loader_test_scene.json");

    // Entity 'a': the one carrying a Mesh, positioned at (1,2,3).
    bool found_a = false;
    for (const entt::entity e : engine.registry.view<Mesh, taco::Transform>()) {
        const taco::Transform &t = engine.registry.get<taco::Transform>(e);
        if (t.position.x == 1 && t.position.y == 2 && t.position.z == 3) {
            found_a = true;
            assert(engine.registry.all_of<BoundingBox>(e));
            assert(engine.registry.get<Mesh>(e).tangents != nullptr);
        }
    }
    assert(found_a);

    // Entity 'b': its Link must resolve to 'a' with the right per-axis flags.
    bool found_b = false;
    for (const entt::entity e : engine.registry.view<taco::Link>()) {
        found_b = true;
        const taco::Link &link = engine.registry.get<taco::Link>(e);
        const taco::Transform &t = engine.registry.get<taco::Transform>(link.entity);
        assert(t.position.x == 1 && t.position.y == 2 && t.position.z == 3);
        assert(link.linkPosX && !link.linkPosY && link.linkPosZ);
    }
    assert(found_b);

    std::printf("loader self-check passed\n");
    return 0;
}
