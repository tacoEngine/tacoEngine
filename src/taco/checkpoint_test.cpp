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
#include "taco/comp/Lights.h"
#include "taco/comp/System.h"
#include "taco/comp/Transform.h"

struct CountSystem : taco::System {
    int value = 0;

    std::shared_ptr<taco::System> Clone() const override {
        return std::make_shared<CountSystem>(*this);
    }
};

struct Untracked {
    int value;
};

struct Tracked {
    int value;
};

int main() {
    const char *scene =
        "{ \"entities\": {"
        "  \"ball\": { \"Transform\": {\"position\":[0,10,0]},"
        "              \"Collider\": {\"sphere\": 1.0} },"
        "  \"walker\": { \"Transform\": {\"position\":[20,10,0]},"
        "                \"Character\": {\"height\": 1.8, \"radius\": 0.2} },"
        "  \"doomed\": { \"Transform\": {\"position\":[5,5,5]},"
        "                \"Collider\": {\"sphere\": 0.5} },"
        "  \"doomed_walker\": { \"Transform\": {\"position\":[40,10,0]},"
        "                       \"Character\": {\"height\": 1.8, \"radius\": 0.2} }"
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

    const entt::entity walker = loader.Resolve("walker");
    assert(walker != entt::null);
    engine.registry.get<taco::Character>(walker).SetPosition({20, 10, 0});

    const entt::entity doomed = loader.Resolve("doomed");
    assert(doomed != entt::null && doomed != ball);
    engine.registry.get<taco::Collider>(doomed).SetPosition({5, 5, 5});

    const entt::entity doomed_walker = loader.Resolve("doomed_walker");
    assert(doomed_walker != entt::null);
    engine.registry.get<taco::Character>(doomed_walker).SetPosition({40, 10, 0});

    engine.registry.get<taco::Collider>(ball).SetPosition({0, 10, 0});

    // Not registered with Track<>: Save must warn, and the value must not restore.
    engine.registry.emplace<Untracked>(ball, 1);

    // Registered with Track<>: proves Track<T>() works end to end.
    engine.Track<Tracked>();
    engine.registry.emplace<Tracked>(ball, 1);

    // Sunlight has a hand-written capture (its shadow_map_ must not round-trip);
    // this proves that one is registered and restores the value fields.
    engine.registry.emplace<taco::Sunlight>(ball, 2.f, WHITE, true);

    // Step first: SetPosition never activates a body, so without this the physics
    // assertions below would only ever cover a sleeping body with zero velocity.
    for (int i = 0; i < 10; i++)
        engine.GetPhysics()->Update(1.0 / 60.0);

    const Vector3 saved_pos = engine.registry.get<taco::Collider>(ball).GetPosition();
    const Vector3 saved_vel = engine.registry.get<taco::Collider>(ball).GetVelocity();
    const Vector3 saved_walker = engine.registry.get<taco::Character>(walker).GetPosition();
    const Vector3 saved_doomed = engine.registry.get<taco::Collider>(doomed).GetPosition();
    const Vector3 saved_doomed_vel = engine.registry.get<taco::Collider>(doomed).GetVelocity();
    const Vector3 saved_doomed_walker = engine.registry.get<taco::Character>(doomed_walker).GetPosition();
    assert(saved_pos.y < 10.f && saved_vel.y < -0.1f); // it really is falling

    taco::Checkpoint cp = engine.Save();

    // Component data changes, plus an entity that did not exist at capture.
    engine.registry.get<taco::Transform>(ball).position = {9, 9, 9};
    engine.registry.get<taco::Collider>(ball).SetPosition({9, 9, 9});
    engine.registry.get<taco::Character>(walker).SetPosition({0, 0, 0});
    for (int i = 0; i < 10; i++)
        engine.GetPhysics()->Update(1.0 / 60.0);
    counter->value = 99;
    engine.registry.get<taco::Sunlight>(ball).intensity = 9.f;
    engine.registry.get<taco::Sunlight>(ball).shadow_casting = false;
    engine.registry.get<Untracked>(ball).value = 2;
    engine.registry.get<Tracked>(ball).value = 2;
    const entt::entity spawned = engine.registry.create();
    engine.registry.emplace<taco::Transform>(spawned, Vector3{1, 1, 1}, taco::Rotation(), Vector3{0, 0, 0});

    // Both destroyed entities must come back, with their bodies: the handles are parked,
    // not destructed. Destroying them frees two entity indices that the creates below then
    // recycle, so the revival has to be able to take them back.
    engine.registry.destroy(doomed);
    engine.registry.destroy(doomed_walker);
    const entt::entity squatter = engine.registry.create();
    const entt::entity squatter2 = engine.registry.create();
    assert(entt::to_entity(squatter) != entt::to_entity(squatter2));
    for (const entt::entity squat : {squatter, squatter2})
        assert(entt::to_entity(squat) == entt::to_entity(doomed)
               || entt::to_entity(squat) == entt::to_entity(doomed_walker));

    engine.Restore(cp);

    const taco::Transform &t = engine.registry.get<taco::Transform>(ball);
    assert(t.position.x == 0 && t.position.y == 10 && t.position.z == 0);
    assert(!engine.registry.valid(spawned));
    assert(!engine.registry.valid(squatter) && !engine.registry.valid(squatter2));
    assert(engine.registry.get<Untracked>(ball).value == 2);
    assert(engine.registry.get<Tracked>(ball).value == 1);
    assert(engine.registry.get<taco::Sunlight>(ball).intensity == 2.f);
    assert(engine.registry.get<taco::Sunlight>(ball).shadow_casting);

    // Not just the position: a live body's velocity has to rewind as well.
    auto close = [](Vector3 a, Vector3 b) {
        return std::fabs(a.x - b.x) < 0.001f && std::fabs(a.y - b.y) < 0.001f && std::fabs(a.z - b.z) < 0.001f;
    };
    assert(close(engine.registry.get<taco::Collider>(ball).GetPosition(), saved_pos));
    assert(close(engine.registry.get<taco::Collider>(ball).GetVelocity(), saved_vel));
    assert(close(engine.registry.get<taco::Character>(walker).GetPosition(), saved_walker));

    // The destroyed entities are back under their original handles, with their components
    // and their parked bodies — and the bodies still carry the state Jolt's stream replayed
    // into them, which only works because they were re-added to the broad phase first.
    assert(engine.registry.valid(doomed) && engine.registry.valid(doomed_walker));
    assert(engine.registry.get<taco::Transform>(doomed).position.x == 5);
    assert(close(engine.registry.get<taco::Collider>(doomed).GetPosition(), saved_doomed));
    assert(close(engine.registry.get<taco::Collider>(doomed).GetVelocity(), saved_doomed_vel));
    assert(close(engine.registry.get<taco::Character>(doomed_walker).GetPosition(), saved_doomed_walker));

    // A revived body is simulated again, not left out of the broad phase.
    for (int i = 0; i < 10; i++)
        engine.GetPhysics()->Update(1.0 / 60.0);
    assert(engine.registry.get<taco::Collider>(doomed).GetPosition().y < saved_doomed.y);

    // The clone is a fresh object; the original pointer is replaced, not mutated.
    const auto &restored =
        static_cast<CountSystem &>(*engine.registry.storage<std::shared_ptr<taco::System>>(
            entt::hashed_string{"CountSystem"}).get(ball));
    assert(restored.value == 5);

    // A checkpoint is reusable.
    engine.registry.get<taco::Transform>(ball).position = {1, 1, 1};
    engine.registry.get<taco::Collider>(ball).SetPosition({1, 1, 1});
    engine.registry.get<taco::Character>(walker).SetPosition({1, 1, 1});
    engine.registry.get<Tracked>(ball).value = 3;
    engine.Restore(cp);
    assert(engine.registry.get<taco::Transform>(ball).position.y == 10);
    assert(engine.registry.get<Tracked>(ball).value == 1);

    // Rewind() has to leave the recorder readable, so the body must come back a second time.
    assert(close(engine.registry.get<taco::Collider>(ball).GetPosition(), saved_pos));
    assert(close(engine.registry.get<taco::Collider>(ball).GetVelocity(), saved_vel));
    assert(close(engine.registry.get<taco::Character>(walker).GetPosition(), saved_walker));

    // The second Restore erased that storage slot, so `restored` above now dangles:
    // re-fetch. Cloning a clone has to work too, or a checkpoint is single-use.
    const auto &restored2 =
        static_cast<CountSystem &>(*engine.registry.storage<std::shared_ptr<taco::System>>(
            entt::hashed_string{"CountSystem"}).get(ball));
    assert(restored2.value == 5);

    // RequestRestore only queues; the drain runs in ApplyPendingRestore, which Run()
    // calls after Update. Nothing must happen until then.
    engine.registry.get<taco::Transform>(ball).position = {7, 7, 7};
    engine.RequestRestore(cp);
    assert(engine.registry.get<taco::Transform>(ball).position.y == 7);

    engine.ApplyPendingRestore();
    assert(engine.registry.get<taco::Transform>(ball).position.y == 10);

    // And the queue is empty again: a second drain must be a no-op.
    engine.registry.get<taco::Transform>(ball).position = {7, 7, 7};
    engine.ApplyPendingRestore();
    assert(engine.registry.get<taco::Transform>(ball).position.y == 7);

    std::remove("checkpoint_test_scene.json");
    printf("checkpoint test: ok\n");
}
