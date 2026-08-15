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

#include "taco/Checkpoint.h"
#include "taco/Engine.h"
#include "taco/Entity.h"
#include "taco/Loader.h"
#include "taco/Physics.h"
#include "taco/comp/Lights.h"
#include "taco/comp/System.h"
#include "taco/comp/Transform.h"

/// A System is an ordinary value component, so Add<T>() checkpoints it like any other.
struct CountSystem : taco::System {
    int value = 0;
};

/// Move-only, so Add<T> can't register it: the value must survive a restore untouched.
struct Untracked {
    int value;

    Untracked(int v) : value(v) {}
    Untracked(const Untracked &) = delete;
    Untracked(Untracked &&) = default;
};

/// A plain game component nobody registered by hand. Add<T> has to pick it up anyway.
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

    taco::Entity ball = loader.Resolve("ball");
    assert(ball.Valid());

    ball.Add<CountSystem>().value = 5;

    const taco::Entity walker = loader.Resolve("walker");
    assert(walker.Valid());
    walker.Get<taco::Character>().SetPosition({20, 10, 0});

    taco::Entity doomed = loader.Resolve("doomed");
    assert(doomed.Valid() && doomed != ball);
    doomed.Get<taco::Collider>().SetPosition({5, 5, 5});

    taco::Entity doomed_walker = loader.Resolve("doomed_walker");
    assert(doomed_walker.Valid());
    doomed_walker.Get<taco::Character>().SetPosition({40, 10, 0});

    ball.Get<taco::Collider>().SetPosition({0, 10, 0});

    ball.Add<Untracked>(1);

    // Never named anywhere: proves Add<T> alone is enough to checkpoint a game component.
    ball.Add<Tracked>(1);

    // Sunlight's hand-written capture (shadow_map_ must not round-trip) is registered too.
    ball.Add<taco::Sunlight>(2.f, WHITE, true);
    // ... and it has to come back on a revived entity like any other component.
    doomed.Add<taco::Sunlight>(3.f, RED, true);

    // SetPosition doesn't activate a body — without this the asserts below would only ever
    // cover a sleeping body at zero velocity.
    for (int i = 0; i < 10; i++)
        engine.GetPhysics()->Update(1.0 / 60.0);

    const Vector3 saved_pos = ball.Get<taco::Collider>().GetPosition();
    const Vector3 saved_vel = ball.Get<taco::Collider>().GetVelocity();
    const Vector3 saved_walker = walker.Get<taco::Character>().GetPosition();
    const Vector3 saved_doomed = doomed.Get<taco::Collider>().GetPosition();
    const Vector3 saved_doomed_vel = doomed.Get<taco::Collider>().GetVelocity();
    const Vector3 saved_doomed_walker = doomed_walker.Get<taco::Character>().GetPosition();
    assert(saved_pos.y < 10.f && saved_vel.y < -0.1f); // it really is falling

    // Gains a body after the capture, so the restore has to take it away again: the physics
    // stream knows nothing about it.
    taco::Entity latecomer = engine.Create();
    latecomer.Add<taco::Transform>(Vector3{0, 30, 0}, taco::Rotation(), Vector3{0, 0, 0});

    taco::Checkpoint cp = engine.Save();

    latecomer.Add<taco::Collider>(engine.GetPhysics()->CreateSphereCollider(1.0));
    latecomer.Add<taco::Character>(engine.GetPhysics()->CreateCharacter(1.8, 0.2));

    // Component data changes, plus an entity that did not exist at capture.
    ball.Get<taco::Transform>().position = {9, 9, 9};
    ball.Get<taco::Collider>().SetPosition({9, 9, 9});
    walker.Get<taco::Character>().SetPosition({0, 0, 0});
    for (int i = 0; i < 10; i++)
        engine.GetPhysics()->Update(1.0 / 60.0);
    ball.Get<CountSystem>().value = 99;
    ball.Get<taco::Sunlight>().intensity = 9.f;
    ball.Get<taco::Sunlight>().shadow_casting = false;
    ball.Get<Untracked>().value = 2;
    ball.Get<Tracked>().value = 2;
    taco::Entity spawned = engine.Create();
    spawned.Add<taco::Transform>(Vector3{1, 1, 1}, taco::Rotation(), Vector3{0, 0, 0});

    // Destroying these frees two entity indices that the creates below then recycle, so the
    // revival has to be able to take them back.
    doomed.Destroy();
    doomed_walker.Destroy();
    const taco::Entity squatter = engine.Create();
    const taco::Entity squatter2 = engine.Create();
    assert(entt::to_entity(squatter.id()) != entt::to_entity(squatter2.id()));
    for (const taco::Entity &squat : {squatter, squatter2})
        assert(entt::to_entity(squat.id()) == entt::to_entity(doomed.id())
               || entt::to_entity(squat.id()) == entt::to_entity(doomed_walker.id()));

    engine.Restore(cp);

    // The late body is gone, component and all: nothing would ever rewind it.
    assert(latecomer.Valid());
    assert(!latecomer.Has<taco::Collider>() && !latecomer.Has<taco::Character>());

    const taco::Transform &t = ball.Get<taco::Transform>();
    assert(t.position.x == 0 && t.position.y == 10 && t.position.z == 0);
    assert(!spawned.Valid());
    assert(!squatter.Valid() && !squatter2.Valid());
    assert(ball.Get<Untracked>().value == 2);
    assert(ball.Get<Tracked>().value == 1);
    assert(ball.Get<taco::Sunlight>().intensity == 2.f);
    assert(ball.Get<taco::Sunlight>().shadow_casting);

    // A System is checkpointed by Add<T> like any other component.
    assert(ball.Get<CountSystem>().value == 5);

    // Not just the position: a live body's velocity has to rewind as well.
    auto close = [](Vector3 a, Vector3 b) {
        return std::fabs(a.x - b.x) < 0.001f && std::fabs(a.y - b.y) < 0.001f && std::fabs(a.z - b.z) < 0.001f;
    };
    assert(close(ball.Get<taco::Collider>().GetPosition(), saved_pos));
    assert(close(ball.Get<taco::Collider>().GetVelocity(), saved_vel));
    assert(close(walker.Get<taco::Character>().GetPosition(), saved_walker));

    // Back under their original handles, bodies carrying the state Jolt replayed — which only
    // works because they were re-added to the broad phase first.
    assert(doomed.Valid() && doomed_walker.Valid());
    assert(doomed.Get<taco::Transform>().position.x == 5);
    assert(doomed.Has<taco::Sunlight>() && doomed.Get<taco::Sunlight>().intensity == 3.f);
    assert(close(doomed.Get<taco::Collider>().GetPosition(), saved_doomed));
    assert(close(doomed.Get<taco::Collider>().GetVelocity(), saved_doomed_vel));
    assert(close(doomed_walker.Get<taco::Character>().GetPosition(), saved_doomed_walker));

    // A revived body is simulated again, not left out of the broad phase.
    for (int i = 0; i < 10; i++)
        engine.GetPhysics()->Update(1.0 / 60.0);
    assert(doomed.Get<taco::Collider>().GetPosition().y < saved_doomed.y);

    // A checkpoint is reusable.
    ball.Get<taco::Transform>().position = {1, 1, 1};
    ball.Get<taco::Collider>().SetPosition({1, 1, 1});
    walker.Get<taco::Character>().SetPosition({1, 1, 1});
    ball.Get<Tracked>().value = 3;
    ball.Get<CountSystem>().value = 99;
    engine.Restore(cp);
    assert(ball.Get<taco::Transform>().position.y == 10);
    assert(ball.Get<Tracked>().value == 1);
    assert(ball.Get<CountSystem>().value == 5);

    // Rewind() has to leave the recorder readable, so the body must come back a second time.
    assert(close(ball.Get<taco::Collider>().GetPosition(), saved_pos));
    assert(close(ball.Get<taco::Collider>().GetVelocity(), saved_vel));
    assert(close(walker.Get<taco::Character>().GetPosition(), saved_walker));

    // RequestRestore only queues; ApplyPendingRestore, which Run() calls after Update, drains.
    ball.Get<taco::Transform>().position = {7, 7, 7};
    engine.RequestRestore(cp);
    assert(ball.Get<taco::Transform>().position.y == 7);

    engine.ApplyPendingRestore();
    assert(ball.Get<taco::Transform>().position.y == 10);

    // And the queue is empty again: a second drain must be a no-op.
    ball.Get<taco::Transform>().position = {7, 7, 7};
    engine.ApplyPendingRestore();
    assert(ball.Get<taco::Transform>().position.y == 7);

    // Save doesn't free the bodies parked since the last one, so the older cp can still revive
    // doomed after cp2 was taken.
    doomed.Destroy();
    assert(!doomed.Valid());
    taco::Checkpoint cp2 = engine.Save();

    engine.Restore(cp);
    assert(doomed.Valid() && doomed.Has<taco::Collider>());
    assert(close(doomed.Get<taco::Collider>().GetPosition(), saved_doomed));

    std::remove("checkpoint_test_scene.json");
    printf("checkpoint self-check passed\n");
}
