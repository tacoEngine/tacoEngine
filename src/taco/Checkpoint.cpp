// tacoEngine (c) Nikolas Wipper 2026

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "Checkpoint.h"

#include <set>
#include <vector>

#include <log/log.h>

#include "Engine.h"

namespace taco {
Checkpoint Engine::Save() {
    Checkpoint cp;

    for (auto &[_, capture] : tracked_)
        capture(registry, cp);

    for (const entt::entity entity : registry.view<entt::entity>())
        cp.entities_.push_back(entity);

    return cp;
}

void Engine::Restore(Checkpoint &cp) {
    // Entities spawned after the capture go first: their Collider/Character
    // destructors remove the Jolt bodies, so the body set matches the recorder.
    const std::set<entt::entity> alive(cp.entities_.begin(), cp.entities_.end());
    std::vector<entt::entity> spawned;
    for (const entt::entity entity : registry.view<entt::entity>())
        if (!alive.count(entity)) spawned.push_back(entity);
    for (const entt::entity entity : spawned)
        registry.destroy(entity);

    for (const entt::entity entity : cp.entities_)
        if (!registry.valid(entity))
            logging::Logger::Error("[checkpoint]: entity destroyed since the capture cannot be restored");

    for (auto &restore : cp.restore_)
        restore(registry);
}
}
