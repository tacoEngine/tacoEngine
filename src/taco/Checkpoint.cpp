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

#include "Physics.h"

namespace taco {
void Checkpoint::CaptureECS(const entt::registry &registry) {
    for (const entt::entity entity : registry.view<entt::entity>())
        entities_.push_back(entity);
}

void Checkpoint::RestoreECS(entt::registry &registry) {
    const std::set<entt::entity> alive(entities_.begin(), entities_.end());
    std::vector<entt::entity> spawned;
    for (const entt::entity entity : registry.view<entt::entity>())
        if (!alive.contains(entity)) spawned.push_back(entity);
    for (const entt::entity entity : spawned)
        registry.destroy(entity);

    for (const entt::entity entity : entities_) {
        if (registry.valid(entity)) continue;

        if (const entt::entity revived = registry.create(entity); revived != entity) {
            registry.destroy(revived);
            logging::Logger::Error("[checkpoint]: entity index in use, cannot restore entity");
        }
    }

    for (auto &restore : restore_)
        restore(registry);
}

void Checkpoint::CapturePhysics(PhysicsEngine &physics, const entt::registry &registry) {
    physics.system_.SaveState(physics_);

    for (const entt::entity entity : registry.view<Collider>())
        colliders_.insert(entity);

    for (auto [entity, character] : registry.view<Character>().each())
        character.character_->SaveState(characters_[entity]);
}

void Checkpoint::RestorePhysics(PhysicsEngine &physics, const entt::registry &registry) {
    // failbit survives Rewind(), and a dead stream reads as zero bodies — which RestoreState
    // calls success. Hence IsFailed().
    physics_.Rewind();
    if (!physics.system_.RestoreState(physics_) || physics_.IsFailed())
        logging::Logger::Error("[checkpoint]: failed to restore the physics state");

    for (auto &[entity, recorder] : characters_) {
        if (!registry.valid(entity) || !registry.all_of<Character>(entity)) continue;

        recorder.Rewind();
        // RestoreState returns void; the stream is the only signal.
        registry.get<const Character>(entity).character_->RestoreState(recorder);
        if (recorder.IsFailed())
            logging::Logger::Error("[checkpoint]: failed to restore a character state");
    }
}
}
