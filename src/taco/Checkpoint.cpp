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
    // Systems live in one named storage per system name, so scan every storage
    // holding shared_ptr<System> — the same walk Engine::Update's visit_systems does.
    for (auto [id, pool] : registry.storage()) {
        if (pool.type() != entt::type_id<std::shared_ptr<System>>())
            continue;

        // Non-null: the id came from iterating the storages, and the type just matched.
        auto system_view = entt::basic_view{*registry.storage<std::shared_ptr<System>>(id)};
        for (auto [entity, system] : system_view.each())
            if (std::shared_ptr<System> clone = system->Clone())
                systems_.emplace_back(id, entity, std::move(clone));
    }

    for (const entt::entity entity : registry.view<entt::entity>())
        entities_.push_back(entity);
}

void Checkpoint::RestoreECS(entt::registry &registry) {
    // Entities spawned after the capture go first: their Collider/Character
    // destructors take the Jolt bodies out of the broad phase, so the body set
    // matches the recorder.
    const std::set<entt::entity> alive(entities_.begin(), entities_.end());
    std::vector<entt::entity> spawned;
    for (const entt::entity entity : registry.view<entt::entity>())
        if (!alive.count(entity)) spawned.push_back(entity);
    for (const entt::entity entity : spawned)
        registry.destroy(entity);

    // Bring back the entities destroyed since the capture. create(hint) returns the exact
    // handle when its index is free, and here it always is: anything that could have
    // recycled the index was spawned after the capture and was just destroyed above.
    for (const entt::entity entity : entities_) {
        if (registry.valid(entity)) continue;

        if (const entt::entity revived = registry.create(entity); revived != entity) {
            registry.destroy(revived);
            logging::Logger::Error("[checkpoint]: entity index in use, cannot restore entity");
        }
    }

    for (auto &restore : restore_)
        restore(registry);

    for (const auto &[id, entity, system] : systems_) {
        if (!registry.valid(entity)) continue;

        auto &storage = registry.storage<std::shared_ptr<System>>(id);
        if (storage.contains(entity)) storage.erase(entity);
        // Clone again so the checkpoint stays usable for the next restore.
        storage.emplace(entity, system->Clone());
    }
}

void Checkpoint::CapturePhysics(PhysicsEngine &physics, const entt::registry &registry) {
    // Jolt's own rollback support: global state, bodies, contacts and constraints.
    physics.system_.SaveState(physics_);

    // Which entities owned a body, so Restore only hands a parked one back to its owner.
    for (const entt::entity entity : registry.view<Collider>())
        colliders_.insert(entity);

    for (auto [entity, character] : registry.view<Character>().each())
        character.character_->SaveState(characters_[entity]);
}

void Checkpoint::RestorePhysics(PhysicsEngine &physics, const entt::registry &registry) {
    // A body added since the capture is absent from the stream and silently keeps its current
    // state — an accepted ceiling. A missing one is not one: every body the stream names is
    // either still attached or was handed back by Engine::HandBackRetired just before this.
    // Rewind() is seekg(0, beg): it clears eofbit but not failbit, and a failed stream reads
    // as zero bodies, which RestoreState reports as success — so check IsFailed() too.
    physics_.Rewind();
    if (!physics.system_.RestoreState(physics_) || physics_.IsFailed())
        logging::Logger::Error("[checkpoint]: failed to restore the physics state");

    for (auto &[entity, recorder] : characters_) {
        if (!registry.valid(entity) || !registry.all_of<Character>(entity)) continue;

        recorder.Rewind();
        // Character::RestoreState returns nothing, so the stream state is the only signal.
        registry.get<const Character>(entity).character_->RestoreState(recorder);
        if (recorder.IsFailed())
            logging::Logger::Error("[checkpoint]: failed to restore a character state");
    }
}
}
