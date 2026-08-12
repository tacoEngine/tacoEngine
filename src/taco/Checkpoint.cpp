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
#include "Physics.h"

namespace taco {
Checkpoint Engine::Save() {
    Checkpoint cp;

    for (auto &[_, capture] : tracked_)
        capture(registry, cp);

    // Systems live in one named storage per system name, so scan every storage
    // holding shared_ptr<System> — the same walk Engine::Update's visit_systems does.
    for (auto [id, pool] : registry.storage()) {
        if (pool.type() != entt::type_id<std::shared_ptr<System>>())
            continue;

        auto system_view = entt::basic_view{registry.storage<std::shared_ptr<System>>(id)};
        for (auto [entity, system] : system_view.each())
            if (std::shared_ptr<System> clone = system->Clone())
                cp.systems_.emplace_back(id, entity, std::move(clone));
    }

    for (const entt::entity entity : registry.view<entt::entity>())
        cp.entities_.push_back(entity);

    // Jolt's own rollback support: global state, bodies, contacts and constraints.
    physics_->system_.SaveState(cp.physics_);

    for (auto [entity, character] : registry.view<Character>().each())
        character.character_->SaveState(cp.characters_[entity]);

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

    for (const auto &[id, entity, system] : cp.systems_) {
        if (!registry.valid(entity)) continue;

        auto &storage = registry.storage<std::shared_ptr<System>>(id);
        if (storage.contains(entity)) storage.erase(entity);
        // Clone again so the checkpoint stays usable for the next restore.
        storage.emplace(entity, system->Clone());
    }

    // A changed body set is only partly detectable, and that is an accepted ceiling here:
    // BodyManager::RestoreState just walks the stream, so (a) a Collider destroyed since
    // the capture aborts it midway — the bodies it already visited keep the restored state,
    // the rest keep the current one, and nothing rolls back; (b) a Character destroyed since
    // the capture leaves its Jolt body alive (~Character only removes it), so old state is
    // written into an orphaned body with no error at all; (c) a body added since the capture
    // is simply absent from the stream and silently keeps its current state.
    // Rewind() is seekg(0, beg): it clears eofbit but not failbit, and a failed stream reads
    // as zero bodies, which RestoreState reports as success — so check IsFailed() too.
    cp.physics_.Rewind();
    if (!physics_->system_.RestoreState(cp.physics_) || cp.physics_.IsFailed())
        logging::Logger::Error("[checkpoint]: failed to restore the physics state");

    for (auto &[entity, recorder] : cp.characters_) {
        if (!registry.valid(entity) || !registry.all_of<Character>(entity)) continue;

        recorder.Rewind();
        // Character::RestoreState returns nothing, so the stream state is the only signal.
        registry.get<Character>(entity).character_->RestoreState(recorder);
        if (recorder.IsFailed())
            logging::Logger::Error("[checkpoint]: failed to restore a character state");
    }
}
}
