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
#include "comp/Lights.h"

namespace taco {
// A component nobody registered would silently not restore. Say so. EnTT only creates a
// storage on first use, so a type with no live instance yet has no storage and cannot be
// warned about at Save time — hence the same scan runs again in Restore.
void Engine::WarnUntracked() const {
    for (auto [id, pool] : registry.storage()) {
        const entt::id_type type = pool.type().hash();
        if (tracked_.count(type) || ignored_.count(type)) continue;

        logging::Logger::Warning("[checkpoint]: untracked component " + std::string(pool.type().name())
                                 + ", call Engine::Track<T>() to include it");
    }
}

// Sunlight cannot go through the generic Track<T>: its shadow_map_ owns a GL fbo plus four
// heap arrays that Engine::Render unloads and reallocates whenever shadow_map_size or
// shadow_casting changes. Writing a saved copy back would reinstall already-freed handles
// and the next Render would free them a second time. So only the public value fields
// round-trip, assigned onto the live component; shadow_map_ is left alone.
// Unlike Track<T>, this does not remove Sunlight from entities that gained it after the
// capture — removing it would leak the shadow map, which has no destructor.
void Engine::TrackSunlight() {
    tracked_[entt::type_id<Sunlight>().hash()] = [](const entt::registry &registry, Checkpoint &cp) {
        std::map<entt::entity, std::tuple<float, Color, bool>> data;
        for (auto [entity, sun] : registry.view<const Sunlight>().each())
            data.emplace(entity, std::tuple {sun.intensity, sun.color, sun.shadow_casting});

        cp.restore_.emplace_back([data = std::move(data)](entt::registry &reg) {
            for (const auto &[entity, values] : data) {
                if (!reg.valid(entity) || !reg.all_of<Sunlight>(entity)) continue;
                Sunlight &sun = reg.get<Sunlight>(entity);
                std::tie(sun.intensity, sun.color, sun.shadow_casting) = values;
            }
        });
    };
}

Checkpoint Engine::Save() {
    Checkpoint cp;

    WarnUntracked();

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

// A System phase hook is the only place game code holds an Engine *, but Restore
// destroys entities and rewrites the shared_ptr<System> storages that Update is
// iterating right then. So systems queue the restore and Run applies it after Update.
void Engine::RequestRestore(Checkpoint &cp) {
    pending_restore_ = &cp;
}

void Engine::ApplyPendingRestore() {
    // Clear first: the pointer must not survive the restore it triggered, or the next
    // frame would restore again. Restore itself never runs systems, so nothing can
    // re-request in between.
    if (Checkpoint *pending = pending_restore_) {
        pending_restore_ = nullptr;
        Restore(*pending);
    }
}

void Engine::Restore(Checkpoint &cp) {
    // A type first emplaced after the capture has no storage at Save time, so this is the
    // only place its missing Track<T>() can be reported.
    WarnUntracked();

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
    // the capture behaves the same way — taco::~Character removes the body and then
    // ~JPH::Character destroys it, so the body is gone from the manager just like (a);
    // (c) a body added since the capture is simply absent from the stream and silently
    // keeps its current state.
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
