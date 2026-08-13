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

// A Jolt body cannot be rebuilt from a checkpoint — its shape, its mass overrides and its
// body id all die with it — so a destroyed physics entity does not destroy its body. It only
// leaves the broad phase and its handle is parked here until a Restore hands it back.
// BodyManager::SaveState skips bodies that are not in the broad phase, so a parked body is
// invisible to every later checkpoint and costs no simulation time.
// Moving the handle out empties its physics_, which is exactly what ~Collider tests, so the
// husk left behind on the entity destructs without touching the body.
// The key cannot collide: a parked handle only becomes reachable again through Restore, which
// takes it out of the map on the way.
// Before the first Save there is nothing to be restored to, so destruction stays destruction
// and a game that never checkpoints never pays for any of this.
void Engine::RetireCollider(entt::registry &reg, const entt::entity entity) {
    if (!checkpointed_) return;

    Collider &collider = reg.get<Collider>(entity);
    physics_->body_interface_.RemoveBody(collider.body_id_);
    retired_colliders_.emplace(entity, std::move(collider));
}

void Engine::RetireCharacter(entt::registry &reg, const entt::entity entity) {
    if (!checkpointed_) return;

    Character &character = reg.get<Character>(entity);
    physics_->body_interface_.RemoveBody(character.character_->GetBodyID());
    retired_characters_.emplace(entity, std::move(character));
}

// Dropping a parked handle takes care: ~Collider and ~Character both remove the body from
// the broad phase, and a parked body is already out — Jolt rejects the second removal. So
// destroy the body here and empty the handle, which is what both destructors no-op on.
// ~JPH::Character destroys the character's body itself, so that one only needs disarming.
void Engine::ClearRetired() {
    for (auto &[entity, collider] : retired_colliders_) {
        physics_->body_interface_.DestroyBody(collider.body_id_);
        collider.physics_.reset();
    }
    retired_colliders_.clear();

    for (auto &[entity, character] : retired_characters_)
        character.physics_.reset();
    retired_characters_.clear();
}

Checkpoint Engine::Save() {
    Checkpoint cp;

    WarnUntracked();

    // Nothing parked before this capture can be brought back by it: the entity is not in
    // entities_, so Restore would drop the handle anyway. Retiring only starts here, which
    // is also why a game that never checkpoints never pays for it.
    ClearRetired();
    checkpointed_ = true;

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

    for (const entt::entity entity : registry.view<Collider>())
        cp.colliders_.insert(entity);

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

    // Bring back the entities destroyed since the capture. create(hint) returns the exact
    // handle when its index is free, and here it always is: anything that could have
    // recycled the index was spawned after the capture and was just destroyed above.
    for (const entt::entity entity : cp.entities_) {
        if (registry.valid(entity)) continue;

        if (const entt::entity revived = registry.create(entity); revived != entity) {
            registry.destroy(revived);
            logging::Logger::Error("[checkpoint]: entity index in use, cannot restore entity");
        }
    }

    for (auto &restore : cp.restore_)
        restore(registry);

    for (const auto &[id, entity, system] : cp.systems_) {
        if (!registry.valid(entity)) continue;

        auto &storage = registry.storage<std::shared_ptr<System>>(id);
        if (storage.contains(entity)) storage.erase(entity);
        // Clone again so the checkpoint stays usable for the next restore.
        storage.emplace(entity, system->Clone());
    }

    // Hand the parked bodies back, and destroy whatever is left over — those handles belong
    // to entities this checkpoint never saw, so it can never revive them. Both halves have to
    // happen before RestoreState: a revived body must be in the broad phase for the stream to
    // find it, and a leftover must be out of the body manager before the stream is replayed.
    for (auto &[entity, collider] : retired_colliders_) {
        if (cp.colliders_.count(entity) && registry.valid(entity) && !registry.all_of<Collider>(entity)) {
            // The body kept its id, shape and mass the whole time; RestoreState sets the
            // rest, including whether it is awake, so it goes back in deactivated.
            physics_->body_interface_.AddBody(collider.body_id_, JPH::EActivation::DontActivate);
            registry.emplace<Collider>(entity, std::move(collider));
        } else {
            physics_->body_interface_.DestroyBody(collider.body_id_);
            collider.physics_.reset();
        }
    }
    retired_colliders_.clear();

    for (auto &[entity, character] : retired_characters_) {
        if (cp.characters_.count(entity) && registry.valid(entity) && !registry.all_of<Character>(entity)) {
            character.character_->AddToPhysicsSystem(JPH::EActivation::DontActivate);
            registry.emplace<Character>(entity, std::move(character));
        } else {
            character.physics_.reset(); // ~JPH::Character still destroys the body
        }
    }
    retired_characters_.clear();

    // A body added since the capture is absent from the stream and silently keeps its current
    // state — an accepted ceiling. A missing one no longer is: every body the stream names is
    // either still attached or was just handed back above.
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
