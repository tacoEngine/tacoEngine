// tacoEngine (c) Nikolas Wipper 2026

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef CHECKPOINT_H
#define CHECKPOINT_H

#include <functional>
#include <map>
#include <memory>
#include <set>
#include <tuple>
#include <vector>

#include <entt/entt.hpp>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/StateRecorderImpl.h>

#include "comp/System.h"

namespace taco {
/// In-memory snapshot of the engine state. Move-only (JPH::StateRecorderImpl is),
/// reusable, and only valid for the run that produced it: it aliases GPU handles
/// and Jolt body ids by value. Reusable up to the first failure: Rewind() does not
/// clear failbit, so once a restore poisons a recorder every later Restore on this
/// checkpoint fails too, and there is no way to reset it.
class Checkpoint {
    friend class Engine;

    /// Entities alive at capture.
    std::vector<entt::entity> entities_;
    /// One closure per tracked component type, holding that type's saved values.
    std::vector<std::function<void(entt::registry &)>> restore_;
    /// Cloned systems: storage id (systems live in named storages), entity, clone.
    std::vector<std::tuple<entt::id_type, entt::entity, std::shared_ptr<System>>> systems_;
    JPH::StateRecorderImpl physics_;
    /// Entities holding a Collider at capture. A body parked by Engine::RetireCollider is
    /// only handed back to one of these; characters_ below does the same job for Characters.
    std::set<entt::entity> colliders_;
    /// Keyed by entity so iteration order cannot cross-apply state between characters.
    std::map<entt::entity, JPH::StateRecorderImpl> characters_;

public:
    Checkpoint() = default;
    Checkpoint(Checkpoint &&) = default;
};
}

#endif //CHECKPOINT_H
