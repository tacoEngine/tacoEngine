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
#include <tuple>
#include <vector>

#include <entt/entt.hpp>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/StateRecorderImpl.h>

#include "comp/System.h"

namespace taco {
/// In-memory snapshot of the engine state. Move-only (JPH::StateRecorderImpl is),
/// reusable, and only valid for the run that produced it: it aliases GPU handles
/// and Jolt body ids by value.
class Checkpoint {
    friend class Engine;

    /// Entities alive at capture.
    std::vector<entt::entity> entities_;
    /// One closure per tracked component type, holding that type's saved values.
    std::vector<std::function<void(entt::registry &)>> restore_;
    /// Cloned systems: storage id (systems live in named storages), entity, clone.
    std::vector<std::tuple<entt::id_type, entt::entity, std::shared_ptr<System>>> systems_;
    JPH::StateRecorderImpl physics_;
    /// Keyed by entity so iteration order cannot cross-apply state between characters.
    std::map<entt::entity, JPH::StateRecorderImpl> characters_;

public:
    Checkpoint() = default;
    Checkpoint(Checkpoint &&) = default;
};
}

#endif //CHECKPOINT_H
