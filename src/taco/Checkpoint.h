// tacoEngine (c) Nikolas Wipper 2026

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef CHECKPOINT_H
#define CHECKPOINT_H

#include <functional>
#include <unordered_map>
#include <set>
#include <vector>

#include <entt/entt.hpp>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/StateRecorderImpl.h>

namespace taco {
class PhysicsEngine;

/// In-memory snapshot. Move-only (JPH::StateRecorderImpl is) and only valid for the run that
/// made it: aliases GPU handles and Jolt body ids by value. Reusable until a restore fails —
/// Rewind() doesn't clear failbit, and nothing else does either.
class Checkpoint {
    friend class Engine;

    /// Entities alive at capture.
    std::vector<entt::entity> entities_;
    /// One closure per tracked type, holding its saved values. Systems ride along as components.
    std::vector<std::function<void(entt::registry &)>> restore_;
    JPH::StateRecorderImpl physics_;
    /// Who held a Collider at capture — a parked body only goes back to its old owner.
    std::set<entt::entity> colliders_;
    std::unordered_map<entt::entity, JPH::StateRecorderImpl> characters_;

    /// Entities, plus the closures Engine handed over. Which types those cover is Engine's call.
    void CaptureECS(const entt::registry &registry);
    void RestoreECS(entt::registry &registry);
    /// Split from the ECS half so Engine can hand back parked bodies in between.
    void CapturePhysics(PhysicsEngine &physics, const entt::registry &registry);
    void RestorePhysics(PhysicsEngine &physics, const entt::registry &registry);

public:
    Checkpoint() = default;
    Checkpoint(Checkpoint &&) = default;
};
}

#endif //CHECKPOINT_H
