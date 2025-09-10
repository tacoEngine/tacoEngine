// tacoEngine (c) Nikolas Wipper 2025

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "Rotation.h"

#include "raymath.h"

namespace taco {
Rotation::Rotation(float x, float y, float z) : Vector3(x, y, z) {}

Quaternion Rotation::GetQuaternion() const {
    if (cached_rot_.x != x || cached_rot_.y != y || cached_rot_.z != z) {
        cached_quat_ = QuaternionFromEuler(x, y, z);
        cached_rot_ = Vector3(x, y, z);
    }
    return cached_quat_;
}

void Rotation::SetFromQuaternion(Quaternion q) {
    Vector3 euler = QuaternionToEuler(q);
    x = euler.x;
    y = euler.y;
    z = euler.z;
}
} // taco
