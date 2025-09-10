// tacoEngine (c) Nikolas Wipper 2025

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef ROTATION_H
#define ROTATION_H

#include <raymath.h>

namespace taco {
class Rotation : public Vector3 {
    mutable Quaternion cached_quat_ = QuaternionIdentity();
    mutable Vector3 cached_rot_ = Vector3Zero();

public:
    Rotation(float x = 0.f, float y = 0.f, float z = 0.f);

    Quaternion GetQuaternion() const;
    void SetFromQuaternion(Quaternion quaternion);
};
} // taco

#endif //ROTATION_H
