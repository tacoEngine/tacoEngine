// tacoEngine (c) Nikolas Wipper 2024-2025

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef POSITION_H
#define POSITION_H

#include "taco/misc/Rotation.h"

namespace taco {
struct Transform {
    Vector3 position;
    Rotation rotation;
    Vector3 velocity;
};

struct Link {
    entt::entity entity;
    bool linkPosX, linkPosY, linkPosZ;
    bool linkRotX, linkRotY, linkRotZ;
    bool linkVelX, linkVelY, linkVelZ;
};
}

#endif //POSITION_H
