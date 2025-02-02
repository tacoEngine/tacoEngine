// tacoEngine (c) Nikolas Wipper 2024

#ifndef POSITION_H
#define POSITION_H

#include <raylib.h>

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
