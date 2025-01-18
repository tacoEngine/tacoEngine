// tacoEngine (c) Nikolas Wipper 2024

#ifndef POSITION_H
#define POSITION_H

#include <raylib.h>

namespace taco {

struct Transform {
    Vector3 position;
    Quaternion rotation;
};

}

#endif //POSITION_H
