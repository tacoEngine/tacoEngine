// game (c) Nikolas Wipper 2025

#ifndef LIGHTS_H
#define LIGHTS_H

#include "Graphics.h"

namespace taco {
struct Sunlight {
    float intensity;
    Color color;

    Sunlight(float inten = 1.f, Color col = WHITE) : intensity(inten), color(col) {}
};
};

#endif //LIGHTS_H
