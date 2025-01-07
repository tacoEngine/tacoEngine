// game (c) Nikolas Wipper 2025

#ifndef LIGHTS_H
#define LIGHTS_H

#include "Graphics.h"

namespace taco {
struct Sunlight {
    friend class Engine;

    float intensity;
    Color color;
    bool shadow_casting;

    Sunlight(float inten = 1.f, Color col = WHITE, bool sh = true) : intensity(inten), color(col), shadow_casting(sh) {}

private:
    ShadowMap shadow_map_ = {0};
};
};

#endif //LIGHTS_H
