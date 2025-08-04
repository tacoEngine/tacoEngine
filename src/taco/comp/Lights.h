// game (c) Nikolas Wipper 2025

#ifndef LIGHTS_H
#define LIGHTS_H

#include "taco/Graphics.h"

#include <tr_effects.h>

namespace taco {
class Sunlight {
    friend class Engine;

    ShadowMap shadow_map_ = {0};
public:

    float intensity;
    Color color;
    bool shadow_casting;

    Sunlight(float inten = 1.f, Color col = WHITE, bool sh = true) : intensity(inten), color(col), shadow_casting(sh) {}
};

class Environment {
    friend class Engine;

    TextureCubemap radiance_;
    TextureCubemap irradiance_;

public:
    Environment(const Image &image);
    Environment(Image irradiance, Image radiance);
};

class Sky {
    friend class Engine;

    Skybox skybox_;
public:

    Sky(const Image &image);
};
};

#endif //LIGHTS_H
