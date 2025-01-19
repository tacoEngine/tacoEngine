// game (c) Nikolas Wipper 2025

#ifndef LIGHTS_H
#define LIGHTS_H

#include "../Graphics.h"

#include <tr_effects.h>

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

struct Environment {
    friend class Engine;

    Environment(const Image &image);
    Environment(Image irradiance, Image radiance);

private:
    TextureCubemap radiance_;
    TextureCubemap irradiance_;
};

struct Sky {
    friend class Engine;

    Sky(const Image &image);

private:
    Skybox skybox_;
};
};

#endif //LIGHTS_H
