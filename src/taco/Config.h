// game (c) Nikolas Wipper 2025

#ifndef CONFIG_H
#define CONFIG_H

#include <rl3d_effects.h>

namespace taco {
struct Config {
    bool debug_physics = false;
    float gamma_correction = 2.2f;
    ToneMapper tone_mapper;
    unsigned int shadow_map_size = 1024;
    float cascade_dist = 100.f;
};
};

#endif //CONFIG_H
