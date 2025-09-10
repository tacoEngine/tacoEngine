// tacoEngine (c) Nikolas Wipper 2025

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <tr_effects.h>

namespace taco {
struct Config {
    bool debug_physics = false;
    bool ssao = true;
    float gamma_correction = 2.2f;
    ToneMapper tone_mapper;
    unsigned int shadow_map_size = 2048;
    float cascade_dist = 100.f;
    // 0 - no filtering
    // 1 - 5x5 gauss blur
    // n - (5x5)^n gauss blur
    unsigned int shadow_map_quality = 1;
};
};

#endif //CONFIG_H
