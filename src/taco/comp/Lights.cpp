// tacoEngine (c) Nikolas Wipper 2025

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "Lights.h"

#include <tr_effects.h>
#include <tr_generators.h>

taco::Environment::Environment(const Image &image) {
    Texture tex = LoadTextureCubemap(image, CUBEMAP_LAYOUT_AUTO_DETECT);
    radiance_ = PrefilterCubemap(tex);
    irradiance_ = IrradianceCubemap(tex);
}

taco::Environment::Environment(Image irradiance, Image radiance)
    : radiance_(LoadTextureCubemap(irradiance, CUBEMAP_LAYOUT_AUTO_DETECT)),
      irradiance_(LoadTextureCubemap(radiance, CUBEMAP_LAYOUT_AUTO_DETECT)) {}

taco::Sky::Sky(const Image &image) : skybox_(LoadSkyboxImage(image)) {}
