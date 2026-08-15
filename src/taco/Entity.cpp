// game (c) Nikolas Wipper 2026

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <algorithm>

#include "Entity.h"
#include "Engine.h"

namespace taco {
void detail::RegisterSystem(Engine *engine, const SystemHooks hooks) {
    if (std::ranges::find(engine->system_hooks_, hooks.early, &SystemHooks::early) == engine->system_hooks_.end())
        engine->system_hooks_.push_back(hooks);
}

void detail::TrackComponent(Engine *engine, const entt::id_type type, const CaptureFn capture) {
    if (!engine->ignored_.count(type))
        engine->tracked_.try_emplace(type, capture);
}
}
