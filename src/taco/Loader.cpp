// tacoEngine (c) Nikolas Wipper 2026

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "Loader.h"

#include "Engine.h"

namespace taco {
Loader::Loader(Engine &engine, std::string asset_dir)
    : engine_(engine), asset_dir_(std::move(asset_dir)) {
    RegisterBuiltins();
}

void Loader::RegisterComponent(const std::string &name, ComponentLoader fn) {
    components_[name] = std::move(fn);
}

void Loader::RegisterSystem(const std::string &name, SystemFactory fn) {
    systems_[name] = std::move(fn);
}

std::string Loader::Path(const std::string &rel) const {
    return asset_dir_ + "/" + rel;
}

entt::entity Loader::Resolve(const std::string &name) const {
    auto it = named_.find(name);
    return it == named_.end() ? entt::null : it->second;
}

void Loader::RegisterBuiltins() {}
void Loader::ApplyComponents(entt::entity, const Value &) {}
void Loader::AttachSystems(entt::entity, const Value &) {}
void Loader::ExpandModel(const Value &) {}
void Loader::LoadScene(const std::string &) {}
}
