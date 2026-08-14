// tacoEngine (c) Nikolas Wipper 2026

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#ifndef LOADER_H
#define LOADER_H

#include <functional>
#include <map>
#include <memory>
#include <string>

#include <entt/entt.hpp>
#include <rapidjson/document.h>

#include "Entity.h"

namespace taco {
class Engine;

class Loader {
public:
    using Value = rapidjson::Value;
    using ComponentLoader = std::function<void(Loader &, Entity, const Value &)>;

    Loader(Engine &engine, std::string asset_dir);

    void RegisterComponent(const std::string &name, ComponentLoader fn);

    /// Register a type that takes no scene parameters — systems, mostly. The scene's value
    /// for this key is ignored; write a RegisterComponent lambda if you need it.
    template<class T>
    void Register(const std::string &name) {
        RegisterComponent(name, [](Loader &, Entity e, const Value &) { e.Add<T>(); });
    }

    void LoadScene(const std::string &rel_path);

    Engine &engine() { return engine_; }
    std::string Path(const std::string &rel) const;
    Entity Resolve(const std::string &name) const;

private:
    void RegisterBuiltins();
    void ApplyComponents(Entity entity, const Value &spec);
    void ApplySystems(Entity entity, const Value &list);
    void ExpandModel(const Value &spec);

    Engine &engine_;
    std::string asset_dir_;
    std::map<std::string, ComponentLoader> components_;
    std::map<std::string, Entity> named_;
};
}

#endif //LOADER_H
