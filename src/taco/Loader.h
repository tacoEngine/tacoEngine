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

#include "comp/System.h"

namespace taco {
class Engine;

class Loader {
public:
    using Value = rapidjson::Value;
    using ComponentLoader = std::function<void(Loader &, entt::entity, const Value &)>;
    using SystemFactory = std::function<std::shared_ptr<System>(const Value &)>;

    Loader(Engine &engine, std::string asset_dir);

    void RegisterComponent(const std::string &name, ComponentLoader fn);
    void RegisterSystem(const std::string &name, SystemFactory fn);

    template<class T>
    void RegisterSystem(const std::string &name) {
        RegisterSystem(name, [](const Value &) { return std::make_shared<T>(); });
    }

    void LoadScene(const std::string &rel_path);

    Engine &engine() { return engine_; }
    std::string Path(const std::string &rel) const;
    entt::entity Resolve(const std::string &name) const;

private:
    void RegisterBuiltins();
    void ApplyComponents(entt::entity entity, const Value &spec);
    void AttachSystems(entt::entity entity, const Value &list);
    void ExpandModel(const Value &spec);

    Engine &engine_;
    std::string asset_dir_;
    std::map<std::string, ComponentLoader> components_;
    std::map<std::string, SystemFactory> systems_;
    std::map<std::string, entt::entity> named_;
};
}

#endif //LOADER_H
