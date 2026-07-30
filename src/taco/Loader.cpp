// tacoEngine (c) Nikolas Wipper 2026

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "Loader.h"

#include <cassert>
#include <fstream>
#include <set>
#include <sstream>

#include <raylib.h>
#include <raymath.h>
#include <entt/core/hashed_string.hpp>

#include "Engine.h"
#include "comp/Camera.h"
#include "comp/Lights.h"
#include "comp/Transform.h"

namespace {
using Value = rapidjson::Value;

Vector3 GetVec3(const Value &v, const char *key, Vector3 def) {
    if (!v.HasMember(key) || !v[key].IsArray()) return def;
    const auto a = v[key].GetArray();
    return {a[0].GetFloat(), a[1].GetFloat(), a[2].GetFloat()};
}

float GetFloat(const Value &v, const char *key, float def) {
    return v.HasMember(key) ? v[key].GetFloat() : def;
}

bool GetBool(const Value &v, const char *key, bool def) {
    return v.HasMember(key) ? v[key].GetBool() : def;
}

Color GetColor(const Value &a, Color def) {
    if (!a.IsArray()) return def;
    const auto arr = a.GetArray();
    Color c = {(unsigned char) arr[0].GetInt(),
               (unsigned char) arr[1].GetInt(),
               (unsigned char) arr[2].GetInt(),
               (unsigned char) (arr.Size() > 3 ? arr[3].GetInt() : 255)};
    return c;
}

void GetBool3(const Value &v, const char *key, bool &x, bool &y, bool &z) {
    x = y = z = false;
    if (!v.HasMember(key) || !v[key].IsArray()) return;
    const auto a = v[key].GetArray();
    x = a[0].GetBool();
    y = a[1].GetBool();
    z = a[2].GetBool();
}
}

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

void Loader::LoadScene(const std::string &rel_path) {
    std::ifstream in(Path(rel_path));
    std::stringstream ss;
    ss << in.rdbuf();
    const std::string text = ss.str();

    rapidjson::Document doc;
    doc.Parse(text.c_str());
    assert(!doc.HasParseError() && doc.HasMember("entities"));

    const Value &entities = doc["entities"];
    named_.clear();

    // Pass 1: create and name every non-model entity.
    for (auto it = entities.MemberBegin(); it != entities.MemberEnd(); ++it) {
        if (it->value.HasMember("model")) continue;
        named_[it->name.GetString()] = engine_.registry.create();
    }

    // Pass 2: apply components (names now resolvable) and attach systems.
    for (auto it = entities.MemberBegin(); it != entities.MemberEnd(); ++it) {
        if (it->value.HasMember("model")) {
            ExpandModel(it->value);
            continue;
        }
        const entt::entity e = named_[it->name.GetString()];
        ApplyComponents(e, it->value);
        if (it->value.HasMember("systems"))
            AttachSystems(e, it->value["systems"]);
    }
}

void Loader::ApplyComponents(entt::entity entity, const Value &spec) {
    // Fixed order so mesh-dependent components (Collider) and name-dependent
    // ones (Link) see what they need. JSON object order is not guaranteed.
    static const char *order[] = {"Transform", "Mesh", "Material", "Collider", "Character"};
    std::set<std::string> done;

    auto apply = [&](const std::string &name, const Value &v) {
        auto f = components_.find(name);
        if (f != components_.end()) f->second(*this, entity, v);
        done.insert(name);
    };

    for (const char *name : order)
        if (spec.HasMember(name)) apply(name, spec[name]);

    for (auto it = spec.MemberBegin(); it != spec.MemberEnd(); ++it) {
        const std::string key = it->name.GetString();
        if (key == "systems" || key == "model" || done.count(key)) continue;
        apply(key, it->value);
    }
}

void Loader::AttachSystems(entt::entity entity, const Value &list) {
    static const Value null_params;
    auto attach = [&](const std::string &name, const Value &params) {
        auto f = systems_.find(name);
        if (f == systems_.end()) return;
        engine_.registry.storage<std::shared_ptr<System>>(entt::hashed_string{name.c_str()})
                .emplace(entity, f->second(params));
    };

    if (list.IsArray())
        for (const auto &s : list.GetArray()) attach(s.GetString(), null_params);
    else if (list.IsObject())
        for (auto it = list.MemberBegin(); it != list.MemberEnd(); ++it)
            attach(it->name.GetString(), it->value);
}

void Loader::RegisterBuiltins() {
    RegisterComponent("Transform", [](Loader &l, entt::entity e, const Value &v) {
        const Vector3 pos = GetVec3(v, "position", {0, 0, 0});
        const Vector3 rot = GetVec3(v, "rotation", {0, 0, 0});
        const Vector3 vel = GetVec3(v, "velocity", {0, 0, 0});
        l.engine().registry.emplace<Transform>(
            e, pos, Rotation(rot.x * DEG2RAD, rot.y * DEG2RAD, rot.z * DEG2RAD), vel);
    });

    RegisterComponent("Camera", [](Loader &l, entt::entity e, const Value &v) {
        l.engine().registry.emplace<Camera>(e, GetFloat(v, "fov", 72.f));
    });

    RegisterComponent("Sunlight", [](Loader &l, entt::entity e, const Value &v) {
        const float intensity = GetFloat(v, "intensity", 1.f);
        const Color color = v.HasMember("color") ? GetColor(v["color"], WHITE) : WHITE;
        const bool shadow = GetBool(v, "shadow", true);
        l.engine().registry.emplace<Sunlight>(e, intensity, color, shadow);
    });

    RegisterComponent("Environment", [](Loader &l, entt::entity e, const Value &v) {
        Image img = LoadImage(l.Path(v["hdr"].GetString()).c_str());
        l.engine().registry.emplace<Environment>(e, img);
        UnloadImage(img);
    });

    RegisterComponent("Sky", [](Loader &l, entt::entity e, const Value &v) {
        Image img = LoadImage(l.Path(v["hdr"].GetString()).c_str());
        l.engine().registry.emplace<Sky>(e, img);
        UnloadImage(img);
    });

    RegisterComponent("Link", [](Loader &l, entt::entity e, const Value &v) {
        const entt::entity target = l.Resolve(v["target"].GetString());
        bool px, py, pz, rx, ry, rz, vx, vy, vz;
        GetBool3(v, "pos", px, py, pz);
        GetBool3(v, "rot", rx, ry, rz);
        GetBool3(v, "vel", vx, vy, vz);
        l.engine().registry.emplace<Link>(e, target, px, py, pz, rx, ry, rz, vx, vy, vz);
    });

    RegisterComponent("Character", [](Loader &l, entt::entity e, const Value &v) {
        l.engine().registry.emplace<Character>(
            e, l.engine().GetPhysics()->CreateCharacter(GetFloat(v, "height", 1.8f),
                                                        GetFloat(v, "radius", 0.2f)));
    });

    RegisterComponent("Collider", [](Loader &l, entt::entity e, const Value &v) {
        auto physics = l.engine().GetPhysics();
        if (v.HasMember("sphere")) {
            l.engine().registry.emplace<Collider>(e, physics->CreateSphereCollider(v["sphere"].GetFloat()));
        } else if (GetBool(v, "mesh", false)) {
            Mesh &m = l.engine().registry.get<Mesh>(e);
            l.engine().registry.emplace<Collider>(e, physics->CreateMeshCollider(m, GetBool(v, "dynamic", true)));
        }
    });

    // Mesh and Material are registered in Task 4.
}

void Loader::ExpandModel(const Value &) {}
}
