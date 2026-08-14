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
#include <rlgl.h>

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

Mesh GenNamedMesh(const std::string &type, const Value &v) {
    if (type == "sphere")
        return GenMeshSphere(GetFloat(v, "radius", 1.f),
                             (int) GetFloat(v, "rings", 20), (int) GetFloat(v, "slices", 20));
    if (type == "plane") {
        const Vector3 s = GetVec3(v, "size", {1, 1, 1});
        return GenMeshPlane(s.x, s.z, (int) GetFloat(v, "resX", 1), (int) GetFloat(v, "resZ", 1));
    }
    const Vector3 s = GetVec3(v, "size", {1, 1, 1}); // default: cube
    return GenMeshCube(s.x, s.y, s.z);
}

Texture2D SolidTexture(Color c) {
    return LoadTextureFromImage(GenImageColor(1, 1, c));
}

void FinishMaterial(Material &mat) {
    mat.shader = GetGBufferShader();
    GenTextureMipmaps(&mat.maps[MATERIAL_MAP_ALBEDO].texture);
    GenTextureMipmaps(&mat.maps[MATERIAL_MAP_NORMAL].texture);
    SetTextureFilter(mat.maps[MATERIAL_MAP_ALBEDO].texture, TEXTURE_FILTER_TRILINEAR);
    SetTextureFilter(mat.maps[MATERIAL_MAP_ALBEDO].texture, TEXTURE_FILTER_ANISOTROPIC_16X);
    SetTextureFilter(mat.maps[MATERIAL_MAP_NORMAL].texture, TEXTURE_FILTER_TRILINEAR);
    SetTextureFilter(mat.maps[MATERIAL_MAP_NORMAL].texture, TEXTURE_FILTER_ANISOTROPIC_16X);
}

Texture2D MapTexture(taco::Loader &l, const Value &v, const char *field, Color fallback) {
    if (v.HasMember(field)) {
        const Value &f = v[field];
        if (f.IsString()) {
            Texture2D t = LoadTexture(l.Path(f.GetString()).c_str());
            return t.id == 0 ? SolidTexture(fallback) : t;
        }
        if (f.IsArray()) return SolidTexture(GetColor(f, fallback));
    }
    return SolidTexture(fallback);
}

void FixModelMaterial(Material &mat) {
    auto &maps = mat.maps;
    if (maps[MATERIAL_MAP_ALBEDO].texture.id == rlGetTextureIdDefault())
        maps[MATERIAL_MAP_ALBEDO].texture.id = 0;

    if (maps[MATERIAL_MAP_ALBEDO].texture.id == 0)
        maps[MATERIAL_MAP_ALBEDO].texture = LoadTextureFromImage(GenImageColor(4, 4, LIGHTGRAY));
    if (maps[MATERIAL_MAP_NORMAL].texture.id == 0)
        maps[MATERIAL_MAP_NORMAL].texture = SolidTexture(Color{128, 128, 255, 255});
    if (maps[MATERIAL_MAP_METALNESS].texture.id == 0)
        maps[MATERIAL_MAP_METALNESS].texture = SolidTexture(WHITE);
    if (maps[MATERIAL_MAP_ROUGHNESS].texture.id == 0)
        maps[MATERIAL_MAP_ROUGHNESS].texture = SolidTexture(Color{217, 217, 217, 255});
    if (maps[MATERIAL_MAP_EMISSION].texture.id == 0)
        maps[MATERIAL_MAP_EMISSION].texture = SolidTexture(BLACK);
    if (maps[MATERIAL_MAP_OCCLUSION].texture.id == 0)
        maps[MATERIAL_MAP_OCCLUSION].texture = SolidTexture(WHITE);

    FinishMaterial(mat);
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

std::string Loader::Path(const std::string &rel) const {
    return asset_dir_ + "/" + rel;
}

Entity Loader::Resolve(const std::string &name) const {
    auto it = named_.find(name);
    return it == named_.end() ? Entity() : it->second;
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
        named_[it->name.GetString()] = engine_.Create();
    }

    // Pass 2: apply components (names now resolvable) and attach systems.
    for (auto it = entities.MemberBegin(); it != entities.MemberEnd(); ++it) {
        if (it->value.HasMember("model")) {
            ExpandModel(it->value);
            continue;
        }
        const Entity e = named_[it->name.GetString()];
        ApplyComponents(e, it->value);
        if (it->value.HasMember("systems"))
            ApplySystems(e, it->value["systems"]);
    }
}

void Loader::ApplyComponents(Entity entity, const Value &spec) {
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

// Both spellings the scene format has always accepted:
//   "systems": ["LookSystem", "AimSystem"]        - no parameters
//   "systems": {"LookSystem": {...}}              - the object is the component Value
// Either way the name is looked up in the one component map, so a system registered with
// Register<T> and a component registered with RegisterComponent are indistinguishable here.
void Loader::ApplySystems(Entity entity, const Value &list) {
    static const Value null_params;

    auto apply = [&](const std::string &name, const Value &params) {
        auto f = components_.find(name);
        if (f != components_.end()) f->second(*this, entity, params);
    };

    if (list.IsArray())
        for (const auto &s : list.GetArray()) apply(s.GetString(), null_params);
    else if (list.IsObject())
        for (auto it = list.MemberBegin(); it != list.MemberEnd(); ++it)
            apply(it->name.GetString(), it->value);
}

void Loader::RegisterBuiltins() {
    RegisterComponent("Transform", [](Loader &l, Entity e, const Value &v) {
        const Vector3 pos = GetVec3(v, "position", {0, 0, 0});
        const Vector3 rot = GetVec3(v, "rotation", {0, 0, 0});
        const Vector3 vel = GetVec3(v, "velocity", {0, 0, 0});
        e.Add<Transform>(pos, Rotation(rot.x * DEG2RAD, rot.y * DEG2RAD, rot.z * DEG2RAD), vel);
    });

    RegisterComponent("Camera", [](Loader &l, Entity e, const Value &v) {
        e.Add<Camera>(GetFloat(v, "fov", 72.f));
    });

    RegisterComponent("Sunlight", [](Loader &l, Entity e, const Value &v) {
        const float intensity = GetFloat(v, "intensity", 1.f);
        const Color color = v.HasMember("color") ? GetColor(v["color"], WHITE) : WHITE;
        const bool shadow = GetBool(v, "shadow", true);
        e.Add<Sunlight>(intensity, color, shadow);
    });

    RegisterComponent("Environment", [](Loader &l, Entity e, const Value &v) {
        Image img = LoadImage(l.Path(v["hdr"].GetString()).c_str());
        e.Add<Environment>(img);
        UnloadImage(img);
    });

    RegisterComponent("Sky", [](Loader &l, Entity e, const Value &v) {
        Image img = LoadImage(l.Path(v["hdr"].GetString()).c_str());
        e.Add<Sky>(img);
        UnloadImage(img);
    });

    RegisterComponent("Link", [](Loader &l, Entity e, const Value &v) {
        const Entity target = l.Resolve(v["target"].GetString());
        bool px, py, pz, rx, ry, rz, vx, vy, vz;
        GetBool3(v, "pos", px, py, pz);
        GetBool3(v, "rot", rx, ry, rz);
        GetBool3(v, "vel", vx, vy, vz);
        e.Add<Link>(target, px, py, pz, rx, ry, rz, vx, vy, vz);
    });

    RegisterComponent("Character", [](Loader &l, Entity e, const Value &v) {
        e.Add<Character>(l.engine().GetPhysics()->CreateCharacter(GetFloat(v, "height", 1.8f),
                                                                  GetFloat(v, "radius", 0.2f)));
    });

    RegisterComponent("Collider", [](Loader &l, Entity e, const Value &v) {
        auto physics = l.engine().GetPhysics();
        if (v.HasMember("sphere")) {
            e.Add<Collider>(physics->CreateSphereCollider(v["sphere"].GetFloat()));
        } else if (GetBool(v, "mesh", false)) {
            e.Add<Collider>(physics->CreateMeshCollider(e.Get<Mesh>(), GetBool(v, "dynamic", true)));
        }
    });

    RegisterComponent("Mesh", [](Loader &l, Entity e, const Value &v) {
        Mesh mesh = {};
        if (v.HasMember("generate")) {
            mesh = GenNamedMesh(v["generate"].GetString(), v);
            if (v.HasMember("offset")) {
                const Vector3 off = GetVec3(v, "offset", {0, 0, 0});
                for (int i = 0; i < mesh.vertexCount; i++) {
                    mesh.vertices[i * 3 + 0] += off.x;
                    mesh.vertices[i * 3 + 1] += off.y;
                    mesh.vertices[i * 3 + 2] += off.z;
                }
                rlUnloadVertexArray(mesh.vaoId);
                if (mesh.vboId) for (int i = 0; i < 9; i++) rlUnloadVertexBuffer(mesh.vboId[i]);
                mesh.vaoId = 0;
                UploadMesh(&mesh, false);
            }
        } else if (v.HasMember("model")) {
            Model m = LoadModel(l.Path(v["model"].GetString()).c_str());
            mesh = m.meshes[(int) GetFloat(v, "meshIndex", 0)];
        }
        if (mesh.tangents == nullptr) GenMeshTangents(&mesh);
        e.Add<Mesh>(mesh);
        if (!e.Has<BoundingBox>())
            e.Add<BoundingBox>(GetMeshBoundingBox(mesh));
    });

    RegisterComponent("Material", [](Loader &l, Entity e, const Value &v) {
        Material mat = LoadMaterialDefault();
        mat.maps[MATERIAL_MAP_ALBEDO].texture    = MapTexture(l, v, "albedo",    LIGHTGRAY);
        mat.maps[MATERIAL_MAP_NORMAL].texture    = MapTexture(l, v, "normal",    Color{128, 128, 255, 255});
        mat.maps[MATERIAL_MAP_METALNESS].texture = MapTexture(l, v, "metalness", WHITE);
        mat.maps[MATERIAL_MAP_ROUGHNESS].texture = MapTexture(l, v, "roughness", Color{217, 217, 217, 255});
        mat.maps[MATERIAL_MAP_EMISSION].texture  = MapTexture(l, v, "emission",  BLACK);
        mat.maps[MATERIAL_MAP_OCCLUSION].texture = MapTexture(l, v, "occlusion", WHITE);
        FinishMaterial(mat);
        e.Add<Material>(mat);
    });
}

void Loader::ExpandModel(const Value &spec) {
    Model model = LoadModel(Path(spec["model"].GetString()).c_str());

    for (int i = 0; i < model.materialCount; i++)
        FixModelMaterial(model.materials[i]);

    for (int i = 0; i < model.meshCount; i++) {
        Entity e = engine_.Create();

        Mesh mesh = model.meshes[i];
        // ponytail: GLTF-provided tangents are kept; only meshes lacking tangents
        // get them generated (the path main.cpp commented out for this model — watch
        // the map's normal mapping after this change).
        if (mesh.tangents == nullptr) GenMeshTangents(&mesh);

        if (spec.HasMember("Transform"))
            components_["Transform"](*this, e, spec["Transform"]);
        else
            e.Add<Transform>(Vector3{0, 0, 0}, Rotation(), Vector3{0, 0, 0});

        e.Add<Mesh>(mesh);
        e.Add<BoundingBox>(GetMeshBoundingBox(mesh));
        e.Add<Material>(model.materials[model.meshMaterial[i]]);

        // Apply the remaining components (e.g. Collider) to each sub-mesh entity.
        for (auto it = spec.MemberBegin(); it != spec.MemberEnd(); ++it) {
            const std::string key = it->name.GetString();
            if (key == "model" || key == "systems" || key == "Transform" ||
                key == "Mesh" || key == "Material")
                continue;
            auto f = components_.find(key);
            if (f != components_.end()) f->second(*this, e, it->value);
        }
    }
}
}
