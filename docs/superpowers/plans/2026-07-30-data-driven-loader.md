# Data-driven Loader Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the game's imperative scene construction with an engine-owned loader that instantiates registered components and systems from a JSON scene file.

**Architecture:** A new `taco::Loader` holds two string→callable registries (components, systems). Built-in components are pre-registered in its constructor; the game registers only its `System` subclasses. `LoadScene` parses a JSON file with rapidjson in two passes (create+name all entities, then apply components in a fixed order so name-resolution and mesh-dependent components work), reproducing the current missing-texture, tangent, generated-mesh, and model-expansion behaviour from `main.cpp`.

**Tech Stack:** C++20, EnTT, Jolt (via `taco::PhysicsEngine`), raylib + tacoRender, rapidjson (header-only, already vendored). CMake.

## Global Constraints

- C++20, `-fno-rtti` (no `dynamic_cast`; `std::function` is fine without RTTI).
- No new third-party dependencies. Use the vendored `ext/rapidjson`.
- `src` is a public include dir; engine headers are included as `#include "taco/..."`.
- MPL-2.0 file header on every new `.h`/`.cpp` (copy the block from any existing `src/taco/*.h`), author line `(c) Nikolas Wipper 2026`.
- The Engine constructor opens the window and loads shaders (embedded via `#embed`), so a `Loader` may only be used **after** an `Engine` exists. Loading textures/meshes/HDRs requires that live GL context.
- Rotations in the scene JSON are **degrees**; convert to euler radians with `DEG2RAD` before constructing `taco::Rotation`.
- Reuse `main.cpp`'s exact fallback colors and material finishing (mipmaps + trilinear + anisotropic-16x on albedo & normal, `shader = GetGBufferShader()`).

---

## File Structure

- `src/taco/Loader.h` — public API: `Loader`, the `ComponentLoader`/`SystemFactory` typedefs, templated `RegisterSystem<T>`. One responsibility: the loader interface.
- `src/taco/Loader.cpp` — implementation: JSON helpers, built-in component loaders, mesh/material helpers, model expansion, two-pass `LoadScene`, system attachment.
- `src/taco/loader_test.cpp` — standalone `assert`-based self-check executable.
- `game/assets/scene.json` — the migrated scene (new asset).
- `game/src/main.cpp` — rewritten to register systems + `LoadScene` + `Run`.
- `CMakeLists.txt` (root) — add Loader sources, rapidjson include dir, and the test executable.

---

## Task 1: Loader skeleton, registration API, CMake wiring

Establishes the header, an empty-behaviour `.cpp` that compiles and links, and the build wiring. No scene loading yet.

**Files:**
- Create: `src/taco/Loader.h`
- Create: `src/taco/Loader.cpp`
- Modify: `CMakeLists.txt` (root) — `TACO_SOURCES` list and include dirs

**Interfaces:**
- Produces:
  - `taco::Loader::Value` = `rapidjson::Value`
  - `taco::Loader::ComponentLoader` = `std::function<void(Loader&, entt::entity, const Value&)>`
  - `taco::Loader::SystemFactory` = `std::function<std::shared_ptr<taco::System>(const Value&)>`
  - `Loader(Engine &engine, std::string asset_dir)`
  - `void RegisterComponent(const std::string &name, ComponentLoader fn)`
  - `void RegisterSystem(const std::string &name, SystemFactory fn)`
  - `template<class T> void RegisterSystem(const std::string &name)`
  - `void LoadScene(const std::string &rel_path)`
  - `Engine &engine()`
  - `std::string Path(const std::string &rel) const`
  - `entt::entity Resolve(const std::string &name) const`

- [ ] **Step 1: Write the header `src/taco/Loader.h`**

```cpp
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
```

- [ ] **Step 2: Write a minimal `src/taco/Loader.cpp`**

```cpp
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
```

- [ ] **Step 3: Wire CMake**

In root `CMakeLists.txt`, add to the `TACO_SOURCES` list (before the closing `)`):

```cmake
        src/taco/Loader.cpp
        src/taco/Loader.h
```

After the `add_library(tacoEngine ${TACO_SOURCES})` / `target_link_libraries` lines, add rapidjson's headers to the engine target:

```cmake
target_include_directories(tacoEngine PRIVATE ext/rapidjson/include)
```

- [ ] **Step 4: Build to verify it compiles and links**

Run: `cmake --build game/cmake-build-debug --target tacoEngine`
Expected: builds with no errors (the game still builds against the old `main.cpp`).

- [ ] **Step 5: Commit**

```bash
git add src/taco/Loader.h src/taco/Loader.cpp CMakeLists.txt
git commit -m "Add taco::Loader skeleton and registration API"
```

---

## Task 2: JSON helpers, two-pass LoadScene, system attachment

Fills in `LoadScene` (parse + two passes), `ApplyComponents` (fixed order), and `AttachSystems`. Built-in component loaders are still empty, so nothing is attached yet, but the traversal and name map work.

**Files:**
- Modify: `src/taco/Loader.cpp`

**Interfaces:**
- Consumes: `Engine::registry` (`entt::registry`), the `components_`/`systems_`/`named_` maps from Task 1.
- Produces: internal file-local helpers `GetVec3`, `GetFloat`, `GetBool`, `GetColor`, `GetBool3`; working `LoadScene`, `ApplyComponents`, `AttachSystems`.

- [ ] **Step 1: Add includes and file-local JSON helpers at the top of `Loader.cpp`**

Add below the existing includes:

```cpp
#include <fstream>
#include <set>
#include <sstream>

#include <raylib.h>
#include <raymath.h>
```

Add an anonymous namespace with helpers (place after the includes, before `namespace taco {`):

```cpp
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
```

- [ ] **Step 2: Implement `LoadScene`, `ApplyComponents`, `AttachSystems`**

Replace the empty `LoadScene`, `ApplyComponents`, and `AttachSystems` stubs with:

```cpp
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
```

Also add `#include <cassert>` and `#include <entt/core/hashed_string.hpp>` to the includes.

- [ ] **Step 3: Build to verify it compiles**

Run: `cmake --build game/cmake-build-debug --target tacoEngine`
Expected: builds with no errors.

- [ ] **Step 4: Commit**

```bash
git add src/taco/Loader.cpp
git commit -m "Implement two-pass LoadScene and system attachment"
```

---

## Task 3: Built-in component loaders (non-mesh)

Registers the simple components: `Transform`, `Camera`, `Sunlight`, `Environment`, `Sky`, `Link`, `Character`, `Collider`.

**Files:**
- Modify: `src/taco/Loader.cpp` — `RegisterBuiltins`

**Interfaces:**
- Consumes: `Engine::registry`, `Engine::GetPhysics()` → `std::shared_ptr<PhysicsEngine>`; `PhysicsEngine::CreateSphereCollider(double)`, `CreateMeshCollider(Mesh, bool)`, `CreateCharacter(double,double)`; component types `Transform`, `Camera`, `Sunlight`, `Environment`, `Sky`, `Link`, `Collider`, `Character`; `Rotation(float,float,float)`.
- Produces: registered loaders under the names above (except Mesh/Material, added in Task 4).

- [ ] **Step 1: Add component includes to `Loader.cpp`**

```cpp
#include "comp/Camera.h"
#include "comp/Lights.h"
#include "comp/Transform.h"
```

(`Physics.h` and `raylib.h` come in via `Engine.h` / Task 2.)

- [ ] **Step 2: Implement `RegisterBuiltins` for the non-mesh components**

Replace the empty `RegisterBuiltins` with:

```cpp
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
```

- [ ] **Step 3: Build to verify it compiles**

Run: `cmake --build game/cmake-build-debug --target tacoEngine`
Expected: builds with no errors.

- [ ] **Step 4: Commit**

```bash
git add src/taco/Loader.cpp
git commit -m "Add built-in non-mesh component loaders"
```

---

## Task 4: Mesh & Material loaders with tangents and texture fallback

Adds generated/loaded meshes (with automatic tangent generation and optional vertex offset), auto `BoundingBox`, and materials with the `main.cpp` missing-texture fallbacks.

**Files:**
- Modify: `src/taco/Loader.cpp`

**Interfaces:**
- Consumes: raylib `GenMeshSphere/Cube/Plane`, `GenMeshTangents`, `GetMeshBoundingBox`, `UploadMesh`, `LoadMaterialDefault`, `LoadTexture`, `LoadTextureFromImage`, `GenImageColor`, `GenTextureMipmaps`, `SetTextureFilter`, `rlUnloadVertexArray`, `rlUnloadVertexBuffer` (`rlgl.h`); tacoRender `GetGBufferShader`.
- Produces: file-local helpers `GenNamedMesh`, `SolidTexture`, `MapTexture`, `FinishMaterial`; registered `"Mesh"` and `"Material"` loaders. `FinishMaterial` is reused by Task 5.

- [ ] **Step 1: Add `#include "rlgl.h"` to `Loader.cpp`**

- [ ] **Step 2: Add mesh/material file-local helpers inside the anonymous namespace**

Append to the existing anonymous namespace from Task 2:

```cpp
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
```

`MapTexture` needs the asset path, so it takes the `Loader`. Add it too (still inside the anonymous namespace — forward-declare `Loader::Path` via the header, which is already included):

```cpp
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
```

- [ ] **Step 3: Register the `Mesh` and `Material` loaders**

Insert these into `RegisterBuiltins` (replacing the `// Mesh and Material are registered in Task 4.` comment):

```cpp
    RegisterComponent("Mesh", [](Loader &l, entt::entity e, const Value &v) {
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
        l.engine().registry.emplace<Mesh>(e, mesh);
        if (!l.engine().registry.all_of<BoundingBox>(e))
            l.engine().registry.emplace<BoundingBox>(e, GetMeshBoundingBox(mesh));
    });

    RegisterComponent("Material", [](Loader &l, entt::entity e, const Value &v) {
        Material mat = LoadMaterialDefault();
        mat.maps[MATERIAL_MAP_ALBEDO].texture    = MapTexture(l, v, "albedo",    LIGHTGRAY);
        mat.maps[MATERIAL_MAP_NORMAL].texture    = MapTexture(l, v, "normal",    Color{128, 128, 255, 255});
        mat.maps[MATERIAL_MAP_METALNESS].texture = MapTexture(l, v, "metalness", WHITE);
        mat.maps[MATERIAL_MAP_ROUGHNESS].texture = MapTexture(l, v, "roughness", Color{217, 217, 217, 255});
        mat.maps[MATERIAL_MAP_EMISSION].texture  = MapTexture(l, v, "emission",  BLACK);
        mat.maps[MATERIAL_MAP_OCCLUSION].texture = MapTexture(l, v, "occlusion", WHITE);
        FinishMaterial(mat);
        l.engine().registry.emplace<Material>(e, mat);
    });
```

- [ ] **Step 4: Build to verify it compiles**

Run: `cmake --build game/cmake-build-debug --target tacoEngine`
Expected: builds with no errors.

- [ ] **Step 5: Commit**

```bash
git add src/taco/Loader.cpp
git commit -m "Add Mesh and Material loaders with tangents and texture fallback"
```

---

## Task 5: Model expansion

Implements `ExpandModel`: a `"model"` entity produces one entity per sub-mesh, carrying the model's (fallback-fixed) materials and any additional components in the spec, matching the current map loading in `main.cpp`.

**Files:**
- Modify: `src/taco/Loader.cpp` — `ExpandModel` and a `FixModelMaterial` helper

**Interfaces:**
- Consumes: raylib `LoadModel`, `Model` (`meshes`, `materials`, `materialCount`, `meshCount`, `meshMaterial`), `rlGetTextureIdDefault`; the `FinishMaterial`/`SolidTexture` helpers from Task 4; the `Transform`/`Collider` etc. loaders in `components_`.
- Produces: working `ExpandModel`.

- [ ] **Step 1: Add `FixModelMaterial` to the anonymous namespace**

Mirrors the model material fixup in `main.cpp` (default/id-0 textures → fallback colors; note albedo uses a 4×4 image as in the original), then reuses `FinishMaterial`:

```cpp
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
```

- [ ] **Step 2: Implement `ExpandModel`**

Replace the empty `ExpandModel` stub with:

```cpp
void Loader::ExpandModel(const Value &spec) {
    Model model = LoadModel(Path(spec["model"].GetString()).c_str());

    for (int i = 0; i < model.materialCount; i++)
        FixModelMaterial(model.materials[i]);

    for (int i = 0; i < model.meshCount; i++) {
        const entt::entity e = engine_.registry.create();

        Mesh mesh = model.meshes[i];
        // ponytail: GLTF-provided tangents are kept; only meshes lacking tangents
        // get them generated (the path main.cpp commented out for this model — watch
        // the map's normal mapping after this change).
        if (mesh.tangents == nullptr) GenMeshTangents(&mesh);

        if (spec.HasMember("Transform"))
            components_["Transform"](*this, e, spec["Transform"]);
        else
            engine_.registry.emplace<Transform>(e, Vector3{0, 0, 0}, Rotation(), Vector3{0, 0, 0});

        engine_.registry.emplace<Mesh>(e, mesh);
        engine_.registry.emplace<BoundingBox>(e, GetMeshBoundingBox(mesh));
        engine_.registry.emplace<Material>(e, model.materials[model.meshMaterial[i]]);

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
```

- [ ] **Step 3: Build to verify it compiles**

Run: `cmake --build game/cmake-build-debug --target tacoEngine`
Expected: builds with no errors.

- [ ] **Step 4: Commit**

```bash
git add src/taco/Loader.cpp
git commit -m "Add model expansion to loader"
```

---

## Task 6: Self-check test executable

A standalone `assert`-based program that loads a tiny generated scene and verifies the two-pass name resolution, generated-mesh + tangent path, and component application. It opens a window (Engine ctor) and exits.

**Files:**
- Create: `src/taco/loader_test.cpp`
- Modify: `CMakeLists.txt` (root) — add the `tacoLoaderTest` executable

**Interfaces:**
- Consumes: `taco::Engine`, `taco::Loader`, `taco::Transform`, `taco::Link`; raylib `Mesh`, `BoundingBox`.

- [ ] **Step 1: Write `src/taco/loader_test.cpp`**

```cpp
// tacoEngine (c) Nikolas Wipper 2026

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include <cassert>
#include <cstdio>
#include <fstream>

#include "taco/Engine.h"
#include "taco/Loader.h"
#include "taco/comp/Transform.h"

int main() {
    const char *scene =
        "{ \"entities\": {"
        "  \"a\": { \"Transform\": {\"position\":[1,2,3]},"
        "           \"Mesh\": {\"generate\":\"cube\",\"size\":[1,1,1]} },"
        "  \"b\": { \"Transform\": {},"
        "           \"Link\": {\"target\":\"a\",\"pos\":[true,false,true]} }"
        "} }";
    { std::ofstream out("loader_test_scene.json"); out << scene; }

    taco::Engine engine;
    taco::Loader loader(engine, ".");
    loader.LoadScene("loader_test_scene.json");

    // Entity 'a': the one carrying a Mesh, positioned at (1,2,3).
    bool found_a = false;
    for (const entt::entity e : engine.registry.view<Mesh, taco::Transform>()) {
        const taco::Transform &t = engine.registry.get<taco::Transform>(e);
        if (t.position.x == 1 && t.position.y == 2 && t.position.z == 3) {
            found_a = true;
            assert(engine.registry.all_of<BoundingBox>(e));
            assert(engine.registry.get<Mesh>(e).tangents != nullptr);
        }
    }
    assert(found_a);

    // Entity 'b': its Link must resolve to 'a' with the right per-axis flags.
    bool found_b = false;
    for (const entt::entity e : engine.registry.view<taco::Link>()) {
        found_b = true;
        const taco::Link &link = engine.registry.get<taco::Link>(e);
        const taco::Transform &t = engine.registry.get<taco::Transform>(link.entity);
        assert(t.position.x == 1 && t.position.y == 2 && t.position.z == 3);
        assert(link.linkPosX && !link.linkPosY && link.linkPosZ);
    }
    assert(found_b);

    std::printf("loader self-check passed\n");
    return 0;
}
```

- [ ] **Step 2: Add the test executable to root `CMakeLists.txt`**

After the `tacoEngine` target block, add:

```cmake
add_executable(tacoLoaderTest src/taco/loader_test.cpp)
target_link_libraries(tacoLoaderTest tacoEngine)
target_include_directories(tacoLoaderTest PRIVATE ext/rapidjson/include)
```

- [ ] **Step 3: Build the test**

Run: `cmake --build game/cmake-build-debug --target tacoLoaderTest`
Expected: builds with no errors.

- [ ] **Step 4: Run the test**

Run: `game/cmake-build-debug/tacoLoaderTest`
Expected: a window flashes, process exits 0, prints `loader self-check passed`. (If any `assert` fires, the process aborts non-zero — that is a real failure to fix.)

- [ ] **Step 5: Commit**

```bash
git add src/taco/loader_test.cpp CMakeLists.txt
git commit -m "Add loader self-check test"
```

---

## Task 7: Migrate the game to the config-driven loader

Rewrite `main.cpp` to register systems and load `scene.json`, and add the scene file reproducing the current scene.

**Files:**
- Create: `game/assets/scene.json`
- Modify: `game/src/main.cpp`

**Interfaces:**
- Consumes: `taco::Loader`, `Loader::RegisterSystem<T>`, `Loader::LoadScene`; the game's system classes (`LookSystem`, `AimSystem`, `ThirdPersonSystem`, `MovementSystem`, `FrictionSystem`, `RotationSystem`).

- [ ] **Step 1: Write `game/assets/scene.json`**

```json
{
  "entities": {
    "ball": {
      "Transform": { "position": [0, 5, 0] },
      "Collider": { "sphere": 1.0 },
      "Mesh": { "generate": "sphere", "radius": 1.0, "rings": 20, "slices": 20 },
      "Material": {
        "albedo": [0, 0, 255],
        "metalness": [0, 0, 0],
        "roughness": [128, 128, 128]
      }
    },
    "map": {
      "model": "IcedOut.glb",
      "Collider": { "mesh": true, "dynamic": false }
    },
    "camera": {
      "Transform": { "position": [5, 5, 5], "rotation": [-25, 90, 0] },
      "Camera": { "fov": 72 },
      "Link": { "target": "character", "pos": [true, true, true] },
      "systems": ["LookSystem", "AimSystem", "ThirdPersonSystem"]
    },
    "character": {
      "Transform": { "position": [5, 3.2, 5] },
      "Character": { "height": 1.8, "radius": 0.2 },
      "Mesh": { "generate": "cube", "size": [0.2, 1.8, 0.2], "offset": [0, 0.9, 0] },
      "Material": {
        "albedo": [255, 255, 255],
        "metalness": [0, 0, 0],
        "roughness": [128, 128, 128]
      },
      "Link": { "target": "camera" },
      "systems": ["MovementSystem", "FrictionSystem", "RotationSystem"]
    },
    "sun": {
      "Transform": { "rotation": [-35, -20, 0] },
      "Sunlight": {}
    },
    "env": { "Environment": { "hdr": "output2.hdr" } },
    "sky": { "Sky": { "hdr": "output2.hdr" } }
  }
}
```

- [ ] **Step 2: Rewrite `game/src/main.cpp`**

```cpp
// (c) Nikolas Wipper 2024-2026

#include <log/log.h>
#include <taco/Engine.h>
#include <taco/Graphics.h>
#include <taco/Loader.h>

#include "AimSystem.h"
#include "FrictionSystem.h"
#include "LookSystem.h"
#include "MovementSystem.h"
#include "RotationSystem.h"
#include "ThirdPersonSystem.h"

int main() {
    logging::Logger::SetLoggingType(logging::DEBUG);

    taco::Engine engine;
    engine.GetPhysics()->SetGravity(Vector3(0, -15.76f, 0));

    taco::Loader loader(engine, "assets");
    loader.RegisterSystem<LookSystem>("LookSystem");
    loader.RegisterSystem<AimSystem>("AimSystem");
    loader.RegisterSystem<ThirdPersonSystem>("ThirdPersonSystem");
    loader.RegisterSystem<MovementSystem>("MovementSystem");
    loader.RegisterSystem<FrictionSystem>("FrictionSystem");
    loader.RegisterSystem<RotationSystem>("RotationSystem");

    loader.LoadScene("scene.json");

    DisableCursor();
    engine.Run();
}
```

- [ ] **Step 3: Ensure `scene.json` ships next to the other assets**

The existing assets already load from `assets/` at runtime, so `scene.json` living in `game/assets/` matches. If the build copies `assets/` into the macOS bundle, confirm `scene.json` is picked up the same way as `output2.hdr` / `IcedOut.glb` (same directory, no CMake change expected). If assets are referenced in-place from the source tree, no action needed.

- [ ] **Step 4: Build the game**

Run: `cmake --build game/cmake-build-debug --target game`
Expected: builds with no errors.

- [ ] **Step 5: Run the game and verify the scene matches the old behaviour**

Run the game (via the built `game.app` / `game` binary as usual for this project).
Expected: same scene as before — blue sphere at (0,5,0), the IcedOut map with collision, a controllable third-person character (WASD + space + shift), sun + sky + IBL lighting. Movement, look, aim, friction, third-person, and rotation all work.

- [ ] **Step 6: Commit**

```bash
git add game/src/main.cpp game/assets/scene.json
git commit -m "Migrate game to data-driven scene loader"
```

---

## Self-Review

**Spec coverage:**
- Data-driven loader in the engine → Tasks 1–5 (`src/taco/Loader.*`). ✓
- Components + systems registered, applied from config → Task 1 (API), Task 2 (dispatch), Task 3–4 (built-ins), `main.cpp` registers systems (Task 7). ✓
- Assets + configs from a single directory → `Loader(engine, "assets")` + `Path()` (Tasks 1, 7). ✓
- Missing-texture handling like `main.cpp` → Task 4 (`MapTexture` fallback) + Task 5 (`FixModelMaterial`). ✓
- `GenMeshTangents` → Task 4 (generated + single model mesh) + Task 5 (model expansion), guarded on `tangents == nullptr`. ✓
- Generated meshes configurable → Task 4 (`GenNamedMesh` sphere/cube/plane + params + offset). ✓
- Model expansion preserving current map loading → Task 5. ✓
- Self-check test → Task 6. ✓

**Placeholder scan:** No TBD/TODO; every code step has full content. The one `ponytail:` comment in Task 5 is a deliberate, named ceiling (kept-vs-generated tangents), not a placeholder.

**Type consistency:** `ComponentLoader`/`SystemFactory`/`Value` typedefs used identically across tasks; `Path`, `Resolve`, `engine()`, `RegisterComponent`, `RegisterSystem`, `LoadScene`, `ApplyComponents`, `AttachSystems`, `ExpandModel` names match between header (Task 1) and definitions (Tasks 2–5). `FinishMaterial`/`SolidTexture` defined in Task 4 and reused in Task 5. Scene JSON keys in Task 7 match the loaders in Tasks 3–4 (`Transform`, `Camera`, `Sunlight`, `Environment`, `Sky`, `Link`, `Character`, `Collider`, `Mesh`, `Material`, `systems`, `model`).

**Open risk (carried from spec):** auto-tangent generation on the `IcedOut.glb` sub-meshes only triggers if the GLTF lacks tangents — the very path `main.cpp` disabled. Verified visually in Task 7 Step 5; if the map's normal mapping looks wrong, revert to skipping tangent generation for model meshes (drop the `GenMeshTangents` call in `ExpandModel`).
