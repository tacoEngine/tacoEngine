# Data-driven loader — design

Date: 2026-07-30
Status: approved (design), pending implementation

## Goal

Replace the game's hand-written imperative scene construction (`game/src/main.cpp`)
with a data-driven loader that lives **in the engine**. Components and systems are
registered with the loader by name, then instantiated and attached to entities from a
JSON config. Assets and configs are read from a single directory. The loader reproduces
`main.cpp`'s current behaviour for missing textures, tangent generation, and generated
meshes — all now configurable from the config file.

## Non-goals (YAGNI)

- Generic component reflection. A string→lambda registry with ~11 built-ins is enough.
- System JSON parameters in practice. The custom-factory overload exists for systems whose
  constructor takes arguments, but no current game system needs it.
- Config hot-reload, multiple simultaneous scenes, or JSON schema validation.

## Dependencies

- **rapidjson** — already a vendored submodule, linked at the top level but unused by the
  core lib. Add it as a dependency of the `tacoEngine` target (header-only interface
  target). No new third-party deps.

## New files

- `src/taco/Loader.h`, `src/taco/Loader.cpp` — the loader.

## Changed files

- `game/src/main.cpp` — imperative scene → register systems, point loader at asset dir,
  `LoadScene`, `Run`.
- `game/assets/scene.json` — the migrated scene (new asset).
- `CMakeLists.txt` (root) — add `Loader.{h,cpp}` to `TACO_SOURCES`; link rapidjson into
  `tacoEngine`.

## `taco::Loader` API

```cpp
namespace taco {
class Loader {
    Engine &engine_;
    std::string asset_dir_;
    std::map<std::string, ComponentLoader> components_;  // fn(Loader&, entity, const Value&)
    std::map<std::string, SystemFactory>   systems_;     // fn(const Value&) -> shared_ptr<System>
    std::map<std::string, entt::entity>    named_;       // populated during LoadScene

public:
    Loader(Engine &engine, std::string asset_dir);

    void RegisterComponent(const std::string &name, ComponentLoader fn);

    template<class T> void RegisterSystem(const std::string &name);        // default-constructible
    void RegisterSystem(const std::string &name, SystemFactory fn);        // ctor-with-params

    void LoadScene(const std::string &rel_path);

    // Helpers usable by built-in and custom component loaders:
    std::string  Path(const std::string &rel) const;                       // asset_dir_ / rel
    entt::entity Resolve(const std::string &name) const;                   // named_ lookup
    Engine      &engine() { return engine_; }
};
}
```

`ComponentLoader = std::function<void(Loader&, entt::entity, const rapidjson::Value&)>`.
`SystemFactory   = std::function<std::shared_ptr<System>(const rapidjson::Value&)>`.

The templated `RegisterSystem<T>(name)` registers `[](const Value&){ return std::make_shared<T>(); }`.
Systems whose constructor takes parameters use the non-template overload with a lambda that
reads the JSON node.

## Built-in components (pre-registered in the ctor)

`Transform, Camera, Mesh, Material, BoundingBox, Sunlight, Environment, Sky, Collider,
Character, Link`. The game registers only its **systems**.

## Scene JSON format

```json
{
  "entities": {
    "player": {
      "Transform": { "position": [5, 3.2, 5], "rotation": [0, 0, 0], "velocity": [0,0,0] },
      "Character": { "height": 1.8, "radius": 0.2 },
      "Mesh":      { "generate": "cube", "size": [0.2, 1.8, 0.2], "offset": [0, 0.9, 0] },
      "Material":  { "albedo": [255, 255, 255] },
      "Link":      { "target": "camera", "pos": [false,false,false] },
      "systems":   ["MovementSystem", "FrictionSystem", "RotationSystem"]
    },
    "map": {
      "model":    "IcedOut.glb",
      "Collider": { "mesh": true, "dynamic": false }
    },
    "camera": {
      "Transform": { "position": [5, 5, 5], "rotation": [-25, 90, 0] },
      "Camera":    { "fov": 72 },
      "Link":      { "target": "player", "pos": [true, true, true] },
      "systems":   ["LookSystem", "AimSystem", "ThirdPersonSystem"]
    },
    "sun":  { "Transform": { "rotation": [-35, -20, 0] }, "Sunlight": {} },
    "env":  { "Environment": { "hdr": "output2.hdr" } },
    "sky":  { "Sky": { "hdr": "output2.hdr" } }
  }
}
```

Reserved keys inside an entity spec: `model`, `systems`. Every other key is a registered
component name.

### Load mechanics

1. **Pass 1 — create + name.** For each entry in `entities`, create an entity and record
   `named_[name] = entity`. Model-expansion entries (see below) create a placeholder that is
   discarded / handled in pass 2; simplest is to defer model entries to pass 2 entirely and
   not name them (nothing references a model's sub-meshes).
2. **Pass 2 — apply.** For each entry, apply its components. Iterate components in a **fixed
   order**, not JSON key order (JSON objects are unordered): `Transform → Mesh → Material →
   BoundingBox → Collider → Character → <all remaining registered components>`. This lets
   `Collider` read the already-attached `Mesh`, and `Link` resolve names via `Resolve`.

### Model expansion

An entity spec containing `"model": "file.glb"` expands into **one entity per sub-mesh**,
reproducing today's `main.cpp` map loading:

- `LoadModel(Path(file))`; for each material apply the missing-texture fallback (see below),
  set `shader = GetGBufferShader()`, generate mipmaps + set trilinear/anisotropic filtering
  on albedo and normal.
- For each mesh: create an entity with `Transform` at origin (unless the spec overrides),
  `Mesh`, `Material` (the model's material for that mesh), `BoundingBox` from the mesh, and
  apply any other components listed in the spec (e.g. `Collider: {mesh:true, dynamic:false}`
  → a per-submesh mesh collider read from that entity's `Mesh`).

Model entries are not named and cannot be referenced by `Link`.

## Component loaders (behaviour)

- **Transform** — `position`/`velocity` default `[0,0,0]`; `rotation` in **degrees**
  (`[x,y,z]`), converted to euler radians for `taco::Rotation`.
- **Camera** — `fov` (float).
- **Sunlight** — optional `intensity`, `color` `[r,g,b]`, `shadow` (bool); defaults match
  `Sunlight`'s defaults.
- **Environment / Sky** — `hdr` image filename → `LoadImage(Path(...))`.
- **Link** — `target` (entity name → `Resolve`); nine optional per-axis bools grouped as
  `pos`/`rot`/`vel` arrays of 3, each defaulting to `[false,false,false]`.
- **Collider** — `{ "sphere": radius }` → `CreateSphereCollider`; `{ "mesh": true,
  "dynamic": bool }` → `CreateMeshCollider` from the entity's already-attached `Mesh`.
- **Character** — `height`, `radius` → `CreateCharacter`.
- **Mesh** — see below. **Material** — see below. **BoundingBox** — auto from the mesh if not
  explicitly present; explicit form not needed.

### Meshes (`Mesh` component)

- `"generate"`: one of the raylib `GenMesh*` primitives (`sphere`, `cube`, `plane`, …),
  with params pulled from JSON (`radius`, `rings`, `slices`, `size`, …).
- Optional `"offset": [x,y,z]`: translate the mesh's vertices (used for the character's
  visual cube), then unload the old VAO/VBOs and re-`UploadMesh`, matching `main.cpp`.
- Single mesh from a model: `{ "model": "file", "meshIndex": N }` (distinct from top-level
  `model` expansion).
- **GenMeshTangents** is called whenever `mesh.tangents == nullptr`: generated meshes always
  receive tangents; model meshes that already carry GLTF tangents are left untouched (this
  also sidesteps the wrong-tangent bug `main.cpp` commented out for the map).

### Materials (`Material` component) & missing-texture fallback

Start from `LoadMaterialDefault()`. Each PBR map field
(`albedo, normal, metalness, roughness, emission, occlusion`) may be:

- a `[r,g,b]` array → `LoadTextureFromImage(GenImageColor(1,1,color))`, or
- a `"file.png"` string → load from `Path(...)`, or
- **absent** → the `main.cpp` fallback color for that map (albedo LIGHTGRAY, normal
  `{128,128,255}`, metalness WHITE, roughness `{217}`, emission BLACK, occlusion WHITE).

Then generate mipmaps and set trilinear + anisotropic-16x filtering on albedo and normal,
and set `shader = GetGBufferShader()`. The same fallback routine is applied to materials
that come in via model loading.

## Migrated `main.cpp`

```cpp
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

Gravity stays in `main.cpp` (engine-level setup, not entity data). Everything else moves to
`scene.json`.

## Testing

One `assert`-based self-check (no framework): construct an `Engine` + `Loader`, load a tiny
in-memory / temp-file scene with a generate-cube entity plus a second entity carrying a
`Link` that targets the first by name, then assert the expected components landed
(`Mesh`/`BoundingBox`/`Transform` present, `Mesh.tangents != nullptr`, `Link.entity` resolves
to the named target). This exercises the two-pass name resolution, generated-mesh + tangent
path, and component application order.

## Risks / notes

- Loading textures/meshes requires an active GL context (raylib window). The engine ctor
  opens the window, so `Loader` must be used after `Engine` construction — as in the migrated
  `main.cpp`. The self-check therefore needs a headless-safe subset or a created window;
  if a window is unavailable in CI, guard the GPU-touching asserts.
- `-fno-rtti` is in force — no `dynamic_cast`; the registry uses `std::function`, which is
  fine without RTTI.
