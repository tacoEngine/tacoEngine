// tacoEngine (c) Nikolas Wipper 2024-2025

/*
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 */

#include "Engine.h"

#include <vector>
#include <memory>
#include <ratio>

#include <log/log.h>
#include <raylib.h>
#include <raymath.h>
#include <tr_effects.h>

#include "rlgl.h"
#include "tr_shaders.h"
#include "tr_timing.h"
#include "comp/Camera.h"
#include "comp/Lights.h"
#include "misc/Debug.h"

namespace taco {
Engine::Engine() {
#if defined(NDEBUG)
    ChangeDirectory(GetApplicationDirectory());
#endif

    SetConfigFlags(FLAG_WINDOW_HIGHDPI | FLAG_WINDOW_UNDECORATED | FLAG_WINDOW_MAXIMIZED);
    InitWindow(0, 0, "taco");
    Init3D();

    SetExitKey(0);

    physics_ = std::make_unique<PhysicsEngine>();
    debug_renderer_ = std::make_unique<RaylibDebugRenderer>();

    ReloadGBuffers();

    registry_.on_destroy<Collider>().connect<&Engine::DestroyColliderBody>(this);
    registry_.on_destroy<Character>().connect<&Engine::DestroyCharacterBody>(this);

    // Sunlight's shadowMap has heap arrays that cannot be trivially deep-copied
    TrackSunlight();

    // entt::entity is the entity list itself; the two physics types ride Jolt's state stream.
    Ignore<entt::entity>();
    Ignore<Collider>();
    Ignore<Character>();
}

Engine::~Engine() {
    registry_.clear();
    ClearRetired();
}

void Engine::DestroyColliderBody(entt::registry &reg, const entt::entity entity) {
    const Collider &collider = reg.get<Collider>(entity);
    physics_->body_interface_.RemoveBody(collider.body_id_);

    // Don't destroy the collider when there's a checkpoint, so it can be restored
    if (checkpointed_)
        retired_colliders_.emplace(entity, collider);
    else
        physics_->body_interface_.DestroyBody(collider.body_id_);
}

void Engine::DestroyCharacterBody(entt::registry &reg, const entt::entity entity) {
    const Character &character = reg.get<Character>(entity);
    character.character_->RemoveFromPhysicsSystem();

    // Don't destroy the character when there's a checkpoint, so it can be restored
    if (checkpointed_)
        retired_characters_.emplace(entity, character);
}

void Engine::TrackSunlight() {
    struct TrackedSunlight {
        float intensity;
        Color color;
        bool shadow_casting;
    };

    tracked_[entt::type_id<Sunlight>().hash()] =
            [](const entt::registry &registry) -> std::function<void(entt::registry &)> {
                std::map<entt::entity, TrackedSunlight> data;
                for (auto [entity, sun] : registry.view<const Sunlight>().each())
                    data.emplace(entity,
                                 TrackedSunlight {
                                     .intensity = sun.intensity,
                                     .color = sun.color,
                                     .shadow_casting = sun.shadow_casting
                                 });

                return [data = std::move(data)](entt::registry &registry) {
                    for (const auto &[entity, values] : data) {
                        if (!registry.valid(entity)) continue;
                        // Patched in place, never replaced: shadow_map_ is a live GPU allocation,
                        // not state, so it outlives every restore. A sun revived with the entity
                        // starts at {0} and Render allocates one, exactly like a fresh Add.
                        Sunlight &sun = registry.get_or_emplace<Sunlight>(entity);
                        sun.intensity = values.intensity;
                        sun.color = values.color;
                        sun.shadow_casting = values.shadow_casting;
                    }
                };
            };
}

// A Collider is a bare id and needs DestroyBody; a Character frees its body when the last Ref
// to it goes, which is the clear() below.
void Engine::ClearRetired() {
    for (auto &[entity, collider] : retired_colliders_)
        physics_->body_interface_.DestroyBody(collider.body_id_);
    retired_colliders_.clear();

    retired_characters_.clear();
}

// Between RestoreECS and RestorePhysics: a revived body has to be in the broad phase before the
// stream replays, a leftover has to be gone.
void Engine::HandBackRetired(const Checkpoint &cp) {
    // Attached since the capture, so the stream would leave it running. Remove parks it via
    // on_destroy, the loops below destroy it. Collected first — removing mid-iteration is UB.
    std::vector<entt::entity> added;
    for (const entt::entity entity : registry_.view<Collider>())
        if (!cp.colliders_.contains(entity))
            added.push_back(entity);
    for (const entt::entity entity : added)
        registry_.remove<Collider>(entity);

    added.clear();
    for (const entt::entity entity : registry_.view<Character>())
        if (!cp.characters_.contains(entity))
            added.push_back(entity);
    for (const entt::entity entity : added)
        registry_.remove<Character>(entity);

    for (auto &[entity, collider] : retired_colliders_) {
        if (cp.colliders_.count(entity) && registry_.valid(entity) && !registry_.all_of<Collider>(entity)) {
            // RestoreState sets whether the body is awake, so it goes back in deactivated.
            physics_->body_interface_.AddBody(collider.body_id_, JPH::EActivation::DontActivate);
            registry_.emplace<Collider>(entity, collider);
        } else {
            physics_->body_interface_.DestroyBody(collider.body_id_);
        }
    }
    retired_colliders_.clear();

    // No else branch: a leftover is destroyed by the clear() below dropping the last Ref.
    for (auto &[entity, character] : retired_characters_) {
        if (cp.characters_.contains(entity) && registry_.valid(entity) && !registry_.all_of<Character>(entity)) {
            character.character_->AddToPhysicsSystem(JPH::EActivation::DontActivate);
            registry_.emplace<Character>(entity, character);
        }
    }
    retired_characters_.clear();
}

Checkpoint Engine::Save() {
    checkpointed_ = true;

    Checkpoint cp;
    for (auto &[_, capture] : tracked_)
        cp.restore_.push_back(capture(registry_));
    cp.CaptureECS(registry_);
    cp.CapturePhysics(*physics_, registry_);

    return cp;
}

void Engine::Restore(Checkpoint &cp) {
    cp.RestoreECS(registry_);
    HandBackRetired(cp);
    cp.RestorePhysics(*physics_, registry_);
}

void Engine::RequestRestore(Checkpoint &cp) {
    pending_restore_ = &cp;
}

void Engine::ApplyPendingRestore() {
    if (Checkpoint *pending = pending_restore_) {
        pending_restore_ = nullptr;
        Restore(*pending);
    }
}

void Engine::Run() {
    running_ = true;

    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point last_frame = start;

    while (running_) {
        Render();

        auto now = std::chrono::steady_clock::now();

        std::chrono::duration<int64_t, std::nano> frame_delta = now - last_frame;
        delta_time_ = frame_delta.count();
        last_frame = now;

        // Render() polls the window, so this is the freshest input state.
        std::chrono::duration<int64_t, std::nano> elapsed = now - start;
        input_.SetTime(elapsed.count());
        input_.UpdateFromLocalInput();

        Update();

        ApplyPendingRestore();
    }
}

void Engine::Update() {
    DispatchSystems(&SystemHooks::early);
    DispatchSystems(&SystemHooks::pre_physics);

    auto collider_view = registry_.view<Collider, Transform>();
    auto character_view = registry_.view<Character, Transform>();
    auto link_view = registry_.view<Link, Transform>();

    for (auto [_, collider, transform] : collider_view.each()) {
        collider.SetPosition(transform.position);
        collider.SetRotation(transform.rotation.GetQuaternion());
        collider.SetVelocity(transform.velocity);
    }

    for (auto [_, character, transform] : character_view.each()) {
        character.SetPosition(transform.position);
        character.SetRotation(transform.rotation.GetQuaternion());
        character.SetVelocity(transform.velocity);
    }

    physics_->Update(GetDeltaTime());

    for (auto [_, collider, transform] : collider_view.each()) {
        transform.position = collider.GetPosition();
        transform.rotation.SetFromQuaternion(collider.GetRotation());
        transform.velocity = collider.GetVelocity();
    }

    for (auto [_, character, transform] : character_view.each()) {
        transform.position = character.GetPosition();
        transform.rotation.SetFromQuaternion(character.GetRotation());
        transform.velocity = character.GetVelocity();

        character.character_->PostSimulation(0.01f);
    }

    for (auto [_, link, transform] : link_view.each()) {
        auto &remote_transform = link.target.Get<Transform>();

        if (link.linkPosX)
            transform.position.x = remote_transform.position.x;
        if (link.linkPosY)
            transform.position.y = remote_transform.position.y;
        if (link.linkPosZ)
            transform.position.z = remote_transform.position.z;

        if (link.linkRotX)
            transform.rotation.x = remote_transform.rotation.x;
        if (link.linkRotY)
            transform.rotation.y = remote_transform.rotation.y;
        if (link.linkRotZ)
            transform.rotation.z = remote_transform.rotation.z;

        if (link.linkVelX)
            transform.velocity.x = remote_transform.velocity.x;
        if (link.linkVelY)
            transform.velocity.y = remote_transform.velocity.y;
        if (link.linkVelZ)
            transform.velocity.z = remote_transform.velocity.z;
    }

    DispatchSystems(&SystemHooks::post_physics);
    DispatchSystems(&SystemHooks::late);
}

void Engine::DispatchSystems(void (*SystemHooks::*phase)(entt::registry &, Engine *)) {
    for (auto &system_hook : system_hooks_)
        (system_hook.*phase)(registry_, this);
}

void Engine::RunSystemPhasesForTest() {
    DispatchSystems(&SystemHooks::early);
    DispatchSystems(&SystemHooks::pre_physics);
    DispatchSystems(&SystemHooks::post_physics);
    DispatchSystems(&SystemHooks::late);
    DispatchSystems(&SystemHooks::ui);
}

void Engine::Render() {
    static Timer timers[7];
    std::array<float, 7> timings = {0.0f};

    if (!timers[0].query) {
        for (auto &timer : timers) {
            InitTimer(&timer);
        }
    }

    for (int i = 0; i < 6; i++) {
        timings[i] = timers[i].Get();
    }

    if (IsWindowResized())
        ReloadGBuffers();

    auto camera_view = registry_.view<const Transform, const Camera>();
    auto model_view = registry_.view<const Transform, const Mesh, Material>();
    auto env_view = registry_.view<const Environment>();
    auto sky_view = registry_.view<const Sky>();

    timers[0].Start();

    BeginGBufferMode(gbuffers_);
    ClearBackground(BLACK);

    Camera3D raylib_camera = {};
    size_t drawn_meshes;

    for (auto [_, transform, cam] : camera_view.each()) {
        Vector3 camera_target = transform.position + transform.rotation.GetDirection();
        raylib_camera = {transform.position, camera_target, {0, 1, 0}, cam.fov, CAMERA_PERSPECTIVE};

        Frustum frustum = CreateFrustumFromCamera(raylib_camera,
                                                  (float) GetRenderWidth() / (float) GetRenderHeight(),
                                                  cam.fov,
                                                  RL_CULL_DISTANCE_NEAR,
                                                  RL_CULL_DISTANCE_FAR);

        BeginMode3D(raylib_camera);

        drawn_meshes = DrawAllMeshes(model_view, frustum, GetGBufferShader());

        if (config_.debug_physics)
            physics_->Render();

        for (auto [_, sky] : sky_view.each()) {
            DrawSkybox(sky.skybox_, WHITE);
        }

        EndMode3D();
    }

    EndGBufferMode();

    timers[0].Stop();

    auto sun_view = registry_.view<const Transform, Sunlight>();

    for (auto [_, transform, sun] : sun_view.each()) {
        if (!sun.shadow_casting) {
            if (sun.shadow_map_.fbo) {
                UnloadShadowMap(sun.shadow_map_);
                sun.shadow_map_.fbo = 0;
            }
            continue;
        }
        const int cascadeCount = 3;
        if (sun.shadow_map_.size != config_.shadow_map_size) {
            UnloadShadowMap(sun.shadow_map_);
            sun.shadow_map_ = LoadShadowMap(config_.shadow_map_size, cascadeCount, config_.cascade_dist);
        }

        timers[1].Start();

        for (int i = 0; i < cascadeCount; i++) {
            Vector3 direction = Vector3RotateByQuaternion(Vector3 {0, 0, -1}, transform.rotation.GetQuaternion());
            BeginShadowMap(sun.shadow_map_, raylib_camera, direction, i);

            ClearBackground(BLANK);

            // 0 frustum draws all meshes
            // fixme: create frustum from sun.shadow_map_.projections[i]
            DrawAllMeshes(model_view, {});

            EndShadowMap();
        }

        timers[1].Stop();
        timers[2].Start();

        FilterShadowMap(sun.shadow_map_, config_.shadow_map_quality, 1);

        timers[2].Stop();
    }

    timers[3].Start();

    if (config_.ssao)
        ApplySSAO(presenter_, raylib_camera);

    timers[3].Stop();
    timers[4].Start();

    ClearPresenter(presenter_);

    BeginLightingPass(presenter_);

    for (auto [_, transform, sun] : sun_view.each()) {
        LightSun(presenter_,
                 raylib_camera,
                 Vector3RotateByQuaternion(Vector3 {0, 0, -1}, transform.rotation.GetQuaternion()),
                 sun.intensity,
                 sun.color,
                 sun.shadow_casting ? sun.shadow_map_ : NULL_SHADOW_MAP);
    }

    for (auto [_, env] : env_view.each()) {
        // TODO: At least select the closest IBL to the camera
        LightIBL(presenter_, raylib_camera, env.radiance_, env.irradiance_);
    }

    CopyBackground(presenter_);

    EndLightingPass();

    timers[4].Stop();
    timers[5].Start();

    ApplyToneMapping(presenter_, config_.tone_mapper);
    ApplyGammaCorrection(presenter_, config_.gamma_correction);

    timers[5].Stop();
    timers[6].Start();

    BeginDrawing();

    ClearBackground(BLANK);

    DrawTexture(presenter_.back[0].texture, 0, 0, WHITE);
    rlEnableColorBlend();
    DrawFPS(0, 0);

    DrawText("Geometry", 0, 150 + 12 * 0, 12, WHITE);
    DrawText("Shadow", 0, 150 + 12 * 1, 12, WHITE);
    DrawText("Shadow PP", 0, 150 + 12 * 2, 12, WHITE);
    DrawText("SSAO", 0, 150 + 12 * 3, 12, WHITE);
    DrawText("Lighting", 0, 150 + 12 * 4, 12, WHITE);
    DrawText("PP", 0, 150 + 12 * 5, 12, WHITE);
    DrawText("Blit", 0, 150 + 12 * 6, 12, WHITE);
    DrawText("Total", 0, 150 + 12 * 7, 12, WHITE);
    DrawText("FPS", 0, 150 + 12 * 8, 12, WHITE);
    DrawText("Unacc", 0, 150 + 12 * 9, 12, WHITE);

    float total_time = std::accumulate(timings.begin(), timings.end(), 0.0f);
    float fps = 1000.0f / total_time;
    float real_time = GetFrameTime();

    DrawText(std::to_string(timings[0]).c_str(), 70, 150 + 12 * 0, 12, WHITE);
    DrawText(std::to_string(timings[1]).c_str(), 70, 150 + 12 * 1, 12, WHITE);
    DrawText(std::to_string(timings[2]).c_str(), 70, 150 + 12 * 2, 12, WHITE);
    DrawText(std::to_string(timings[3]).c_str(), 70, 150 + 12 * 3, 12, WHITE);
    DrawText(std::to_string(timings[4]).c_str(), 70, 150 + 12 * 4, 12, WHITE);
    DrawText(std::to_string(timings[5]).c_str(), 70, 150 + 12 * 5, 12, WHITE);
    DrawText(std::to_string(timings[6]).c_str(), 70, 150 + 12 * 6, 12, WHITE);
    DrawText(std::to_string(total_time).c_str(), 70, 150 + 12 * 7, 12, WHITE);
    DrawText(std::to_string(fps).c_str(), 70, 150 + 12 * 8, 12, WHITE);
    if (total_time < real_time)
        DrawText(std::to_string(real_time - total_time).c_str(), 70, 150 + 12 * 9, 12, WHITE);

    DrawText((std::to_string(drawn_meshes) + "/" + std::to_string(mesh_count_)).c_str(), 0, 300, 12, WHITE);

    DispatchSystems(&SystemHooks::ui);

    EndDrawing();

    timers[6].Stop();

    running_ = !WindowShouldClose();
}

size_t Engine::DrawAllMeshes(const decltype(registry_.view<const Transform, const Mesh, Material>()) &model_view,
                             Frustum frustum,
                             Shader shader) {
    size_t drawn_meshes = 0;
    mesh_count_ = 0;
    for (auto [ent, transform, mesh, material] : model_view.each()) {
        Matrix mat_translate = MatrixTranslate(transform.position.x, transform.position.y, transform.position.z);
        Matrix mat_rotate = QuaternionToMatrix(transform.rotation.GetQuaternion());
        Matrix mat_model = MatrixMultiply(mat_rotate, mat_translate);

        mesh_count_++;

        auto *bb = registry_.try_get<BoundingBox>(ent);
        if (bb) {
            BoundingBox transformed = TransformAABB(*bb, mat_model);
            DrawBoundingBox(transformed, RED);
            if (!IsAABBInFrustum(frustum, transformed)) {
                continue;
            }
        }
        material.shader = shader;

        DrawMesh(mesh, material, mat_model);
        drawn_meshes++;
    }
    return drawn_meshes;
}

void Engine::ReloadGBuffers() {
    UnloadGBuffers(gbuffers_);
    UnloadPresenter(presenter_);
    gbuffers_ = LoadGBuffers(GetScreenWidth(), GetScreenHeight());
    presenter_ = LoadPresenter(gbuffers_);
}

Entity Engine::Create() {
    return Entity(this, &registry_, registry_.create());
}

PhysicsEngine *Engine::GetPhysics() const {
    return physics_.get();
}

double Engine::GetDeltaTime() const {
    return (double) delta_time_ / (double) std::nano::den;
}

Input &Engine::GetInput() {
    return input_;
}

Config Engine::SwapConfig(Config con) {
    std::swap(con, config_);
    return con;
}
} // taco
