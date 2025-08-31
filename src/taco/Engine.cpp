// tacoEngine (c) Nikolas Wipper 2024

#include "Engine.h"

#include <vector>
#include <memory>
#include <ratio>

#include <raylib.h>
#include <raymath.h>
#include <tr_effects.h>

#include "rlgl.h"
#include "tr_timing.h"
#include "comp/Camera.h"
#include "comp/Lights.h"
#include "comp/System.h"
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

    physics_ = std::make_shared<PhysicsEngine>();
    debug_renderer_ = std::make_unique<RaylibDebugRenderer>();

    ReloadGBuffers();
}

void Engine::Run() {
    running_ = true;

    std::chrono::steady_clock::time_point last_frame = std::chrono::steady_clock::now();

    while (running_) {
        Render();

        auto now = std::chrono::steady_clock::now();

        std::chrono::duration<int64_t, std::nano> frame_delta = now - last_frame;
        delta_time_ = frame_delta.count();
        last_frame = now;

        Update();
    }
}

void Engine::Update() {
    auto visit_systems = [&](auto func) {
        for (auto [id, pool] : registry.storage()) {
            if (registry.storage(id)->type() != entt::type_id<std::shared_ptr<System>>())
                continue;
            auto system_view = entt::basic_view {registry.storage<std::shared_ptr<System>>(id)};
            for (auto [entity, system] : system_view.each()) {
                func(system, entity);
            }
        }
    };

    visit_systems([&](std::shared_ptr<System> system, entt::entity entity) {
        system->UpdateEarly(this, entity);
    });
    visit_systems([&](std::shared_ptr<System> system, entt::entity entity) {
        system->UpdatePrePhysics(this, entity);
    });

    auto collider_view = registry.view<Collider, Transform>();
    auto character_view = registry.view<Character, Transform>();
    auto link_view = registry.view<Link, Transform>();

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
        auto &remote_transform = registry.get<Transform>(link.entity);

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

    visit_systems([&](std::shared_ptr<System> system, entt::entity entity) {
        system->UpdatePostPhysics(this, entity);
    });
    visit_systems([&](std::shared_ptr<System> system, entt::entity entity) {
        system->UpdateLate(this, entity);
    });
}

void Engine::Render() {
    static Timer timers[7];
    float timings[7] = {0.0f};

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

    auto camera_view = registry.view<const Transform, const Camera>();
    auto model_view = registry.view<const Transform, const Mesh, Material>();
    auto env_view = registry.view<const Environment>();
    auto sky_view = registry.view<const Sky>();

    timers[0].Start();

    BeginGBufferMode(gbuffers_);
    ClearBackground(BLACK);

    Camera3D raylib_camera = {};
    size_t drawn_meshes;

    for (auto [_, transform, cam] : camera_view.each()) {
        Vector3 camera_target = transform.position + Vector3RotateByQuaternion(
                                    Vector3 {0, 0, -1},
                                    transform.rotation.GetQuaternion());
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

    auto sun_view = registry.view<const Transform, Sunlight>();

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

        FilterShadowMap(sun.shadow_map_, config_.shadow_map_quality);

        timers[2].Stop();
    }

    timers[3].Start();

    ClearPresenter(presenter_);

    if (config_.ssao)
        ApplySSAO(presenter_, raylib_camera);

    timers[3].Stop();
    timers[4].Start();

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

    DrawTexture(presenter_.target.texture, 0, 0, WHITE);
    rlEnableColorBlend();
    DrawFPS(0, 0);

    DrawText("Geometry", 0, 150 + 12 * 0, 12, WHITE);
    DrawText("Shadow", 0, 150 + 12 * 1, 12, WHITE);
    DrawText("Shadow PP", 0, 150 + 12 * 2, 12, WHITE);
    DrawText("SSAO", 0, 150 + 12 * 3, 12, WHITE);
    DrawText("Lighting", 0, 150 + 12 * 4, 12, WHITE);
    DrawText("PP", 0, 150 + 12 * 5, 12, WHITE);
    DrawText("Blit", 0, 150 + 12 * 6, 12, WHITE);

    DrawText(std::to_string(timings[0]).c_str(), 70, 150 + 12 * 0, 12, WHITE);
    DrawText(std::to_string(timings[1]).c_str(), 70, 150 + 12 * 1, 12, WHITE);
    DrawText(std::to_string(timings[2]).c_str(), 70, 150 + 12 * 2, 12, WHITE);
    DrawText(std::to_string(timings[3]).c_str(), 70, 150 + 12 * 3, 12, WHITE);
    DrawText(std::to_string(timings[4]).c_str(), 70, 150 + 12 * 4, 12, WHITE);
    DrawText(std::to_string(timings[5]).c_str(), 70, 150 + 12 * 5, 12, WHITE);
    DrawText(std::to_string(timings[6]).c_str(), 70, 150 + 12 * 6, 12, WHITE);

    DrawText((std::to_string(drawn_meshes) + "/" + std::to_string(mesh_count_)).c_str(), 0, 250, 12, WHITE);

    for (auto [id, pool] : registry.storage()) {
        if (registry.storage(id)->type() != entt::type_id<std::shared_ptr<System>>())
            continue;
        auto system_view = entt::basic_view {registry.storage<std::shared_ptr<System>>(id)};
        for (auto [entity, system] : system_view.each()) {
            system->UpdateUI(this, entity);
        }
    }

    rlDrawRenderBatchActive();

    timers[6].Stop();

    EndDrawing();

    running_ = !WindowShouldClose();
}

size_t Engine::DrawAllMeshes(const decltype(registry.view<const Transform, const Mesh, Material>()) &model_view,
                             Frustum frustum,
                             Shader shader) {
    size_t drawn_meshes = 0;
    mesh_count_ = 0;
    for (auto [ent, transform, mesh, material] : model_view.each()) {
        Matrix mat_translate = MatrixTranslate(transform.position.x, transform.position.y, transform.position.z);
        Matrix mat_rotate = QuaternionToMatrix(transform.rotation.GetQuaternion());
        Matrix mat_model = MatrixMultiply(mat_rotate, mat_translate);

        mesh_count_++;

        auto *bb = registry.try_get<BoundingBox>(ent);
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
    //UnloadGBuffers(gbuffers_);
    //UnloadPresenter(presenter_);
    gbuffers_ = LoadGBuffers(GetScreenWidth(), GetScreenHeight());
    presenter_ = LoadPresenter(gbuffers_);
}

std::shared_ptr<PhysicsEngine> Engine::GetPhysics() const {
    return physics_;
}

double Engine::GetDeltaTime() const {
    return (double) delta_time_ / (double) std::nano::den;
}

Config Engine::SwapConfig(Config con) {
    std::swap(con, config_);
    return con;
}
} // taco
