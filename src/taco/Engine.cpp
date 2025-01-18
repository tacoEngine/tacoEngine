// tacoEngine (c) Nikolas Wipper 2024

#include "Engine.h"

#include <vector>
#include <memory>

#include <raylib.h>
#include <raymath.h>
#include <tr_effects.h>

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

#if !defined(NDEBUG)
    SetTargetFPS(60);
#endif

    physics_ = std::make_shared<PhysicsEngine>();
    debug_renderer_ = std::make_unique<RaylibDebugRenderer>();

    ReloadGBuffers();
}

void Engine::Run() {
    running_ = true;

    double last_time = GetTime();

    while (running_) {
        Render();

        double current_time = GetTime();
        double delta_time = current_time - last_time;
        last_time = current_time;

        Update(delta_time);
    }
}

void Engine::Update(double delta_time) {
    auto collider_view = registry.view<Collider, Transform>();

    for (auto [_, collider, transform] : collider_view.each()) {
        collider.SetPosition(transform.position);
        collider.SetRotation(transform.rotation);
    }

    physics_->Update(delta_time);

    for (auto [_, collider, transform] : collider_view.each()) {
        transform.position = collider.GetPosition();
        transform.rotation = collider.GetRotation();
    }
}

void Engine::Render() {
    if (IsWindowResized())
        ReloadGBuffers();

    auto camera_view = registry.view<const Transform, const Camera>();
    auto model_view = registry.view<const Transform, const Mesh, Material>();

    BeginGBufferMode(gbuffers_);
    ClearBackground(BLACK);

    Camera3D raylib_camera = {};

    for (auto [_, transform, cam] : camera_view.each()) {
        Vector3 camera_target = transform.position + Vector3RotateByQuaternion(Vector3 {0, 0, -1}, transform.rotation);
        raylib_camera = {transform.position, camera_target, {0, 1, 0}, cam.fov, CAMERA_PERSPECTIVE};

        BeginMode3D(raylib_camera);

        DrawAllMeshes(model_view, GetGBufferShader());

        if (config_.debug_physics)
            physics_->Render();

        EndMode3D();
    }

    EndGBufferMode();

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

        for (int i = 0; i < cascadeCount; i++) {
            Vector3 direction = Vector3RotateByQuaternion(Vector3 {0, 0, -1}, transform.rotation);
            BeginShadowMap(sun.shadow_map_, raylib_camera, direction, i);

            ClearBackground(BLANK);

            DrawAllMeshes(model_view);

            EndShadowMap();
        }

        FilterShadowMap(sun.shadow_map_);
    }

    ClearPresenter(presenter_);

    BeginLightingPass(presenter_);

    for (auto [_, transform, sun] : sun_view.each()) {
        LightSun(presenter_,
                 raylib_camera,
                 Vector3RotateByQuaternion(Vector3 {0, 0, -1}, transform.rotation),
                 sun.intensity,
                 sun.color,
                 sun.shadow_casting ? sun.shadow_map_ : NULL_SHADOW_MAP);
    }

    EndLightingPass();

    ApplyToneMapping(presenter_, config_.tone_mapper);
    ApplyGammaCorrection(presenter_, config_.gamma_correction);

    Present(presenter_);

    running_ = !WindowShouldClose();
}

void Engine::DrawAllMeshes(const decltype(registry.view<const Transform, const Mesh, Material>()) &model_view, Shader shader) {
    for (auto [_, transform, mesh, material] : model_view.each()) {
        Matrix mat_translate = MatrixTranslate(transform.position.x, transform.position.y, transform.position.z);
        Matrix mat_rotate = QuaternionToMatrix(transform.rotation);
        material.shader = shader;
        DrawMesh(mesh, material, MatrixMultiply(mat_rotate, mat_translate));
    }
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

Config Engine::SwapConfig(Config con) {
    std::swap(con, config_);
    return con;
}
} // taco
