// tacoEngine (c) Nikolas Wipper 2024

#include "Engine.h"

#include <vector>
#include <memory>

#include <raylib.h>
#include <raymath.h>
#include <tr_effects.h>

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
        delta_time_ = current_time - last_time;
        last_time = current_time;

        Update(delta_time_);
    }
}

void Engine::Update(double delta_time) {
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

    physics_->Update(delta_time);

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
    if (IsWindowResized())
        ReloadGBuffers();

    auto camera_view = registry.view<const Transform, const Camera>();
    auto model_view = registry.view<const Transform, const Mesh, Material>();
    auto env_view = registry.view<const Environment>();
    auto sky_view = registry.view<const Sky>();

    BeginGBufferMode(gbuffers_);
    ClearBackground(BLACK);

    Camera3D raylib_camera = {};

    for (auto [_, transform, cam] : camera_view.each()) {
        Vector3 camera_target = transform.position + Vector3RotateByQuaternion(
            Vector3 {0, 0, -1},
            transform.rotation.GetQuaternion());
        raylib_camera = {transform.position, camera_target, {0, 1, 0}, cam.fov, CAMERA_PERSPECTIVE};

        BeginMode3D(raylib_camera);

        DrawAllMeshes(model_view, GetGBufferShader());

        if (config_.debug_physics)
            physics_->Render();

        for (auto [_, sky] : sky_view.each()) {
            DrawSkybox(sky.skybox_, WHITE);
        }

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
            Vector3 direction = Vector3RotateByQuaternion(Vector3 {0, 0, -1}, transform.rotation.GetQuaternion());
            BeginShadowMap(sun.shadow_map_, raylib_camera, direction, i);

            ClearBackground(BLANK);

            DrawAllMeshes(model_view);

            EndShadowMap();
        }

        FilterShadowMap(sun.shadow_map_);
    }

    ClearPresenter(presenter_);

    if (config_.ssao)
        ApplySSAO(presenter_, raylib_camera);

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

    ApplyToneMapping(presenter_, config_.tone_mapper);
    ApplyGammaCorrection(presenter_, config_.gamma_correction);

    Present(presenter_);

    running_ = !WindowShouldClose();
}

void Engine::DrawAllMeshes(const decltype(registry.view<const Transform, const Mesh, Material>()) &model_view,
                           Shader shader) {
    for (auto [_, transform, mesh, material] : model_view.each()) {
        Matrix mat_translate = MatrixTranslate(transform.position.x, transform.position.y, transform.position.z);
        Matrix mat_rotate = QuaternionToMatrix(transform.rotation.GetQuaternion());
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

double Engine::GetDeltaTime() const {
    return delta_time_;
}

Config Engine::SwapConfig(Config con) {
    std::swap(con, config_);
    return con;
}
} // taco
