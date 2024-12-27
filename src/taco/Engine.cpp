// tacoEngine (c) Nikolas Wipper 2024

#include "Engine.h"

#include <vector>
#include <memory>

#include <raylib.h>
#include <raymath.h>

#include "Camera.h"
#include "Transform.h"

namespace taco {
Engine::Engine() {
#if defined(NDEBUG)
    ChangeDirectory(GetApplicationDirectory());
#endif

    SetConfigFlags(FLAG_WINDOW_HIGHDPI | FLAG_WINDOW_UNDECORATED | FLAG_WINDOW_MAXIMIZED);
    InitWindow(0, 0, "taco");

    SetExitKey(0);

#if !defined(NDEBUG)
    SetTargetFPS(60);
#endif

    physics_ = std::make_shared<PhysicsEngine>();
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
    auto camera_view = registry.view<const Transform, const Camera>();

    BeginDrawing();
    ClearBackground(BLACK);

    for (auto [_, transform, cam] : camera_view.each()) {
        Vector3 camera_target = transform.position + Vector3RotateByQuaternion(Vector3 {0, 0, -1}, transform.rotation);
        Camera3D raylib_camera = {transform.position, camera_target, {0, 1, 0}, cam.fov, CAMERA_PERSPECTIVE};

        BeginMode3D(raylib_camera);

        auto model_view = registry.view<const Transform, const Mesh, const Material>();

        for (auto [_, transform, mesh, material] : model_view.each()) {
            Matrix mat_translate = MatrixTranslate(transform.position.x, transform.position.y, transform.position.z);
            Matrix mat_rotate = QuaternionToMatrix(transform.rotation);
            DrawMesh(mesh, material, MatrixMultiply(mat_translate, mat_rotate));
        }

        EndMode3D();
    }

    EndDrawing();

    running_ = !WindowShouldClose();
}

std::shared_ptr<PhysicsEngine> Engine::GetPhysics() const {
    return physics_;
}
} // taco
