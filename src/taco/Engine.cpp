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
    ChangeDirectory(GetApplicationDirectory());

    SetConfigFlags(FLAG_WINDOW_HIGHDPI | FLAG_WINDOW_UNDECORATED | FLAG_WINDOW_MAXIMIZED);
    InitWindow(0, 0, "taco");

    SetExitKey(0);

#if !defined(NDEBUG)
    SetTargetFPS(60);
#endif
}

void Engine::Run() {
    running_ = true;

    double last_time = GetTime();

    while (running_) {
        double current_time = GetTime();
        double delta_time = current_time - last_time;

        Update(delta_time);
        Render();
    }
}

void Engine::Update(double delta_time) {}

void Engine::Render() {
    BeginDrawing();
    ClearBackground(BLACK);

    EndDrawing();

    running_ = !WindowShouldClose();
}
} // taco
