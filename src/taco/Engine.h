// tacoEngine (c) Nikolas Wipper 2024

#ifndef ENGINE_H
#define ENGINE_H

#include <entt/entt.hpp>
#include <entt/core/hashed_string.hpp>
#include <tacoRender.h>

#include "comp/Transform.h"
#include "Config.h"
#include "misc/Debug.h"
#include "Physics.h"

using namespace entt::literals;

namespace taco {
class Engine {
    bool running_ = false;
    int64_t delta_time_ = 0.0f;
    long long accumulator_ = 0.f;

    std::shared_ptr<PhysicsEngine> physics_;
    std::unique_ptr<RaylibDebugRenderer> debug_renderer_;
    Config config_;

    GBuffers gbuffers_;
    GBufferPresenter presenter_;

public:
    entt::registry registry;

    Engine();

    void Run();

    std::shared_ptr<PhysicsEngine> GetPhysics() const;
    double GetDeltaTime() const;

    Config SwapConfig(Config con);

private:
    void Update();
    void Render();
    void DrawAllMeshes(const decltype(registry.view<const Transform, const Mesh, Material>()) &model_view,
                       Shader shader = LoadMaterialDefault().shader);

    void ReloadGBuffers();
};
} // taco

#endif //ENGINE_H
