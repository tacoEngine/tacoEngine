// tacoEngine (c) Nikolas Wipper 2024

#ifndef ENGINE_H
#define ENGINE_H

#include <entt/entt.hpp>

#include "Config.h"
#include "Debug.h"
#include "Physics.h"
#include "tacoRender.h"
#include "Transform.h"

namespace taco {

class Engine {
    bool running_ = false;
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
    Config SwapConfig(Config con);
private:
    void Update(double delta_time);
    void Render();
    void DrawAllMeshes(const decltype(registry.view<const Transform, const Mesh, Material>()) &model_view, Shader shader = LoadMaterialDefault().shader);

    void ReloadGBuffers();
};

} // taco

#endif //ENGINE_H
