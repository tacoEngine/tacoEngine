// tacoEngine (c) Nikolas Wipper 2024

#ifndef ENGINE_H
#define ENGINE_H

#include <entt/entt.hpp>

#include "Debug.h"
#include "Physics.h"

namespace taco {

class Engine {
    bool running_ = false;
    std::shared_ptr<PhysicsEngine> physics_;
    std::unique_ptr<RaylibDebugRenderer> debug_renderer_;
public:
    entt::registry registry;

    Engine();

    void Run();

    std::shared_ptr<PhysicsEngine> GetPhysics() const;
private:
    void Update(double delta_time);
    void Render();
};

} // taco

#endif //ENGINE_H
