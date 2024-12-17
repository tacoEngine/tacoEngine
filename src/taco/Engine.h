// tacoEngine (c) Nikolas Wipper 2024

#ifndef ENGINE_H
#define ENGINE_H

#include <entt/entt.hpp>
#include <Jolt/Jolt.h>

namespace taco {

class Engine {
    bool running_ = false;
public:
    entt::registry registry;

    Engine();

    void Run();
private:
    void Update(double delta_time);
    void Render();
};

} // taco

#endif //ENGINE_H
