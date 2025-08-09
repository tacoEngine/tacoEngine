// $PROJECT_HEADER (c) Nikolas 2025

#ifndef SYSTEM_H
#define SYSTEM_H

#include <entt/entt.hpp>

namespace taco {
class Engine;

class System {
  public:
    virtual void UpdateEarly(Engine *engine, entt::entity entity) {}
    virtual void UpdatePrePhysics(Engine *engine, entt::entity entity) {}
    virtual void UpdatePostPhysics(Engine *engine, entt::entity entity) {}
    virtual void UpdateLate(Engine *engine, entt::entity entity) {}
    virtual void UpdateUI(Engine *engine, entt::entity entity) {}
};
}

#endif //SYSTEM_H
