// tacoEngine (c) Nikolas Wipper 2024

#include "Physics.h"

#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/RegisterTypes.h>

#include <log/log.h>
#include <raymath.h>
#include <utility>

#include "misc/Log.h"

taco::Collider::Collider(std::shared_ptr<PhysicsEngine> physics, JPH::BodyID body_id, Vector3 com) : body_id_(body_id),
    physics_(std::move(physics)), com_(com) {}

void taco::Collider::SetPosition(Vector3 position) {
    JPH::RVec3 pos(position.x, position.y, position.z);
    physics_->body_interface_.SetPosition(body_id_, pos, JPH::EActivation::DontActivate);
}

Vector3 taco::Collider::GetPosition() const {
    JPH::RVec3 pos = physics_->body_interface_.GetPosition(body_id_);
    return {pos.GetX(), pos.GetY(), pos.GetZ()};
}

void taco::Collider::SetRotation(Quaternion rotation) {
    JPH::QuatArg rot(rotation.x, rotation.y, rotation.z, rotation.w);
    physics_->body_interface_.SetRotation(body_id_, rot, JPH::EActivation::DontActivate);
}

Quaternion taco::Collider::GetRotation() const {
    JPH::Quat rot = physics_->body_interface_.GetRotation(body_id_);

    return {rot.GetX(), rot.GetY(), rot.GetZ(), rot.GetW()};
}

Vector3 taco::Collider::GetCenterOfMass() const {
    return com_;
}

taco::PhysicsEngine::PhysicsEngine() : body_interface_(system_.GetBodyInterface()) {
    // Startup physics
    JPH::RegisterDefaultAllocator();

    temp_alloc_ = std::make_unique<JPH::TempAllocatorImplWithMallocFallback>(10 * 1024 * 1024);
    job_system_ = std::make_unique<JPH::JobSystemThreadPool>(JPH::cMaxPhysicsJobs,
                                                             JPH::cMaxPhysicsBarriers,
                                                             std::thread::hardware_concurrency() - 1);

    JPH::Trace = TraceImpl;
    JPH_IF_ENABLE_ASSERTS(JPH::AssertFailed = AssertFailedImpl;)

    JPH::Factory::sInstance = new JPH::Factory();

    JPH::RegisterTypes();

    system_.Init(65536,
                 0,
                 65536,
                 10240,
                 broad_phase_layer_interface_,
                 object_vs_broadphase_layer_filter_,
                 object_vs_object_layer_filter_);
}

void taco::PhysicsEngine::Update(double delta_time) {
    int steps = std::ceil((1. / 60.) / delta_time);
    system_.Update(delta_time, steps, temp_alloc_.get(), job_system_.get());

}

void taco::PhysicsEngine::Render() {
    system_.DrawBodies(JPH::BodyManager::DrawSettings{}, JPH::DebugRenderer::sInstance);
}

taco::Collider taco::PhysicsEngine::CreateSphereCollider(double radius) {
    JPH::BodyCreationSettings sphere_settings(new JPH::SphereShape(radius),
                                              JPH::RVec3(0.0, 0.0, 0.0),
                                              JPH::Quat::sIdentity(),
                                              JPH::EMotionType::Dynamic,
                                              Layers::MOVING);
    JPH::BodyID sphere_id = body_interface_.CreateAndAddBody(sphere_settings, JPH::EActivation::Activate);

    return Collider(shared_from_this(), sphere_id, Vector3Zero());
}

taco::Collider taco::PhysicsEngine::CreateMeshCollider(Mesh mesh, bool dynamic) {
    JPH::ConvexHullShapeSettings settings;

    if (mesh.indices) {
        JPH::Array<JPH::Vec3> points;
        points.reserve(mesh.triangleCount * 3);

        for (int i = 0; i < mesh.triangleCount * 3; i++) {
            int index = mesh.indices[i];
            points.emplace_back(mesh.vertices[index * 3], mesh.vertices[index * 3 + 1], mesh.vertices[index * 3 + 2]);
        }
        // Construct the settings into the stack var, because there is neither a copy nor a move assignment operator
        new(&settings) JPH::ConvexHullShapeSettings(points);
    }
    else {
        new(&settings) JPH::ConvexHullShapeSettings((JPH::Vec3 *) mesh.vertices, mesh.vertexCount);
    }

    JPH::ConvexHullShape::ShapeResult result;
    JPH::BodyCreationSettings sphere_settings(new JPH::ConvexHullShape(settings, result),
                                              JPH::RVec3(0.0, 0.0, 0.0),
                                              JPH::Quat::sIdentity(),
                                              dynamic ? JPH::EMotionType::Dynamic : JPH::EMotionType::Static,
                                              dynamic ? Layers::MOVING : Layers::NON_MOVING);
    JPH::BodyID mesh_id = body_interface_.CreateAndAddBody(sphere_settings, JPH::EActivation::Activate);

    Vector3 center_of_mass = Vector3Zero();
    if (!result.HasError()) {
        auto com = result.Get()->GetCenterOfMass();
        center_of_mass.x = com.GetX();
        center_of_mass.y = com.GetY();
        center_of_mass.z = com.GetZ();
    }

    return Collider(shared_from_this(), mesh_id, center_of_mass);
}
