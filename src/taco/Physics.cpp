// tacoEngine (c) Nikolas Wipper 2024

#include "Physics.h"

#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/RegisterTypes.h>

#include <log/log.h>
#include <raymath.h>
#include <utility>

#include "Jolt/Physics/Collision/Shape/CapsuleShape.h"
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

void taco::Collider::SetVelocity(Vector3 velocity) {
    JPH::RVec3 vel(velocity.x, velocity.y, velocity.z);
    physics_->body_interface_.SetLinearVelocity(body_id_, vel);
}

Vector3 taco::Collider::GetVelocity() const {
    JPH::RVec3 vel = physics_->body_interface_.GetLinearVelocity(body_id_);
    return {vel.GetX(), vel.GetY(), vel.GetZ()};
}

Vector3 taco::Collider::GetCenterOfMass() const {
    return com_;
}

taco::Character::Character(std::shared_ptr<PhysicsEngine> physics, std::unique_ptr<JPH::Character> character) :
    physics_(std::move(physics)), character_(std::move(character)) {}

void taco::Character::SetPosition(Vector3 position) {
    JPH::RVec3 pos(position.x, position.y, position.z);
    character_->SetPosition(pos, JPH::EActivation::DontActivate);
}

Vector3 taco::Character::GetPosition() const {
    JPH::RVec3 pos = character_->GetPosition();
    return {pos.GetX(), pos.GetY(), pos.GetZ()};
}

void taco::Character::SetRotation(Quaternion rotation) {
    JPH::Quat rot(rotation.x, rotation.y, rotation.z, rotation.w);
    character_->SetRotation(rot);
}

Quaternion taco::Character::GetRotation() const {
    JPH::Quat rot = character_->GetRotation();
    return {rot.GetX(), rot.GetY(), rot.GetZ(), rot.GetW()};
}

void taco::Character::SetVelocity(Vector3 velocity) {
    JPH::RVec3 vel(velocity.x, velocity.y, velocity.z);
   character_->SetLinearVelocity(vel);
}

Vector3 taco::Character::GetVelocity() const {
    JPH::RVec3 vel = character_->GetLinearVelocity();
    return {vel.GetX(), vel.GetY(), vel.GetZ()};
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
    system_.DrawBodies(JPH::BodyManager::DrawSettings {}, JPH::DebugRenderer::sInstance);
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
    } else {
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

taco::Character taco::PhysicsEngine::CreateCharacter(double height, double radius) {
    JPH::Ref<JPH::CharacterSettings> settings = new JPH::CharacterSettings();
    settings->mMaxSlopeAngle = DEG2RAD * 45.0f;
    settings->mLayer = Layers::MOVING;
    settings->mShape = JPH::RotatedTranslatedShapeSettings(JPH::Vec3(0, -0.5f * height + radius, 0),
                                                           JPH::Quat::sIdentity(),
                                                           new JPH::CapsuleShape(0.5f * height, radius)).Create().Get();
    settings->mFriction = 0.5f;
    settings->mSupportingVolume = JPH::Plane(JPH::Vec3::sAxisY(), -radius);
    // Accept contacts that touch the lower sphere of the capsule
    auto character = std::make_unique<JPH::Character>(settings,
                                                      JPH::RVec3::sZero(),
                                                      JPH::Quat::sIdentity(),
                                                      0,
                                                      &system_);
    character->AddToPhysicsSystem(JPH::EActivation::Activate);

    return Character(shared_from_this(), std::move(character));
}
