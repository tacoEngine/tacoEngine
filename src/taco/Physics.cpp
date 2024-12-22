// tacoEngine (c) Nikolas Wipper 2024

#include "Physics.h"

#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/JobSystemThreadPool.h>

#include "Log.h"

taco::PhysicsEngine::PhysicsEngine() : temp_alloc_(new JPH::TempAllocatorImplWithMallocFallback(10 * 1024 * 1024)),
                                       job_system_(new JPH::JobSystemThreadPool(JPH::cMaxPhysicsJobs,
                                           JPH::cMaxPhysicsBarriers,
                                           std::thread::hardware_concurrency() - 1)) {
    // Startup physics
    JPH::RegisterDefaultAllocator();

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
    system_.Update(delta_time, 1, temp_alloc_.get(), job_system_.get());
}

