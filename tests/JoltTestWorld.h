#pragma once
// Testler için ortak Jolt dünyası kurulumu: katman tanımları, sistem
// başlatma ve statik kutu oluşturma yardımcıları.

#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>

#include <memory>
#include <thread>

namespace alazforge_test {

namespace Layers {
    static constexpr JPH::ObjectLayer NON_MOVING = 0;
    static constexpr JPH::ObjectLayer MOVING = 1;
    static constexpr JPH::ObjectLayer NUM_LAYERS = 2;
}

namespace BroadPhaseLayers {
    static constexpr JPH::BroadPhaseLayer NON_MOVING(0);
    static constexpr JPH::BroadPhaseLayer MOVING(1);
    static constexpr JPH::uint NUM_LAYERS(2);
}

class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface {
public:
    BPLayerInterfaceImpl() {
        mObjectToBroadPhase[Layers::NON_MOVING] = BroadPhaseLayers::NON_MOVING;
        mObjectToBroadPhase[Layers::MOVING] = BroadPhaseLayers::MOVING;
    }
    JPH::uint GetNumBroadPhaseLayers() const override { return BroadPhaseLayers::NUM_LAYERS; }
    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override {
        return mObjectToBroadPhase[inLayer];
    }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer) const override { return "?"; }
#endif
private:
    JPH::BroadPhaseLayer mObjectToBroadPhase[Layers::NUM_LAYERS];
};

class ObjectVsBroadPhaseLayerFilterImpl : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override {
        switch (inLayer1) {
        case Layers::NON_MOVING: return inLayer2 == BroadPhaseLayers::MOVING;
        case Layers::MOVING: return true;
        default: return false;
        }
    }
};

class ObjectLayerPairFilterImpl : public JPH::ObjectLayerPairFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override {
        switch (inObject1) {
        case Layers::NON_MOVING: return inObject2 == Layers::MOVING;
        case Layers::MOVING: return true;
        default: return false;
        }
    }
};

// Jolt dünyasını kurar; kapsam sonunda temizler
class TestWorld {
public:
    TestWorld() {
        JPH::RegisterDefaultAllocator();
        JPH::Factory::sInstance = new JPH::Factory();
        JPH::RegisterTypes();

        tempAllocator = std::make_unique<JPH::TempAllocatorImpl>(10 * 1024 * 1024);
        jobSystem = std::make_unique<JPH::JobSystemThreadPool>(
            JPH::cMaxPhysicsJobs, JPH::cMaxPhysicsBarriers,
            (int)std::thread::hardware_concurrency() - 1);

        physics.Init(1024, 0, 1024, 1024, bpLayerInterface,
                     objectVsBroadPhaseFilter, objectLayerPairFilter);
    }

    ~TestWorld() {
        JPH::UnregisterTypes();
        delete JPH::Factory::sInstance;
        JPH::Factory::sInstance = nullptr;
    }

    // Statik kutu ekler; userData'ya malzeme id'si yazılır.
    // İnce duvarlar için convexRadius 0 verilir.
    JPH::BodyID AddStaticBox(JPH::RVec3 center, JPH::Vec3 halfExtents, JPH::uint64 materialId) {
        const float convexRadius =
            halfExtents.ReduceMin() < JPH::cDefaultConvexRadius ? 0.0f
                                                                : JPH::cDefaultConvexRadius;
        JPH::BodyCreationSettings settings(
            new JPH::BoxShape(halfExtents, convexRadius), center,
            JPH::Quat::sIdentity(), JPH::EMotionType::Static, Layers::NON_MOVING);
        settings.mUserData = materialId;
        JPH::BodyID id = physics.GetBodyInterface().CreateAndAddBody(
            settings, JPH::EActivation::DontActivate);
        physics.OptimizeBroadPhase();
        return id;
    }

    // Fiziği bir adım ilerletir (dahili allocator + job system ile)
    void Step(float deltaTime, int collisionSteps = 1) {
        physics.Update(deltaTime, collisionSteps, tempAllocator.get(), jobSystem.get());
    }

    // Büyük statik zemin ekler, üst yüzeyi y=0
    JPH::BodyID AddGround(float halfSize = 200.0f, JPH::uint64 materialId = 0) {
        return AddStaticBox(JPH::RVec3(0.0, -1.0, 0.0),
                            JPH::Vec3(halfSize, 1.0f, halfSize), materialId);
    }

    JPH::PhysicsSystem physics;

private:
    BPLayerInterfaceImpl bpLayerInterface;
    ObjectVsBroadPhaseLayerFilterImpl objectVsBroadPhaseFilter;
    ObjectLayerPairFilterImpl objectLayerPairFilter;
    std::unique_ptr<JPH::TempAllocatorImpl> tempAllocator;
    std::unique_ptr<JPH::JobSystemThreadPool> jobSystem;
};

} // namespace alazforge_test
