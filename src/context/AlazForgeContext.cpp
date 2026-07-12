#include "context/AlazForgeContext.h"

#include "core/JoltAdapter.h"

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/StateRecorderImpl.h>
#include <Jolt/RegisterTypes.h>

#include <algorithm>
#include <thread>

namespace alazforge {

namespace {
// JPH::Factory::sInstance process-global; yalnizca ilk context'te kurulur,
// yalnizca son context yikilirken sokulur.
int g_joltInitRefCount = 0;
} // namespace

class BPLayerInterfaceImpl final : public JPH::BroadPhaseLayerInterface {
public:
    BPLayerInterfaceImpl(JPH::ObjectLayer nonMoving, JPH::ObjectLayer moving) {
        objectToBroadPhase[nonMoving] = JPH::BroadPhaseLayer(0);
        objectToBroadPhase[moving] = JPH::BroadPhaseLayer(1);
    }
    JPH::uint GetNumBroadPhaseLayers() const override { return 2; }
    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer inLayer) const override {
        return objectToBroadPhase[inLayer];
    }
#if defined(JPH_EXTERNAL_PROFILE) || defined(JPH_PROFILE_ENABLED)
    const char* GetBroadPhaseLayerName(JPH::BroadPhaseLayer) const override { return "?"; }
#endif

private:
    JPH::BroadPhaseLayer objectToBroadPhase[2];
};

class ObjectVsBroadPhaseLayerFilterImpl final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    ObjectVsBroadPhaseLayerFilterImpl(JPH::ObjectLayer nonMoving, JPH::ObjectLayer moving)
        : nonMovingLayer(nonMoving), movingLayer(moving) {}
    bool ShouldCollide(JPH::ObjectLayer inLayer1, JPH::BroadPhaseLayer inLayer2) const override {
        if (inLayer1 == nonMovingLayer) return inLayer2 == JPH::BroadPhaseLayer(1);
        return true;
    }

private:
    JPH::ObjectLayer nonMovingLayer, movingLayer;
};

class ObjectLayerPairFilterImpl final : public JPH::ObjectLayerPairFilter {
public:
    ObjectLayerPairFilterImpl(JPH::ObjectLayer nonMoving, JPH::ObjectLayer moving)
        : nonMovingLayer(nonMoving), movingLayer(moving) {}
    bool ShouldCollide(JPH::ObjectLayer inObject1, JPH::ObjectLayer inObject2) const override {
        if (inObject1 == nonMovingLayer) return inObject2 == movingLayer;
        return true;
    }

private:
    JPH::ObjectLayer nonMovingLayer, movingLayer;
};

AlazForgeContext::AlazForgeContext() : AlazForgeContext(Config()) {}

AlazForgeContext::AlazForgeContext(const Config& inConfig) : config(inConfig) {
    if (g_joltInitRefCount == 0) {
        JPH::RegisterDefaultAllocator();
        JPH::Factory::sInstance = new JPH::Factory();
        JPH::RegisterTypes();
    }
    ++g_joltInitRefCount;

    tempAllocator = std::make_unique<JPH::TempAllocatorImpl>(10 * 1024 * 1024);
    const int numThreads =
        config.numThreads >= 0
            ? config.numThreads
            : std::max(1, static_cast<int>(std::thread::hardware_concurrency()) - 1);
    jobSystem = std::make_unique<JPH::JobSystemThreadPool>(JPH::cMaxPhysicsJobs,
                                                           JPH::cMaxPhysicsBarriers, numThreads);

    bpLayerInterface =
        std::make_unique<BPLayerInterfaceImpl>(config.nonMovingLayer, config.movingLayer);
    objectVsBroadPhaseFilter = std::make_unique<ObjectVsBroadPhaseLayerFilterImpl>(
        config.nonMovingLayer, config.movingLayer);
    objectLayerPairFilter =
        std::make_unique<ObjectLayerPairFilterImpl>(config.nonMovingLayer, config.movingLayer);

    physics = std::make_unique<JPH::PhysicsSystem>();
    physics->Init(config.maxBodies, 0, config.maxBodyPairs, config.maxContactConstraints,
                  *bpLayerInterface, *objectVsBroadPhaseFilter, *objectLayerPairFilter);

    materials = std::make_unique<MaterialDatabase>(MaterialDatabase::CreateDefault());
    ballistics = std::make_unique<BallisticsSystem>(*physics, *materials);

    wind = std::make_unique<WindSystem>(config.wind);
    zones = std::make_unique<FrictionZoneSystem>();
    buoyancy = std::make_unique<BuoyancySystem>();
    lod = std::make_unique<LODSystem>(LODConfig{});
    debugDraw = std::make_unique<DebugDrawBridge>();

    collisionEvents = std::make_unique<CollisionEventSystem>(config.collisionEventMinImpactSpeed);
    collisionEvents->Attach(*physics);
}

AlazForgeContext::~AlazForgeContext() {
    destructible.reset();
    terrain.reset();
    collisionEvents.reset();
    debugDraw.reset();
    lod.reset();
    buoyancy.reset();
    zones.reset();
    wind.reset();
    ballistics.reset();
    materials.reset();
    physics.reset();
    objectLayerPairFilter.reset();
    objectVsBroadPhaseFilter.reset();
    bpLayerInterface.reset();
    jobSystem.reset();
    tempAllocator.reset();

    --g_joltInitRefCount;
    if (g_joltInitRefCount == 0) {
        JPH::UnregisterTypes();
        delete JPH::Factory::sInstance;
        JPH::Factory::sInstance = nullptr;
    }
}

void AlazForgeContext::Step(float dt, int collisionSteps, const Vec3* lodReferencePoint) {
    // Sirali guncelleme: zones/buoyancy fizik adimindan ONCE (o adimda
    // gecerli olacak friction/drag degerlerini kursunlar), fizik adimi,
    // sonra LOD (bir sonraki adim icin uyut/uyandir kararini versin).
    wind->Update(dt);
    zones->Update(*physics);
    buoyancy->Update(*physics, dt);

    physics->Update(dt, collisionSteps, tempAllocator.get(), jobSystem.get());

    if (lodReferencePoint) lod->Update(*physics, *lodReferencePoint);
}

JPH::PhysicsSystem& AlazForgeContext::Physics() { return *physics; }

BallisticsSystem& AlazForgeContext::Ballistics() { return *ballistics; }

MaterialDatabase& AlazForgeContext::Materials() { return *materials; }

WindSystem& AlazForgeContext::Wind() { return *wind; }

FrictionZoneSystem& AlazForgeContext::Zones() { return *zones; }

BuoyancySystem& AlazForgeContext::Buoyancy() { return *buoyancy; }

LODSystem& AlazForgeContext::LOD() { return *lod; }

DebugDrawBridge& AlazForgeContext::DebugDraw() { return *debugDraw; }

void AlazForgeContext::DrainCollisionEvents(std::vector<CollisionEvent>& out) {
    collisionEvents->DrainEvents(out);
}

TerrainDeformSystem& AlazForgeContext::EnableTerrain(const TerrainDeformSystem::Config& cfg) {
    if (!terrain) terrain = std::make_unique<TerrainDeformSystem>(cfg);
    return *terrain;
}

DestructibleStructureSystem& AlazForgeContext::EnableDestructible(
    const DestructibleStructureSystem::Config& cfg) {
    if (!destructible) destructible = std::make_unique<DestructibleStructureSystem>(cfg);
    return *destructible;
}

size_t AlazForgeContext::SnapshotActiveBodies(std::vector<BodySnapshot>& out) const {
    out.clear();

    JPH::BodyIDVector activeIds;
    physics->GetActiveBodies(JPH::EBodyType::RigidBody, activeIds);

    for (JPH::BodyID id : activeIds) {
        JPH::BodyLockRead lock(physics->GetBodyLockInterface(), id);
        if (!lock.Succeeded()) continue;
        const JPH::Body& body = lock.GetBody();

        BodySnapshot snap;
        snap.bodyId = id.GetIndexAndSequenceNumber();
        snap.transform.position = FromJolt(JPH::Vec3(body.GetPosition()));
        snap.transform.rotation = FromJolt(body.GetRotation());
        snap.linearVelocity = FromJolt(body.GetLinearVelocity());
        snap.angularVelocity = FromJolt(body.GetAngularVelocity());
        out.push_back(snap);
    }
    return out.size();
}

size_t AlazForgeContext::SaveWorldState(std::vector<uint8_t>& out) const {
    JPH::StateRecorderImpl recorder;
    physics->SaveState(recorder);
    const std::string data = recorder.GetData();
    out.assign(data.begin(), data.end());
    return out.size();
}

bool AlazForgeContext::RestoreWorldState(const std::vector<uint8_t>& data) {
    if (data.empty()) return false;
    JPH::StateRecorderImpl recorder;
    recorder.WriteBytes(data.data(), data.size());
    recorder.Rewind();
    return physics->RestoreState(recorder);
}

} // namespace alazforge
