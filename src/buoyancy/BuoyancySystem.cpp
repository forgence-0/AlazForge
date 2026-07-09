#include "buoyancy/BuoyancySystem.h"

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyLock.h>

#include <algorithm>

namespace alazforge {

size_t BuoyancySystem::AddVolume(const WaterVolume& volume) {
    for (size_t i = 0; i < volumes.size(); ++i) {
        if (!volumes[i].active) {
            volumes[i] = {volume, true};
            return i;
        }
    }
    volumes.push_back({volume, true});
    return volumes.size() - 1;
}

void BuoyancySystem::RemoveVolume(size_t index) {
    if (index < volumes.size()) volumes[index].active = false;
}

void BuoyancySystem::TrackBody(JPH::BodyID body) {
    if (std::find(trackedBodies.begin(), trackedBodies.end(), body) == trackedBodies.end())
        trackedBodies.push_back(body);
}

void BuoyancySystem::UntrackBody(JPH::BodyID body) {
    trackedBodies.erase(std::remove(trackedBodies.begin(), trackedBodies.end(), body),
                        trackedBodies.end());
}

const WaterVolume* BuoyancySystem::FindVolumeContaining(const Vec3& worldPos) const {
    for (const StoredVolume& sv : volumes) {
        if (!sv.active) continue;
        const WaterVolume& v = sv.volume;
        if (worldPos.x >= v.boundsMin.x && worldPos.x <= v.boundsMax.x &&
            worldPos.y >= v.boundsMin.y && worldPos.y <= v.boundsMax.y &&
            worldPos.z >= v.boundsMin.z && worldPos.z <= v.boundsMax.z) {
            return &v;
        }
    }
    return nullptr;
}

void BuoyancySystem::Update(JPH::PhysicsSystem& physics, float dt) {
    JPH::BodyLockInterface const& lockInterface = physics.GetBodyLockInterface();

    for (JPH::BodyID id : trackedBodies) {
        JPH::BodyLockWrite lock(lockInterface, id);
        if (!lock.Succeeded()) continue;
        JPH::Body& body = lock.GetBody();
        if (!body.IsActive()) continue;

        const Vec3 pos = FromJolt(JPH::Vec3(body.GetPosition()));
        const WaterVolume* volume = FindVolumeContaining(pos);
        if (!volume) continue;

        const JPH::RVec3 surfacePos(body.GetPosition().GetX(), volume->surfaceY,
                                    body.GetPosition().GetZ());
        body.ApplyBuoyancyImpulse(surfacePos, JPH::Vec3(0, 1, 0), volume->buoyancyFactor,
                                  volume->linearDrag, volume->angularDrag,
                                  ToJolt(volume->flowVelocity), physics.GetGravity(), dt);
    }
}

bool BuoyancySystem::IsSubmerged(JPH::PhysicsSystem& physics, JPH::BodyID body) const {
    JPH::BodyLockRead lock(physics.GetBodyLockInterface(), body);
    if (!lock.Succeeded()) return false;
    const Vec3 pos = FromJolt(JPH::Vec3(lock.GetBody().GetPosition()));
    const WaterVolume* volume = FindVolumeContaining(pos);
    return volume != nullptr && pos.y < volume->surfaceY;
}

} // namespace alazforge
