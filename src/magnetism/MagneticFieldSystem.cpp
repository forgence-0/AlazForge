#include "magnetism/MagneticFieldSystem.h"

#include "core/JoltAdapter.h"

#include <Jolt/Physics/Body/Body.h>
#include <Jolt/Physics/Body/BodyLock.h>

#include <algorithm>
#include <cmath>

namespace alazforge {

size_t MagneticFieldSystem::AddSource(const MagneticFieldSource& source) {
    for (size_t i = 0; i < sources.size(); ++i) {
        if (!sources[i].active) {
            sources[i] = {source, true};
            return i;
        }
    }
    sources.push_back({source, true});
    return sources.size() - 1;
}

void MagneticFieldSystem::RemoveSource(size_t index) {
    if (index < sources.size()) sources[index].active = false;
}

void MagneticFieldSystem::TrackBody(JPH::BodyID body) {
    if (std::find(trackedBodies.begin(), trackedBodies.end(), body) == trackedBodies.end())
        trackedBodies.push_back(body);
}

void MagneticFieldSystem::UntrackBody(JPH::BodyID body) {
    trackedBodies.erase(std::remove(trackedBodies.begin(), trackedBodies.end(), body),
                        trackedBodies.end());
}

void MagneticFieldSystem::Update(JPH::PhysicsSystem& physics) {
    JPH::BodyLockInterface const& lockInterface = physics.GetBodyLockInterface();

    for (JPH::BodyID id : trackedBodies) {
        JPH::BodyLockWrite lock(lockInterface, id);
        if (!lock.Succeeded()) continue;
        JPH::Body& body = lock.GetBody();
        if (!body.IsActive()) continue;

        const JPH::Vec3 pos = JPH::Vec3(body.GetPosition());
        JPH::Vec3 totalForce = JPH::Vec3::sZero();

        for (const StoredSource& stored : sources) {
            if (!stored.active) continue;
            const MagneticFieldSource& src = stored.source;
            const JPH::Vec3 toBody = pos - ToJolt(src.position);
            const float dist = toBody.Length();
            if (dist > src.radius || dist < 1.0e-6f) continue;

            const float clampedDist = std::max(dist, src.minDistance);
            const float magnitude = src.strength / (clampedDist * clampedDist);
            const JPH::Vec3 direction = toBody / dist; // kaynaktan disari
            totalForce += (src.repel ? direction : -direction) * magnitude;
        }

        if (totalForce.LengthSq() > 0.0f) body.AddForce(totalForce);
    }
}

} // namespace alazforge
