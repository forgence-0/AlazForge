#include "lod/LODSystem.h"

#include "core/JoltAdapter.h"

#include <Jolt/Physics/Body/BodyInterface.h>

#include <cmath>

namespace alazforge {

LODSystem::LODSystem(const LODConfig& inConfig) : config(inConfig) {}

void LODSystem::TrackBody(JPH::BodyID body) {
    for (const TrackedBody& t : tracked)
        if (t.body == body) return;
    tracked.push_back({body, false});
}

void LODSystem::UntrackBody(JPH::PhysicsSystem& physics, JPH::BodyID body) {
    for (auto it = tracked.begin(); it != tracked.end(); ++it) {
        if (it->body == body) {
            if (it->forcedSleep) physics.GetBodyInterface().ActivateBody(body);
            tracked.erase(it);
            return;
        }
    }
}

void LODSystem::Update(JPH::PhysicsSystem& physics, const Vec3& referencePoint) {
    JPH::BodyInterface& bi = physics.GetBodyInterface();
    const JPH::RVec3 ref = ToJoltR(referencePoint);

    for (TrackedBody& t : tracked) {
        const JPH::RVec3 pos = bi.GetPosition(t.body);
        const JPH::Vec3 delta = JPH::Vec3(pos - ref);
        const float dist = delta.Length();

        if (!t.forcedSleep && dist > config.sleepDistance) {
            bi.DeactivateBody(t.body);
            t.forcedSleep = true;
        } else if (t.forcedSleep && dist < config.wakeDistance) {
            bi.ActivateBody(t.body);
            t.forcedSleep = false;
        }
    }
}

size_t LODSystem::SleepingCount() const {
    size_t count = 0;
    for (const TrackedBody& t : tracked)
        if (t.forcedSleep) ++count;
    return count;
}

} // namespace alazforge
