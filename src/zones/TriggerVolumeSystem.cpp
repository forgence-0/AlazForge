#include "zones/TriggerVolumeSystem.h"

#include "core/JoltAdapter.h"

#include <Jolt/Physics/Body/BodyLock.h>

#include <algorithm>

namespace alazforge {

namespace {

bool Contains(const TriggerVolume& v, const Vec3& p) {
    return p.x >= v.boundsMin.x && p.x <= v.boundsMax.x && p.y >= v.boundsMin.y &&
           p.y <= v.boundsMax.y && p.z >= v.boundsMin.z && p.z <= v.boundsMax.z;
}

} // namespace

size_t TriggerVolumeSystem::AddVolume(const TriggerVolume& volume) {
    volumes.push_back({volume, true});
    const size_t index = volumes.size() - 1;
    for (TrackedBody& t : tracked)
        t.insideVolume.resize(volumes.size(), false);
    return index;
}

void TriggerVolumeSystem::RemoveVolume(size_t index) {
    if (index < volumes.size()) volumes[index].active = false;
}

void TriggerVolumeSystem::TrackBody(JPH::BodyID body) {
    for (const TrackedBody& t : tracked)
        if (t.body == body) return;
    tracked.push_back({body, std::vector<bool>(volumes.size(), false)});
}

void TriggerVolumeSystem::UntrackBody(JPH::BodyID body) {
    tracked.erase(std::remove_if(tracked.begin(), tracked.end(),
                                 [&](const TrackedBody& t) { return t.body == body; }),
                  tracked.end());
}

void TriggerVolumeSystem::Update(JPH::PhysicsSystem& physics,
                                 std::vector<TriggerEvent>& outEvents) {
    for (TrackedBody& t : tracked) {
        if (t.insideVolume.size() != volumes.size()) t.insideVolume.resize(volumes.size(), false);

        JPH::BodyLockRead lock(physics.GetBodyLockInterface(), t.body);
        if (!lock.Succeeded()) continue;
        const Vec3 pos = FromJolt(JPH::Vec3(lock.GetBody().GetPosition()));

        for (size_t i = 0; i < volumes.size(); ++i) {
            const StoredVolume& sv = volumes[i];
            const bool nowInside = sv.active && Contains(sv.volume, pos);
            if (nowInside != t.insideVolume[i]) {
                outEvents.push_back(
                    {i, sv.volume.userTag, t.body,
                     nowInside ? TriggerEventType::Entered : TriggerEventType::Exited});
                t.insideVolume[i] = nowInside;
            }
        }
    }
}

bool TriggerVolumeSystem::IsBodyInVolume(JPH::BodyID body, size_t volumeIndex) const {
    for (const TrackedBody& t : tracked)
        if (t.body == body)
            return volumeIndex < t.insideVolume.size() && t.insideVolume[volumeIndex];
    return false;
}

} // namespace alazforge
