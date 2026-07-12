#include "zones/FrictionZoneSystem.h"

#include "core/JoltAdapter.h"

#include <Jolt/Physics/Body/BodyInterface.h>

#include <algorithm>

namespace alazforge {

namespace {

bool Contains(const FrictionZone& z, const Vec3& p) {
    return p.x >= z.boundsMin.x && p.x <= z.boundsMax.x && p.y >= z.boundsMin.y &&
           p.y <= z.boundsMax.y && p.z >= z.boundsMin.z && p.z <= z.boundsMax.z;
}

} // namespace

size_t FrictionZoneSystem::AddZone(const FrictionZone& zone) {
    for (size_t i = 0; i < zones.size(); ++i) {
        if (!zones[i].active) {
            zones[i] = {zone, true};
            return i;
        }
    }
    zones.push_back({zone, true});
    return zones.size() - 1;
}

void FrictionZoneSystem::RemoveZone(size_t index) {
    if (index < zones.size()) zones[index].active = false;
}

void FrictionZoneSystem::TrackBody(JPH::PhysicsSystem& physics, JPH::BodyID body) {
    for (const TrackedBody& t : tracked)
        if (t.body == body) return;
    JPH::BodyInterface& bi = physics.GetBodyInterface();
    TrackedBody t;
    t.body = body;
    t.originalFriction = bi.GetFriction(body);
    t.originalRestitution = bi.GetRestitution(body);
    tracked.push_back(t);
}

void FrictionZoneSystem::UntrackBody(JPH::PhysicsSystem& physics, JPH::BodyID body) {
    JPH::BodyInterface& bi = physics.GetBodyInterface();
    for (auto it = tracked.begin(); it != tracked.end(); ++it) {
        if (it->body == body) {
            if (it->inZone) {
                bi.SetFriction(body, it->originalFriction);
                bi.SetRestitution(body, it->originalRestitution);
            }
            tracked.erase(it);
            return;
        }
    }
}

const FrictionZone* FrictionZoneSystem::FindZoneContaining(const Vec3& worldPos) const {
    for (const StoredZone& sz : zones) {
        if (sz.active && Contains(sz.zone, worldPos)) return &sz.zone;
    }
    return nullptr;
}

void FrictionZoneSystem::Update(JPH::PhysicsSystem& physics) {
    JPH::BodyInterface& bi = physics.GetBodyInterface();
    for (TrackedBody& t : tracked) {
        const JPH::RVec3 posJ = bi.GetPosition(t.body);
        const Vec3 pos = FromJolt(JPH::Vec3(posJ));
        const FrictionZone* zone = FindZoneContaining(pos);
        if (zone) {
            bi.SetFriction(t.body, zone->friction);
            if (zone->restitution >= 0.0f) bi.SetRestitution(t.body, zone->restitution);
            t.inZone = true;
        } else if (t.inZone) {
            bi.SetFriction(t.body, t.originalFriction);
            bi.SetRestitution(t.body, t.originalRestitution);
            t.inZone = false;
        }
    }
}

float FrictionZoneSystem::QueryFrictionAt(const Vec3& worldPos, float defaultFriction) const {
    const FrictionZone* zone = FindZoneContaining(worldPos);
    return zone ? zone->friction : defaultFriction;
}

} // namespace alazforge
