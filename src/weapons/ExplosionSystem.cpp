#include "weapons/ExplosionSystem.h"

#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseQuery.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>

#include <cmath>

namespace alazforge {

void ExplosionSystem::Detonate(JPH::PhysicsSystem& physics, const Vec3& center,
                               const ExplosionConfig& config, std::vector<ExplosionHit>& outHits) {
    outHits.clear();
    const JPH::Vec3 centerJ = ToJolt(center);

    // Yaricap icindeki gövde adaylarini topla (broad phase AABB testi --
    // kucuk bir fazla-kapsama olabilir, asagida gercek mesafeyle elenir).
    JPH::AllHitCollisionCollector<JPH::CollideShapeBodyCollector> collector;
    physics.GetBroadPhaseQuery().CollideSphere(centerJ, config.radius, collector);

    JPH::BodyInterface& bi = physics.GetBodyInterface();
    for (const JPH::BodyID& id : collector.mHits) {
        JPH::RVec3 comPos;
        bool isDynamic = false;
        {
            JPH::BodyLockRead lock(physics.GetBodyLockInterface(), id);
            if (!lock.Succeeded()) continue;
            const JPH::Body& body = lock.GetBody();
            comPos = body.GetCenterOfMassPosition();
            isDynamic = body.IsDynamic();
        }

        const JPH::Vec3 delta = JPH::Vec3(comPos) - centerJ;
        const float distance = delta.Length();
        if (distance > config.radius) continue; // AABB fazla-kapsamasini ele

        const float falloff = 1.0f - distance / config.radius;

        ExplosionHit hit;
        hit.body = id;
        hit.distance = distance;
        hit.falloff = falloff;
        hit.impulse = Vec3{0, 0, 0};

        if (isDynamic) {
            // Tam merkezde yon tanimsiz -- yukari it
            JPH::Vec3 dir = distance > 1.0e-4f ? delta / distance : JPH::Vec3::sAxisY();
            dir = (dir + JPH::Vec3(0, config.upwardBias, 0)).Normalized();
            const JPH::Vec3 impulse = dir * (config.baseImpulseNs * falloff);
            bi.AddImpulse(id, impulse);
            bi.ActivateBody(id);
            hit.impulse = FromJolt(impulse);
        }
        outHits.push_back(hit);
    }
}

} // namespace alazforge
