#include "weapons/ExplosionSystem.h"

#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseQuery.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/RayCast.h>

#include <cmath>

namespace alazforge {

namespace {

float ComputeFalloff(ExplosionFalloff model, float distance, float radius) {
    // radius<=0: distance/radius NaN olurdu -- pratikte CollideSphere bu
    // yaricapta govde donmez, ama savunmaci olarak sifir doner.
    if (radius <= 0.0f) return 0.0f;
    const float linear = 1.0f - distance / radius;
    switch (model) {
        case ExplosionFalloff::InverseSquare:
            // Blast basinci mesafeyle cok hizli duser; (1-d/R)^2, [0,1]
            // araliginda kalan, kenarda ~0'a yumusakca inen bir yaklasim.
            return linear * linear;
        case ExplosionFalloff::Linear:
        default:
            return linear;
    }
}

// Merkezden hedefin kutle merkezine raycast: ilk carpilan govde hedefin
// kendisi degilse hedef siperdedir.
bool IsOccluded(JPH::PhysicsSystem& physics, const JPH::Vec3& center, const JPH::RVec3& targetCom,
                JPH::BodyID target) {
    const JPH::Vec3 delta = JPH::Vec3(targetCom) - center;
    if (delta.LengthSq() < 1.0e-8f) return false; // merkezle cakisiyor

    JPH::RRayCast ray(JPH::RVec3(center), delta);
    JPH::RayCastResult hit;
    if (!physics.GetNarrowPhaseQuery().CastRay(ray, hit)) return false; // hicbir sey yok
    return hit.mBodyID != target;
}

} // namespace

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

        ExplosionHit hit;
        hit.body = id;
        hit.distance = distance;
        hit.impulse = Vec3{0, 0, 0};

        // Siper kontrolu: arada baska govde varsa patlama bu hedefe ulasmaz
        if (config.losOcclusion && IsOccluded(physics, centerJ, comPos, id)) {
            hit.occluded = true;
            hit.falloff = 0.0f;
            outHits.push_back(hit);
            continue;
        }

        hit.falloff = ComputeFalloff(config.falloff, distance, config.radius);

        if (isDynamic) {
            // Tam merkezde yon tanimsiz -- yukari it
            JPH::Vec3 dir = distance > 1.0e-4f ? delta / distance : JPH::Vec3::sAxisY();
            dir = (dir + JPH::Vec3(0, config.upwardBias, 0)).Normalized();
            const JPH::Vec3 impulse = dir * (config.baseImpulseNs * hit.falloff);
            bi.AddImpulse(id, impulse);
            bi.ActivateBody(id);
            hit.impulse = FromJolt(impulse);
        }
        outHits.push_back(hit);
    }
}

} // namespace alazforge
