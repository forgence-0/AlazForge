#include "scene/QueryHelpers.h"

#include "core/JoltAdapter.h"

#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/ShapeCast.h>

namespace alazforge {

RaycastHit RaycastClosest(JPH::PhysicsSystem& physics, const Vec3& origin, const Vec3& direction,
                          float maxDistance) {
    RaycastHit result;
    JPH::Vec3 dir = ToJolt(direction);
    const float len = dir.Length();
    if (len < 1.0e-6f || maxDistance <= 0.0f) return result;
    dir = dir / len * maxDistance;

    const JPH::RRayCast ray(ToJoltR(origin), dir);
    JPH::RayCastResult hit;
    if (!physics.GetNarrowPhaseQuery().CastRay(ray, hit)) return result;

    result.hit = true;
    result.body = hit.mBodyID;
    result.fraction = hit.mFraction;
    const JPH::RVec3 worldPoint = ray.GetPointOnRay(hit.mFraction);
    result.point = FromJolt(worldPoint);

    JPH::BodyLockRead lock(physics.GetBodyLockInterface(), hit.mBodyID);
    if (lock.Succeeded())
        result.normal =
            FromJolt(lock.GetBody().GetWorldSpaceSurfaceNormal(hit.mSubShapeID2, worldPoint));

    return result;
}

std::vector<JPH::BodyID> OverlapSphere(JPH::PhysicsSystem& physics, const Vec3& center,
                                       float radius) {
    const JPH::SphereShape shape(radius);
    const JPH::RVec3 c = ToJoltR(center);

    JPH::AllHitCollisionCollector<JPH::CollideShapeCollector> collector;
    physics.GetNarrowPhaseQuery().CollideShape(&shape, JPH::Vec3::sReplicate(1.0f),
                                               JPH::RMat44::sTranslation(c),
                                               JPH::CollideShapeSettings(), c, collector);

    std::vector<JPH::BodyID> results;
    results.reserve(collector.mHits.size());
    for (const auto& hitResult : collector.mHits)
        results.push_back(hitResult.mBodyID2);
    return results;
}

std::vector<JPH::BodyID> OverlapBox(JPH::PhysicsSystem& physics, const Vec3& center,
                                    const Vec3& halfExtents) {
    const JPH::BoxShape shape(ToJolt(halfExtents));
    const JPH::RVec3 c = ToJoltR(center);

    JPH::AllHitCollisionCollector<JPH::CollideShapeCollector> collector;
    physics.GetNarrowPhaseQuery().CollideShape(&shape, JPH::Vec3::sReplicate(1.0f),
                                               JPH::RMat44::sTranslation(c),
                                               JPH::CollideShapeSettings(), c, collector);

    std::vector<JPH::BodyID> results;
    results.reserve(collector.mHits.size());
    for (const auto& hitResult : collector.mHits)
        results.push_back(hitResult.mBodyID2);
    return results;
}

bool SweepSphere(JPH::PhysicsSystem& physics, const Vec3& start, const Vec3& direction,
                 float maxDistance, float radius, RaycastHit& outHit) {
    outHit = RaycastHit{};
    JPH::Vec3 dir = ToJolt(direction);
    const float len = dir.Length();
    if (len < 1.0e-6f || maxDistance <= 0.0f) return false;
    dir = dir / len * maxDistance;

    const JPH::SphereShape shape(radius);
    const JPH::RVec3 startPos = ToJoltR(start);
    const JPH::RShapeCast cast(&shape, JPH::Vec3::sReplicate(1.0f),
                               JPH::RMat44::sTranslation(startPos), dir);

    JPH::ClosestHitCollisionCollector<JPH::CastShapeCollector> collector;
    physics.GetNarrowPhaseQuery().CastShape(cast, JPH::ShapeCastSettings(), startPos, collector);
    if (!collector.HadHit()) return false;

    const auto& hit = collector.mHit;
    outHit.hit = true;
    outHit.body = hit.mBodyID2;
    outHit.fraction = hit.mFraction;
    // mContactPointOn2, CastShape'e gecirilen inBaseOffset'e (burada
    // startPos) GORE -- mutlak dunya konumuna donmek icin geri eklenir.
    outHit.point = FromJolt(JPH::RVec3(startPos + hit.mContactPointOn2));
    const JPH::Vec3 axis = hit.mPenetrationAxis;
    outHit.normal = axis.LengthSq() > 1.0e-12f ? FromJolt(-axis.Normalized()) : Vec3{0, 1, 0};
    return true;
}

} // namespace alazforge
