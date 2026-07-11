#include "weapons/TrajectoryPredictor.h"

#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/RayCast.h>

namespace alazforge {

TrajectoryResult TrajectoryPredictor::Predict(JPH::PhysicsSystem* physics, const Vec3& origin,
                                              const Vec3& initialVelocity,
                                              const TrajectoryConfig& config) {
    TrajectoryResult result;

    JPH::RVec3 pos = ToJoltR(origin);
    JPH::Vec3 vel = ToJolt(initialVelocity);
    const JPH::Vec3 wind = ToJolt(config.wind);
    const float dt = config.timeStep;

    result.points.push_back(origin);

    float t = 0.0f;
    while (t < config.maxTime) {
        // BallisticsSystem ile ayni model: yercekimi + havaya-bagil surukleme
        vel += JPH::Vec3(0, -config.gravity, 0) * dt;
        const JPH::Vec3 airRelVel = vel - wind;
        vel -= airRelVel * (config.dragFactor * airRelVel.Length() * dt);

        const JPH::Vec3 segment = vel * dt;
        if (physics) {
            JPH::RRayCast ray{pos, segment};
            JPH::RayCastResult hit;
            if (physics->GetNarrowPhaseQuery().CastRay(ray, hit)) {
                const JPH::RVec3 hitPos = ray.GetPointOnRay(hit.mFraction);
                result.hitSomething = true;
                result.hitPoint = FromJolt(JPH::Vec3(hitPos));
                result.hitBody = hit.mBodyID;
                result.flightTime = t + hit.mFraction * dt;
                result.points.push_back(result.hitPoint);
                return result;
            }
        }
        pos += segment;
        t += dt;
        result.points.push_back(FromJolt(JPH::Vec3(pos)));
    }
    result.flightTime = t;
    return result;
}

} // namespace alazforge
