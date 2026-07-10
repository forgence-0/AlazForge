#include "sportsball/BallPhysicsSystem.h"

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>

#include <algorithm>
#include <cmath>

namespace alazforge {

namespace {
constexpr float kPi = 3.14159265358979323846f;
constexpr float kMinSpeedForForces = 1.0e-4f; // sıfıra yakın hızlarda yön belirsiz, kuvvet atla
} // namespace

JPH::BodyID BallPhysicsSystem::CreateBall(JPH::PhysicsSystem& physics, const BallConfig& config,
                                          const Vec3& worldPosition, const Quat& worldRotation,
                                          JPH::ObjectLayer layer) {
    JPH::SphereShapeSettings sphereSettings(config.radius);
    sphereSettings.SetEmbedded();

    JPH::RefConst<JPH::Shape> shape;
    if (config.elongation != 1.0f) {
        // elongation>1: kurenin +z (yerel ileri) ekseninde uzatilmasi --
        // prolate spheroid, Amerikan futbolu topu gibi.
        JPH::ScaledShapeSettings scaledSettings(sphereSettings.Create().Get(),
                                                JPH::Vec3(1.0f, 1.0f, config.elongation));
        scaledSettings.SetEmbedded();
        shape = scaledSettings.Create().Get();
    } else {
        shape = sphereSettings.Create().Get();
    }

    JPH::BodyCreationSettings bodySettings(shape, ToJoltR(worldPosition), ToJolt(worldRotation),
                                           JPH::EMotionType::Dynamic, layer);
    bodySettings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    bodySettings.mMassPropertiesOverride.mMass = config.mass;
    bodySettings.mRestitution = config.restitution;
    bodySettings.mFriction = config.friction;

    JPH::BodyInterface& bi = physics.GetBodyInterface();
    JPH::BodyID body = bi.CreateAndAddBody(bodySettings, JPH::EActivation::Activate);

    balls.push_back({body, config});
    return body;
}

void BallPhysicsSystem::UntrackBall(JPH::BodyID body) {
    balls.erase(std::remove_if(balls.begin(), balls.end(),
                               [body](const TrackedBall& b) { return b.body == body; }),
                balls.end());
}

void BallPhysicsSystem::Update(JPH::PhysicsSystem& physics, float dt) {
    (void)dt; // Kuvvetler anlık uygulanır (Jolt entegrasyonu kendi dt'sini kullanır)
    JPH::BodyInterface& bi = physics.GetBodyInterface();

    for (const TrackedBall& tracked : balls) {
        if (!bi.IsActive(tracked.body)) continue;

        const JPH::Vec3 v = bi.GetLinearVelocity(tracked.body);
        const float speed = v.Length();
        if (speed < kMinSpeedForForces) continue;

        const BallConfig& cfg = tracked.config;
        const float crossSectionArea = kPi * cfg.radius * cfg.radius;

        // Surukleme: hizin tersi yonde, hizin karesiyle buyur.
        const float dragMag =
            0.5f * cfg.airDensity * cfg.dragCoefficient * crossSectionArea * speed * speed;
        const JPH::Vec3 dragForce = -(v / speed) * dragMag;

        // Magnus: spin (w) ve lineer hiz (v) eksenine dik kaldirma kuvveti.
        JPH::Vec3 magnusForce = JPH::Vec3::sZero();
        if (cfg.magnusCoefficient != 0.0f) {
            const JPH::Vec3 w = bi.GetAngularVelocity(tracked.body);
            magnusForce = w.Cross(v) *
                          (cfg.magnusCoefficient * cfg.airDensity * crossSectionArea * cfg.radius);
        }

        bi.AddForce(tracked.body, dragForce + magnusForce);
    }
}

void BallPhysicsSystem::RemoveBall(JPH::PhysicsSystem& physics, JPH::BodyID body) {
    UntrackBall(body);
    JPH::BodyInterface& bi = physics.GetBodyInterface();
    bi.RemoveBody(body);
    bi.DestroyBody(body);
}

} // namespace alazforge
