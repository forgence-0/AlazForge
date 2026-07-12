#include "joints/SpringDamperJoint.h"

#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Constraints/DistanceConstraint.h>

#include <cmath>

namespace alazforge {

SpringDamperJoint::SpringDamperJoint(const SpringDamperJointConfig& inConfig) : config(inConfig) {}

SpringDamperJoint::~SpringDamperJoint() { Destroy(); }

void SpringDamperJoint::Create(JPH::PhysicsSystem& inPhysics, JPH::BodyID a, JPH::BodyID b,
                               const Vec3& worldAnchorA, const Vec3& worldAnchorB) {
    Destroy();
    physics = &inPhysics;
    bodyA = a;
    bodyB = b;

    const JPH::RVec3 anchorA = ToJoltR(worldAnchorA);
    const JPH::RVec3 anchorB = ToJoltR(worldAnchorB);
    const float rest =
        config.restLength >= 0.0f ? config.restLength : JPH::Vec3(anchorB - anchorA).Length();

    JPH::DistanceConstraintSettings settings;
    settings.mSpace = JPH::EConstraintSpace::WorldSpace;
    settings.mPoint1 = anchorA;
    settings.mPoint2 = anchorB;
    settings.mMinDistance = rest;
    settings.mMaxDistance = rest;
    settings.mLimitsSpringSettings.mMode = JPH::ESpringMode::FrequencyAndDamping;
    settings.mLimitsSpringSettings.mFrequency = config.frequencyHz;
    settings.mLimitsSpringSettings.mDamping = config.damping;

    JPH::Body* aPtr = nullptr;
    {
        JPH::BodyLockWrite lock(physics->GetBodyLockInterface(), a);
        if (lock.Succeeded()) aPtr = &lock.GetBody();
    }
    JPH::Body* bPtr = nullptr;
    {
        JPH::BodyLockWrite lock(physics->GetBodyLockInterface(), b);
        if (lock.Succeeded()) bPtr = &lock.GetBody();
    }
    if (!aPtr || !bPtr) return;

    // Cagirandan sonra CurrentLength() gorece capalari izleyebilsin diye
    // her govdenin kendi yerel uzayindaki capa ofsetini sakla.
    localAnchorA = aPtr->GetRotation().InverseRotate(JPH::Vec3(anchorA - aPtr->GetPosition()));
    localAnchorB = bPtr->GetRotation().InverseRotate(JPH::Vec3(anchorB - bPtr->GetPosition()));

    constraint = settings.Create(*aPtr, *bPtr);
    physics->AddConstraint(constraint);
}

void SpringDamperJoint::Destroy() {
    if (physics && constraint) physics->RemoveConstraint(constraint);
    constraint = nullptr;
    physics = nullptr;
}

float SpringDamperJoint::CurrentLength() const {
    if (!physics || !constraint) return 0.0f;
    JPH::BodyLockRead lockA(physics->GetBodyLockInterface(), bodyA);
    JPH::BodyLockRead lockB(physics->GetBodyLockInterface(), bodyB);
    if (!lockA.Succeeded() || !lockB.Succeeded()) return 0.0f;
    const JPH::RVec3 worldA =
        lockA.GetBody().GetPosition() + lockA.GetBody().GetRotation() * localAnchorA;
    const JPH::RVec3 worldB =
        lockB.GetBody().GetPosition() + lockB.GetBody().GetRotation() * localAnchorB;
    return JPH::Vec3(worldA - worldB).Length();
}

float SpringDamperJoint::CurrentTensionForce(float dt) const {
    if (!constraint || dt <= 0.0f) return 0.0f;
    auto* dc = static_cast<JPH::DistanceConstraint*>(constraint.GetPtr());
    return std::abs(dc->GetTotalLambdaPosition()) / dt;
}

} // namespace alazforge
