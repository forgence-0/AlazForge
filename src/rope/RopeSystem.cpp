#include "rope/RopeSystem.h"

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Constraints/PointConstraint.h>

#include <algorithm>
#include <cmath>

namespace alazforge {

namespace {

// Iki govde arasinda dunya-uzayli top eklem kurar (kod tabaninin
// dogrulanmis body-lock + settings.Create deseni).
JPH::Ref<JPH::Constraint> CreateBallJoint(JPH::PhysicsSystem& physics, JPH::BodyID a, JPH::BodyID b,
                                          const JPH::RVec3& worldPoint) {
    JPH::PointConstraintSettings settings;
    settings.mSpace = JPH::EConstraintSpace::WorldSpace;
    settings.mPoint1 = settings.mPoint2 = worldPoint;

    JPH::Body* aPtr = nullptr;
    {
        JPH::BodyLockWrite lock(physics.GetBodyLockInterface(), a);
        if (lock.Succeeded()) aPtr = &lock.GetBody();
    }
    JPH::Body* bPtr = nullptr;
    {
        JPH::BodyLockWrite lock(physics.GetBodyLockInterface(), b);
        if (lock.Succeeded()) bPtr = &lock.GetBody();
    }
    if (!aPtr || !bPtr) return nullptr;

    JPH::Ref<JPH::Constraint> constraint = settings.Create(*aPtr, *bPtr);
    physics.AddConstraint(constraint);
    return constraint;
}

} // namespace

RopeSystem::RopeSystem(const RopeConfig& inConfig) : config(inConfig) {}

RopeSystem::~RopeSystem() { Destroy(); }

void RopeSystem::Create(JPH::PhysicsSystem& inPhysics, const Vec3& startPos, const Vec3& endPos,
                        JPH::ObjectLayer layer) {
    Destroy();
    physics = &inPhysics;

    const JPH::RVec3 start = ToJoltR(startPos);
    const JPH::RVec3 end = ToJoltR(endPos);
    JPH::Vec3 dir = JPH::Vec3(end - start);
    const float span = dir.Length();
    dir = span > 1.0e-4f ? dir / span : JPH::Vec3::sAxisX();

    const int count = std::max(2, config.segmentCount);
    const float segLength = config.totalLength / static_cast<float>(count);
    const float halfCyl = std::max(0.005f, 0.5f * segLength - config.radius);
    const float segMass = std::max(0.01f, config.massPerMeter * segLength);

    // Kapsul yerel Y ekseni boyunca uzanir; segmenti halat yonune cevir
    const JPH::Quat segRot = JPH::Quat::sFromTo(JPH::Vec3::sAxisY(), dir);

    JPH::BodyInterface& bi = physics->GetBodyInterface();
    JPH::Ref<JPH::CapsuleShape> shape = new JPH::CapsuleShape(halfCyl, config.radius);

    segments.reserve(static_cast<size_t>(count));
    for (int i = 0; i < count; ++i) {
        const JPH::RVec3 center = start + dir * (segLength * (static_cast<float>(i) + 0.5f));
        JPH::BodyCreationSettings settings(shape, center, segRot, JPH::EMotionType::Dynamic, layer);
        settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
        settings.mMassPropertiesOverride.mMass = segMass;
        segments.push_back(bi.CreateAndAddBody(settings, JPH::EActivation::Activate));
    }

    // Ardisik segmentleri aralarindaki ek noktasindan bagla
    for (int i = 0; i + 1 < count; ++i) {
        const JPH::RVec3 joint = start + dir * (segLength * static_cast<float>(i + 1));
        JPH::Ref<JPH::Constraint> c = CreateBallJoint(*physics, segments[static_cast<size_t>(i)],
                                                      segments[static_cast<size_t>(i + 1)], joint);
        if (c) constraints.push_back(c);
    }
    segmentJointCount = constraints.size();
}

void RopeSystem::AttachStartTo(JPH::BodyID body, const Vec3& worldAnchor) {
    if (!segments.empty()) AttachTo(segments.front(), body, worldAnchor);
}

void RopeSystem::AttachEndTo(JPH::BodyID body, const Vec3& worldAnchor) {
    if (!segments.empty()) AttachTo(segments.back(), body, worldAnchor);
}

void RopeSystem::AttachTo(JPH::BodyID segment, JPH::BodyID target, const Vec3& worldAnchor) {
    if (!physics) return;
    JPH::Ref<JPH::Constraint> c = CreateBallJoint(*physics, target, segment, ToJoltR(worldAnchor));
    if (c) constraints.push_back(c);
}

void RopeSystem::Update(float dt, std::vector<int>* outBrokenJoints) {
    if (!physics || config.maxTensionN <= 0.0f || dt <= 0.0f) return;

    for (size_t i = 0; i < segmentJointCount; ++i) {
        JPH::Ref<JPH::Constraint>& c = constraints[i];
        if (!c) continue;
        auto* point = static_cast<JPH::PointConstraint*>(c.GetPtr());
        const float force = point->GetTotalLambdaPosition().Length() / dt;
        if (force > config.maxTensionN) {
            physics->RemoveConstraint(c);
            c = nullptr;
            if (outBrokenJoints) outBrokenJoints->push_back(static_cast<int>(i));
        }
    }
}

bool RopeSystem::IsJointBroken(int jointIndex) const {
    if (jointIndex < 0 || static_cast<size_t>(jointIndex) >= segmentJointCount) return true;
    return !constraints[static_cast<size_t>(jointIndex)];
}

JPH::BodyID RopeSystem::GetSegmentBodyID(int index) const {
    if (index < 0 || static_cast<size_t>(index) >= segments.size()) return JPH::BodyID();
    return segments[static_cast<size_t>(index)];
}

Transform RopeSystem::GetSegmentTransform(int index) const {
    if (!physics || index < 0 || static_cast<size_t>(index) >= segments.size()) return Transform{};
    JPH::BodyLockRead lock(physics->GetBodyLockInterface(), segments[static_cast<size_t>(index)]);
    if (!lock.Succeeded()) return Transform{};
    const JPH::Body& body = lock.GetBody();
    Transform t;
    t.position = FromJolt(JPH::Vec3(body.GetPosition()));
    t.rotation = FromJolt(body.GetRotation());
    return t;
}

void RopeSystem::Destroy() {
    if (!physics) {
        segments.clear();
        constraints.clear();
        return;
    }
    JPH::BodyInterface& bi = physics->GetBodyInterface();
    for (JPH::Ref<JPH::Constraint>& c : constraints)
        if (c) physics->RemoveConstraint(c);
    constraints.clear();
    segmentJointCount = 0;
    for (JPH::BodyID id : segments) {
        if (id.IsInvalid()) continue;
        bi.RemoveBody(id);
        bi.DestroyBody(id);
    }
    segments.clear();
    physics = nullptr;
}

} // namespace alazforge
