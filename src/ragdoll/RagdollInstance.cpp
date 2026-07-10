#include "ragdoll/RagdollInstance.h"

#include "core/JoltAdapter.h"
#include "ragdoll/RagdollDefinition.h"
#include "ragdoll/RagdollSkeleton.h"

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>

namespace alazforge {

RagdollInstance::~RagdollInstance() { Destroy(); }

void RagdollInstance::Activate(JPH::PhysicsSystem& inPhysics, const RagdollSkeleton& skeleton,
                               const Transform& worldPose, JPH::ObjectLayer layer,
                               const std::vector<Vec3>* initialBoneVelocities) {
    Destroy();
    physics = &inPhysics;

    const RagdollSkeletonConfig& config = skeleton.Config();
    const JPH::Quat worldRot = ToJolt(worldPose.rotation);
    const JPH::RVec3 worldPos = ToJoltR(worldPose.position);

    JPH::BodyInterface& bi = physics->GetBodyInterface();
    boneBodies.resize(config.bones.size());

    // ── Her kemik için gerçek bir dinamik gövde (hemen canlı, create+add
    // birlikte — sıralama belirsizliğini ortadan kaldırır) ──────────────
    for (size_t i = 0; i < config.bones.size(); ++i) {
        const RagdollBoneConfig& bone = config.bones[i];
        const JPH::RVec3 bonePos = worldPos + worldRot * ToJolt(bone.localPosition);
        const JPH::Quat boneRot = worldRot * ToJolt(bone.localRotation);

        JPH::BodyCreationSettings bodySettings(skeleton.ShapeAt(static_cast<int>(i)), bonePos,
                                               boneRot, JPH::EMotionType::Dynamic, layer);
        boneBodies[i] = bi.CreateAndAddBody(bodySettings, JPH::EActivation::Activate);

        if (initialBoneVelocities && i < initialBoneVelocities->size())
            bi.SetLinearVelocity(boneBodies[i], ToJolt((*initialBoneVelocities)[i]));
    }

    // ── Ebeveyn-çocuk eklemleri: SwingTwistConstraint, eklem noktası
    // çocuk kemiğin kendi pozisyonunda (bkz. RagdollDefinition.h) ───────
    for (size_t i = 0; i < config.bones.size(); ++i) {
        const RagdollBoneConfig& bone = config.bones[i];
        if (bone.parentIndex < 0) continue;

        const JPH::RVec3 jointPos = worldPos + worldRot * ToJolt(bone.localPosition);

        JPH::SwingTwistConstraintSettings twistSettings;
        twistSettings.mSpace = JPH::EConstraintSpace::WorldSpace;
        twistSettings.mPosition1 = twistSettings.mPosition2 = jointPos;
        twistSettings.mTwistAxis1 = twistSettings.mTwistAxis2 = worldRot * JPH::Vec3::sAxisY();
        twistSettings.mPlaneAxis1 = twistSettings.mPlaneAxis2 = worldRot * JPH::Vec3::sAxisX();
        twistSettings.mTwistMinAngle = JPH::DegreesToRadians(bone.twistMinDeg);
        twistSettings.mTwistMaxAngle = JPH::DegreesToRadians(bone.twistMaxDeg);
        twistSettings.mNormalHalfConeAngle = JPH::DegreesToRadians(bone.swingMaxDeg);
        twistSettings.mPlaneHalfConeAngle = JPH::DegreesToRadians(bone.swingMaxDeg);

        JPH::Body* parentPtr = nullptr;
        {
            JPH::BodyLockWrite lock(physics->GetBodyLockInterface(),
                                    boneBodies[static_cast<size_t>(bone.parentIndex)]);
            if (lock.Succeeded()) parentPtr = &lock.GetBody();
        }
        JPH::Body* childPtr = nullptr;
        {
            JPH::BodyLockWrite lock(physics->GetBodyLockInterface(), boneBodies[i]);
            if (lock.Succeeded()) childPtr = &lock.GetBody();
        }
        if (!parentPtr || !childPtr) continue;

        JPH::Ref<JPH::Constraint> constraint = twistSettings.Create(*parentPtr, *childPtr);
        physics->AddConstraint(constraint);
        constraints.push_back(constraint);
    }
}

Transform RagdollInstance::GetBoneWorldTransform(int boneIndex) const {
    if (!physics || boneIndex < 0 || static_cast<size_t>(boneIndex) >= boneBodies.size())
        return Transform{};

    JPH::BodyLockRead lock(physics->GetBodyLockInterface(),
                           boneBodies[static_cast<size_t>(boneIndex)]);
    if (!lock.Succeeded()) return Transform{};

    const JPH::Body& body = lock.GetBody();
    Transform t;
    t.position = FromJolt(JPH::Vec3(body.GetPosition()));
    t.rotation = FromJolt(body.GetRotation());
    return t;
}

int RagdollInstance::BoneCount() const { return static_cast<int>(boneBodies.size()); }

void RagdollInstance::Destroy() {
    if (!physics) {
        boneBodies.clear();
        constraints.clear();
        return;
    }
    JPH::BodyInterface& bi = physics->GetBodyInterface();

    for (JPH::Ref<JPH::Constraint>& c : constraints)
        physics->RemoveConstraint(c);
    constraints.clear();

    for (JPH::BodyID id : boneBodies) {
        if (id.IsInvalid()) continue;
        bi.RemoveBody(id);
        bi.DestroyBody(id);
    }
    boneBodies.clear();
    physics = nullptr;
}

} // namespace alazforge
