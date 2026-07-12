#include "ragdoll/RagdollSkeleton.h"

#include "core/JoltAdapter.h"

#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Skeleton/Skeleton.h>

namespace alazforge {

RagdollSkeleton::RagdollSkeleton(const RagdollSkeletonConfig& inConfig)
    : boneCount(static_cast<int>(inConfig.bones.size())) {
    JPH::Ref<JPH::Skeleton> skeleton = new JPH::Skeleton();
    for (const RagdollBoneConfig& bone : inConfig.bones) {
        const char* parentName =
            bone.parentIndex >= 0 ? inConfig.bones[bone.parentIndex].name.c_str() : nullptr;
        skeleton->AddJoint(bone.name.c_str(), parentName);
    }
    skeleton->CalculateParentJointIndices();

    settings = new JPH::RagdollSettings();
    settings->mSkeleton = skeleton;
    settings->mParts.resize(inConfig.bones.size());

    for (size_t i = 0; i < inConfig.bones.size(); ++i) {
        const RagdollBoneConfig& bone = inConfig.bones[i];
        JPH::RagdollSettings::Part& part = settings->mParts[i];

        JPH::CapsuleShapeSettings capsuleSettings(bone.halfHeight, bone.radius);
        capsuleSettings.SetEmbedded();
        part.SetShape(capsuleSettings.Create().Get());
        part.mPosition = ToJoltR(bone.localPosition);
        part.mRotation = ToJolt(bone.localRotation);
        part.mMotionType = JPH::EMotionType::Dynamic;
        // mObjectLayer, RagdollInstance::Activate() sırasında çağırana özgü
        // katmana göre üzerine yazılır (paylaşılan settings, aktivasyon
        // anında hedef katmana göre kısaca değiştirilir).

        if (bone.parentIndex >= 0) {
            auto* twist = new JPH::SwingTwistConstraintSettings();
            twist->mPosition1 = twist->mPosition2 = part.mPosition;
            twist->mTwistAxis1 = twist->mTwistAxis2 = JPH::Vec3::sAxisY();
            twist->mPlaneAxis1 = twist->mPlaneAxis2 = JPH::Vec3::sAxisX();
            twist->mTwistMinAngle = JPH::DegreesToRadians(bone.twistMinDeg);
            twist->mTwistMaxAngle = JPH::DegreesToRadians(bone.twistMaxDeg);
            twist->mNormalHalfConeAngle = JPH::DegreesToRadians(bone.swingMaxDeg);
            twist->mPlaneHalfConeAngle = JPH::DegreesToRadians(bone.swingMaxDeg);
            part.mToParent = twist;
        }
    }

    settings->Stabilize();
    settings->DisableParentChildCollisions();
    settings->CalculateBodyIndexToConstraintIndex();
}

} // namespace alazforge
