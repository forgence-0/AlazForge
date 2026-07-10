#include "ragdoll/RagdollSkeleton.h"

#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>

namespace alazforge {

RagdollSkeleton::RagdollSkeleton(const RagdollSkeletonConfig& inConfig) : config(inConfig) {
    shapes.reserve(config.bones.size());
    for (const RagdollBoneConfig& bone : config.bones) {
        JPH::CapsuleShapeSettings capsuleSettings(bone.halfHeight, bone.radius);
        capsuleSettings.SetEmbedded();
        shapes.push_back(capsuleSettings.Create().Get());
    }
}

} // namespace alazforge
