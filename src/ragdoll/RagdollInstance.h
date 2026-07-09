#pragma once
// Ragdoll örneği (Faz B.6): bir RagdollSkeleton'dan, ölüm anında tek seferlik
// canlandırılan fizik örneği. Son bilinen dünya pozu/hızıyla aktive edilir
// (momentum sürekliliği için — karakter koşarken ölürse ragdoll o hızla
// devam eder).

#include "core/AlazMath.h"

#include <physics_ext/export.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>

#include <vector>

namespace alazforge {

class RagdollSkeleton;

class PHYSICS_EXT_API RagdollInstance {
public:
    RagdollInstance() = default;
    ~RagdollInstance();

    RagdollInstance(const RagdollInstance&) = delete;
    RagdollInstance& operator=(const RagdollInstance&) = delete;

    void Activate(JPH::PhysicsSystem& inPhysics, const RagdollSkeleton& skeleton,
                  const Transform& worldPose, JPH::ObjectLayer layer,
                  const std::vector<Vec3>* initialBoneVelocities = nullptr);

    Transform GetBoneWorldTransform(int boneIndex) const;
    int BoneCount() const;
    bool IsActive() const { return ragdoll != nullptr; }

    void Destroy();

private:
    JPH::PhysicsSystem* physics = nullptr;
    JPH::Ref<JPH::Ragdoll> ragdoll;
};

} // namespace alazforge
