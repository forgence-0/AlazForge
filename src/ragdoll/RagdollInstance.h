#pragma once
// Ragdoll örneği (Faz B.6): bir RagdollSkeleton'dan, ölüm anında tek seferlik
// canlandırılan fizik örneği. Son bilinen dünya pozu/hızıyla aktive edilir
// (momentum sürekliliği için — karakter koşarken ölürse ragdoll o hızla
// devam eder).
//
// Düşük seviye desen: her kemik CreateAndAddBody ile gerçek bir dinamik
// gövde, ebeveyni olan kemiklere JPH::SwingTwistConstraint ile bağlanır
// (VehicleSystem/TurretMount ile aynı, doğrulanmış desen — bkz.
// RagdollSkeleton.h'deki not).

#include "core/AlazMath.h"

#include <ragdoll/export.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Constraints/Constraint.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <vector>

namespace alazforge {

class RagdollSkeleton;

class RAGDOLL_API RagdollInstance {
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
    bool IsActive() const { return !boneBodies.empty(); }

    void Destroy();

private:
    JPH::PhysicsSystem* physics = nullptr;
    std::vector<JPH::BodyID> boneBodies;
    std::vector<JPH::Ref<JPH::Constraint>> constraints;
};

} // namespace alazforge
