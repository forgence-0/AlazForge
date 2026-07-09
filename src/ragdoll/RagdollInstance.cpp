#include "ragdoll/RagdollInstance.h"

#include "core/JoltAdapter.h"
#include "ragdoll/RagdollSkeleton.h"

#include <Jolt/Physics/Body/BodyLock.h>

namespace alazforge {

RagdollInstance::~RagdollInstance() { Destroy(); }

void RagdollInstance::Activate(JPH::PhysicsSystem& inPhysics, const RagdollSkeleton& skeleton,
                               const Transform& worldPose, JPH::ObjectLayer layer,
                               const std::vector<Vec3>* initialBoneVelocities) {
    Destroy();
    physics = &inPhysics;

    JPH::RagdollSettings* settings = skeleton.Settings();
    for (JPH::RagdollSettings::Part& part : settings->mParts)
        part.mObjectLayer = layer;

    ragdoll = settings->CreateRagdoll(/*inCollisionGroup=*/0, /*inUserData=*/0, physics);

    // Her gövdeyi, iskeletin dinlenme pozundaki (ragdoll orijinine göre)
    // konum/rotasyonunu worldPose ile dönüştürerek yerleştir — yalnızca kök
    // değil TÜM gövdeler; aksi halde constraint'ler ilk adımda şiddetli bir
    // "snap" yaşar (kök dünyaya ışınlanır, diğerleri eski yerinde kalır).
    JPH::BodyInterface& bi = physics->GetBodyInterface();
    const JPH::Quat worldRot = ToJolt(worldPose.rotation);
    const int bodyCount = ragdoll->GetBodyCount();
    for (int i = 0; i < bodyCount; ++i) {
        const JPH::RagdollSettings::Part& part = settings->mParts[i];
        const JPH::Vec3 restPos(part.mPosition);
        const JPH::RVec3 newWorldPos = ToJoltR(worldPose.position) + worldRot * restPos;
        const JPH::Quat newWorldRot = worldRot * part.mRotation;
        bi.SetPositionAndRotation(ragdoll->GetBodyID(i), newWorldPos, newWorldRot,
                                  JPH::EActivation::DontActivate);
        if (initialBoneVelocities && i < static_cast<int>(initialBoneVelocities->size()))
            bi.SetLinearVelocity(ragdoll->GetBodyID(i), ToJolt((*initialBoneVelocities)[i]));
    }

    ragdoll->AddToPhysicsSystem(JPH::EActivation::Activate);
}

Transform RagdollInstance::GetBoneWorldTransform(int boneIndex) const {
    if (!ragdoll || !physics) return Transform{};

    const JPH::BodyID id = ragdoll->GetBodyID(boneIndex);
    JPH::BodyLockRead lock(physics->GetBodyLockInterface(), id);
    if (!lock.Succeeded()) return Transform{};

    const JPH::Body& body = lock.GetBody();
    Transform t;
    t.position = FromJolt(JPH::Vec3(body.GetPosition()));
    t.rotation = FromJolt(body.GetRotation());
    return t;
}

int RagdollInstance::BoneCount() const { return ragdoll ? ragdoll->GetBodyCount() : 0; }

void RagdollInstance::Destroy() {
    if (ragdoll) {
        ragdoll->RemoveFromPhysicsSystem();
        ragdoll = nullptr;
    }
    physics = nullptr;
}

} // namespace alazforge
