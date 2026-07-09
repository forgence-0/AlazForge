#pragma once
// Yeniden kullanılabilir ragdoll iskeleti (Faz B.6): kemik hiyerarşisinden bir
// kez JPH::RagdollSettings inşa eder, birden çok RagdollInstance arasında
// paylaşılabilir. VehicleSystem'in VehicleConstraint'i sarmalama deseniyle
// tutarlı — Jolt'un hazır ragdoll sistemi kullanılır, sıfırdan constraint
// zinciri yazılmaz.
//
// NOT: Jolt submodule bu ortamda checkout edilmediği için RagdollSettings'in
// tam programatik kuruluşu (mParts dizisi + mToParent constraint'leri,
// Stabilize()/DisableParentChildCollisions() gerekliliği, Skeleton sınıfının
// tam header konumu) gerçek header'lara karşı doğrulanamadı — bilinen genel
// Jolt Ragdoll API şekli kullanıldı, ilk derlemede teyit edilmeli.

#include "ragdoll/RagdollDefinition.h"

#include <alazforge/export.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>

namespace alazforge {

class ALAZFORGE_API RagdollSkeleton {
public:
    explicit RagdollSkeleton(const RagdollSkeletonConfig& inConfig);

    int BoneCount() const { return boneCount; }
    JPH::RagdollSettings* Settings() const { return settings.GetPtr(); }

private:
    JPH::Ref<JPH::RagdollSettings> settings;
    int boneCount = 0;
};

} // namespace alazforge
