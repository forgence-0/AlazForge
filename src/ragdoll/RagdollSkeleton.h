#pragma once
// Yeniden kullanılabilir ragdoll iskeleti (Faz B.6): kemik hiyerarşisinden
// bir kez şekil (capsule) listesi inşa eder, birden çok RagdollInstance
// arasında paylaşılabilir.
//
// NOT: Jolt'un hazır JPH::Ragdoll/RagdollSettings/Skeleton sistemi (mParts +
// CreateRagdoll) bu ortamda gerçek Jolt header'larına karşı doğrulanamayan
// çok sayıda API noktası içeriyordu ve CI'da segfault'a yol açtı. Bunun
// yerine, bu oturumda TurretMount'ta kanıtlanmış çalışan düşük seviye desen
// kullanılıyor: RagdollSkeleton yalnızca kemik tanımlarını + önceden
// oluşturulmuş capsule shape'leri tutar; gerçek gövde/constraint kurulumu
// RagdollInstance::Activate() içinde CreateAndAddBody + SwingTwistConstraint
// ile yapılır (VehicleSystem/TurretMount ile aynı, doğrulanmış desen).

#include "ragdoll/RagdollDefinition.h"

#include <ragdoll/export.h>

#include <Jolt/Jolt.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>

#include <vector>

namespace alazforge {

class RAGDOLL_API RagdollSkeleton {
public:
    explicit RagdollSkeleton(const RagdollSkeletonConfig& inConfig);

    int BoneCount() const { return static_cast<int>(config.bones.size()); }
    const RagdollSkeletonConfig& Config() const { return config; }
    JPH::Shape* ShapeAt(int boneIndex) const { return shapes[static_cast<size_t>(boneIndex)]; }

private:
    RagdollSkeletonConfig config;
    std::vector<JPH::Ref<JPH::Shape>> shapes;
};

} // namespace alazforge
