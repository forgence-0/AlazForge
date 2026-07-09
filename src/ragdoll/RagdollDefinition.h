#pragma once
// Ragdoll iskelet tanımı (Faz B.6). AlazEngine'in gerçek animasyon/iskelet
// formatını bu repodan bilmiyoruz (ECS bridge'deki gibi dürüst bir sınır) —
// kemik listesi çağıran tarafından, oyunun kendi iskelet verisinden üretilerek
// sağlanır.

#include "core/AlazMath.h"

#include <string>
#include <vector>

namespace alazforge {

struct RagdollBoneConfig {
    std::string name;
    int parentIndex = -1; // -1 = kök
    float radius = 0.15f; // ilk sürüm: kapsül gövde
    float halfHeight = 0.2f;
    Vec3 localPosition;
    Quat localRotation;
    float twistMinDeg = -30.0f, twistMaxDeg = 30.0f;
    float swingMaxDeg = 45.0f;
};

struct RagdollSkeletonConfig {
    std::vector<RagdollBoneConfig> bones;
};

} // namespace alazforge
