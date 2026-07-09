// Faz B.6 doğrulama testi: RagdollSkeleton + RagdollInstance.
// 2 kemikli minimal iskelet (pelvis + 1 uzuv). Aktivasyon sonrası yerçekimiyle
// düşüp yerleşmeleri ve eklem mesafesinin (SwingTwistConstraint limitleri
// içinde) sabit kalması — zincirin kopmadığı — doğrulanır.

#include "JoltTestWorld.h"
#include "ragdoll/RagdollInstance.h"
#include "ragdoll/RagdollSkeleton.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

using namespace alazforge;
using namespace alazforge_test;

static int g_failCount = 0;

#define CHECK(cond, msg)                                      \
    do {                                                      \
        if (cond) {                                           \
            printf("  OK  : %s\n", msg);                      \
        } else {                                              \
            printf("  FAIL: %s (satir %d)\n", msg, __LINE__); \
            ++g_failCount;                                    \
        }                                                     \
    } while (0)

int main() {
    TestWorld world;
    world.AddGround();

    RagdollSkeletonConfig skelConfig;

    RagdollBoneConfig pelvis;
    pelvis.name = "pelvis";
    pelvis.parentIndex = -1;
    pelvis.radius = 0.2f;
    pelvis.halfHeight = 0.25f;
    pelvis.localPosition = Vec3{0, 0, 0};
    skelConfig.bones.push_back(pelvis);

    RagdollBoneConfig limb;
    limb.name = "limb";
    limb.parentIndex = 0;
    limb.radius = 0.12f;
    limb.halfHeight = 0.3f;
    limb.localPosition = Vec3{0, -0.7f, 0}; // pelvisin altinda
    limb.swingMaxDeg = 60.0f;
    skelConfig.bones.push_back(limb);

    RagdollSkeleton skeleton(skelConfig);
    RagdollInstance instance;

    Transform startPose;
    startPose.position = Vec3{0, 10.0f, 0};
    instance.Activate(world.physics, skeleton, startPose, Layers::MOVING);

    printf("[Ragdoll]\n");
    CHECK(instance.IsActive(), "ragdoll aktive edildi");
    CHECK(instance.BoneCount() == 2, "iki kemik olusturuldu");

    float maxJointDist = 0.0f;
    for (int i = 0; i < 300; ++i) {
        world.Step(1.0f / 60.0f);
        const Transform pelvisT = instance.GetBoneWorldTransform(0);
        const Transform limbT = instance.GetBoneWorldTransform(1);
        const float dx = pelvisT.position.x - limbT.position.x;
        const float dy = pelvisT.position.y - limbT.position.y;
        const float dz = pelvisT.position.z - limbT.position.z;
        maxJointDist = std::max(maxJointDist, std::sqrt(dx * dx + dy * dy + dz * dz));
    }

    const Transform finalPelvis = instance.GetBoneWorldTransform(0);
    const Transform finalLimb = instance.GetBoneWorldTransform(1);

    printf("  pelvis y = %.2f, limb y = %.2f, max eklem mesafesi = %.2f\n", finalPelvis.position.y,
           finalLimb.position.y, maxJointDist);

    CHECK(finalPelvis.position.y < 9.0f, "pelvis yercekimiyle dustu");
    CHECK(finalLimb.position.y < 9.0f, "limb yercekimiyle dustu");
    CHECK(maxJointDist < 1.5f, "eklem mesafesi sinir icinde kaldi (zincir kopmadi)");
    CHECK(maxJointDist > 0.3f, "eklem mesafesi makul (govdeler ust uste binmedi)");

    instance.Destroy();

    if (g_failCount == 0) {
        printf("TEST BASARILI: Ragdoll cift kemikli zincir tutarli davraniyor.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
