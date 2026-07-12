// SpringDamperJoint dogrulama testi: yercekimi kapatilip iki serbest gövde
// dinlenme mesafesinden uzaga itilir, yay+sönümleme onlari geri cekip
// sönümlü sekilde dinlenme mesafesine yaklastirmali (asiri salinip
// savrulmamali). CurrentLength/CurrentTensionForce sorgulari da dogrulanir.

#include "JoltTestWorld.h"
#include "joints/SpringDamperJoint.h"

#include <Jolt/Physics/Body/BodyLock.h>

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

static JPH::BodyID AddDynamicBox(TestWorld& world, JPH::RVec3 pos) {
    JPH::BodyCreationSettings settings(new JPH::BoxShape(JPH::Vec3(0.3f, 0.3f, 0.3f)), pos,
                                       JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic,
                                       Layers::MOVING);
    return world.physics.GetBodyInterface().CreateAndAddBody(settings, JPH::EActivation::Activate);
}

int main() {
    TestWorld world;
    world.physics.SetGravity(JPH::Vec3::sZero());

    printf("[SpringDamperJoint]\n");

    // Dinlenme mesafesi 2m olacak sekilde, baslangicta 5m uzaklikta
    // konumlandirilmis iki govde -- yay onlari birbirine cekmeli.
    JPH::BodyID a = AddDynamicBox(world, JPH::RVec3(-2.5, 0, 0));
    JPH::BodyID b = AddDynamicBox(world, JPH::RVec3(2.5, 0, 0));

    SpringDamperJointConfig cfg;
    cfg.restLength = 2.0f;
    cfg.frequencyHz = 3.0f;
    cfg.damping = 0.9f; // asiri sönümlüye yakin: hizli, savrulmadan yakinsasin
    SpringDamperJoint joint(cfg);
    joint.Create(world.physics, a, b, Vec3{-2.5f, 0, 0}, Vec3{2.5f, 0, 0});

    CHECK(joint.IsCreated(), "eklem olusturuldu");

    const float startLength = joint.CurrentLength();
    CHECK(std::abs(startLength - 5.0f) < 0.01f, "baslangic mesafesi ~5m");

    float maxTension = 0.0f;
    for (int i = 0; i < 300; ++i) {
        world.Step(1.0f / 60.0f);
        maxTension = std::max(maxTension, joint.CurrentTensionForce(1.0f / 60.0f));
    }

    const float finalLength = joint.CurrentLength();
    printf("  baslangic=%.3f son=%.3f dinlenme=%.3f maxGerginlik=%.1fN\n", startLength, finalLength,
           cfg.restLength, maxTension);

    CHECK(std::abs(finalLength - cfg.restLength) < 0.15f,
          "300 adim sonra dinlenme mesafesine yakinsadi");
    CHECK(maxTension > 0.0f, "gerilirken pozitif gerginlik kuvveti olculdu");

    // Destroy sonrasi govdeler artik bagli degil -- serbest birakilinca
    // dinlenme mesafesinde kalmali (disari itilmemeli).
    joint.Destroy();
    CHECK(!joint.IsCreated(), "eklem kaldirildiktan sonra IsCreated() false");
    const float lengthAfterFreeStep = [&] {
        world.Step(1.0f / 60.0f);
        JPH::BodyLockRead lockA(world.physics.GetBodyLockInterface(), a);
        JPH::BodyLockRead lockB(world.physics.GetBodyLockInterface(), b);
        return JPH::Vec3(lockA.GetBody().GetPosition() - lockB.GetBody().GetPosition()).Length();
    }();
    CHECK(std::abs(lengthAfterFreeStep - finalLength) < 0.05f,
          "eklem kaldirildiktan sonra govdeler serbest kaliyor (ani sicrama yok)");

    printf("[SpringDamperJoint] %s (%d hata)\n", g_failCount == 0 ? "GECTI" : "BASARISIZ",
           g_failCount);
    return g_failCount == 0 ? 0 : 1;
}
