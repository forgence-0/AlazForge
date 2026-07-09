// Faz B.4 doğrulama testi: BuoyancySystem.
// İki ayrı su hacmi (farklı buoyancyFactor — biri "yüzer" biri "batar"
// davranışını temsil eder) içine aynı şekilde düşürülen iki kutu; yüksek
// buoyancyFactor'lu kutunun su yüzeyine yakın kaldığı, düşük buoyancyFactor'lu
// kutunun ise belirgin şekilde daha derine battığı doğrulanır.

#include "JoltTestWorld.h"
#include "buoyancy/BuoyancySystem.h"

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>

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
    JPH::BodyCreationSettings settings(new JPH::BoxShape(JPH::Vec3(0.5f, 0.5f, 0.5f)), pos,
                                       JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic,
                                       Layers::MOVING);
    JPH::BodyID id =
        world.physics.GetBodyInterface().CreateAndAddBody(settings, JPH::EActivation::Activate);
    return id;
}

int main() {
    TestWorld world;

    // Derin bir taban: batan kutu sonsuza dek dusmesin
    world.AddStaticBox(JPH::RVec3(0.0, -15.0, 0.0), JPH::Vec3(100.0f, 1.0f, 100.0f), 0);

    BuoyancySystem buoyancy;

    WaterVolume floatVolume;
    floatVolume.surfaceY = 0.0f;
    floatVolume.boundsMin = Vec3{-100, -20, -100};
    floatVolume.boundsMax = Vec3{-1, 20, 100};
    floatVolume.buoyancyFactor = 3.0f; // yuksek -> yuzer
    buoyancy.AddVolume(floatVolume);

    WaterVolume sinkVolume;
    sinkVolume.surfaceY = 0.0f;
    sinkVolume.boundsMin = Vec3{1, -20, -100};
    sinkVolume.boundsMax = Vec3{100, 20, 100};
    sinkVolume.buoyancyFactor = 0.2f; // dusuk -> batar
    buoyancy.AddVolume(sinkVolume);

    JPH::BodyID floater = AddDynamicBox(world, JPH::RVec3(-10.0, 5.0, 0.0));
    JPH::BodyID sinker = AddDynamicBox(world, JPH::RVec3(10.0, 5.0, 0.0));
    buoyancy.TrackBody(floater);
    buoyancy.TrackBody(sinker);
    world.physics.OptimizeBroadPhase();

    printf("[BuoyancySystem]\n");

    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 600; ++i) {
        buoyancy.Update(world.physics, dt);
        world.Step(dt);
    }

    const float floaterY =
        FromJolt(JPH::Vec3(world.physics.GetBodyInterface().GetPosition(floater))).y;
    const float sinkerY =
        FromJolt(JPH::Vec3(world.physics.GetBodyInterface().GetPosition(sinker))).y;

    printf("  yuzen kutu y = %.2f, batan kutu y = %.2f\n", floaterY, sinkerY);

    CHECK(floaterY > sinkerY + 3.0f,
          "yuksek buoyancy'li kutu, dusuk olandan belirgin yukarida kaldi");
    CHECK(floaterY > -3.0f, "yuzen kutu su yuzeyine yakin kaldi, tabana batmadi");
    CHECK(sinkerY < -5.0f, "batan kutu belirgin sekilde derine indi");

    buoyancy.UntrackBody(floater);
    buoyancy.UntrackBody(sinker);
    world.physics.GetBodyInterface().RemoveBody(floater);
    world.physics.GetBodyInterface().DestroyBody(floater);
    world.physics.GetBodyInterface().RemoveBody(sinker);
    world.physics.GetBodyInterface().DestroyBody(sinker);

    if (g_failCount == 0) {
        printf("TEST BASARILI: BuoyancySystem yuksek/dusuk buoyancy'yi ayirt ediyor.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
