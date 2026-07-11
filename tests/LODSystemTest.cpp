// LODSystem dogrulama testi: uzak govde zorla uyutulur (yercekimine ragmen
// dusmeyi birakir), yakina donunce uyandirilir (tekrar dusmeye baslar).

#include "JoltTestWorld.h"
#include "lod/LODSystem.h"

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

int main() {
    TestWorld world;
    printf("[LODSystem]\n");

    // Zeminsiz dunya: govde serbest dusuyor, LOD uyuturunsa dusme durur
    JPH::BodyCreationSettings s(new JPH::BoxShape(JPH::Vec3(0.3f, 0.3f, 0.3f)),
                                JPH::RVec3(200.0, 50.0, 0.0), JPH::Quat::sIdentity(),
                                JPH::EMotionType::Dynamic, Layers::MOVING);
    JPH::BodyID box =
        world.physics.GetBodyInterface().CreateAndAddBody(s, JPH::EActivation::Activate);

    LODConfig cfg;
    cfg.sleepDistance = 150.0f;
    cfg.wakeDistance = 100.0f;
    LODSystem lod(cfg);
    lod.TrackBody(box);

    // Oyuncu uzakta (mesafe = 200 > sleepDistance): govde uyutulmali
    lod.Update(world.physics, Vec3{0, 0, 0});
    CHECK(lod.SleepingCount() == 1, "uzak govde LOD tarafindan uyutuldu");

    const double yBeforeStep = world.physics.GetBodyInterface().GetPosition(box).GetY();
    for (int i = 0; i < 120; ++i) {
        lod.Update(world.physics, Vec3{0, 0, 0});
        world.Step(1.0f / 60.0f);
    }
    const double yAfterStep = world.physics.GetBodyInterface().GetPosition(box).GetY();
    CHECK(std::fabs(yAfterStep - yBeforeStep) < 1.0e-4,
          "uyutulmus govde 2 saniyede dusmedi (fizik donduruldu)");

    // Oyuncu yaklasir (mesafe = 50 < wakeDistance): govde uyanmali ve dusmeli
    for (int i = 0; i < 60; ++i) {
        lod.Update(world.physics, Vec3{200, 0, 0});
        world.Step(1.0f / 60.0f);
    }
    CHECK(lod.SleepingCount() == 0, "yakindaki govde uyandirildi");
    const double yAfterWake = world.physics.GetBodyInterface().GetPosition(box).GetY();
    CHECK(yAfterWake < yAfterStep - 0.5, "uyanan govde tekrar dusmeye basladi");

    lod.UntrackBody(world.physics, box);
    CHECK(lod.TrackedCount() == 0, "izleme birakildi");

    if (g_failCount == 0) {
        printf("TEST BASARILI: LODSystem mesafeye gore uyutma/uyandirma dogru.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
