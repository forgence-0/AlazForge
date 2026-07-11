// S3 doğrulama testi: MagneticFieldSystem.
// Çekim kaynağı yakınındaki izlenen gövdeyi kendine çeker, yarıçap dışındaki
// gövdeyi etkilemez; itme (repel) modu tam tersi yönde iter.

#include "JoltTestWorld.h"
#include "magnetism/MagneticFieldSystem.h"

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

static JPH::BodyID AddBox(TestWorld& world, JPH::RVec3 pos) {
    JPH::BodyCreationSettings s(new JPH::BoxShape(JPH::Vec3(0.3f, 0.3f, 0.3f)), pos,
                                JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, Layers::MOVING);
    return world.physics.GetBodyInterface().CreateAndAddBody(s, JPH::EActivation::Activate);
}

int main() {
    // Yercekimini "kapatmak" icin havada, buyuk mesafeler kat etmeden kisa
    // surede net yatay hareketi olcuyoruz (dikey dusme gurultuyu bogmasin).
    TestWorld world;
    printf("[MagneticFieldSystem]\n");

    MagneticFieldSystem field;
    MagneticFieldSource attractor;
    attractor.position = Vec3{0, 50, 0};
    attractor.radius = 20.0f;
    attractor.strength = 80000.0f;
    attractor.minDistance = 1.0f;
    field.AddSource(attractor);

    JPH::BodyID nearBody = AddBox(world, JPH::RVec3(10, 50, 0)); // yaricap icinde
    JPH::BodyID farBody = AddBox(world, JPH::RVec3(100, 50, 0)); // yaricap disinda
    field.TrackBody(nearBody);
    field.TrackBody(farBody);

    JPH::BodyInterface& bi = world.physics.GetBodyInterface();
    const double nearStartX = bi.GetPosition(nearBody).GetX();
    const double farStartX = bi.GetPosition(farBody).GetX();

    for (int i = 0; i < 180; ++i) {
        field.Update(world.physics);
        world.Step(1.0f / 60.0f);
    }

    const double nearEndX = bi.GetPosition(nearBody).GetX();
    const double farEndX = bi.GetPosition(farBody).GetX();

    CHECK(nearEndX < nearStartX - 0.5, "yaricap icindeki govde kaynaga (x=0) dogru cekildi");
    CHECK(std::fabs(farEndX - farStartX) < 0.5,
          "yaricap disindaki govde belirgin sekilde etkilenmedi");

    // ── Itme (repel) modu ────────────────────────────────────────────────
    printf("[MagneticFieldSystem - itme]\n");
    {
        MagneticFieldSystem repelField;
        MagneticFieldSource repeller;
        repeller.position = Vec3{0, 80, 0};
        repeller.radius = 20.0f;
        repeller.strength = 80000.0f;
        repeller.minDistance = 1.0f;
        repeller.repel = true;
        repelField.AddSource(repeller);

        JPH::BodyID body = AddBox(world, JPH::RVec3(10, 80, 0));
        repelField.TrackBody(body);
        const double startX = bi.GetPosition(body).GetX();
        for (int i = 0; i < 180; ++i) {
            repelField.Update(world.physics);
            world.Step(1.0f / 60.0f);
        }
        const double endX = bi.GetPosition(body).GetX();
        CHECK(endX > startX + 0.5, "itme modunda govde kaynaktan uzaklasti");
    }

    if (g_failCount == 0) {
        printf(
            "TEST BASARILI: MagneticFieldSystem cekme/itme + yaricap siniri dogru "
            "calisiyor.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
