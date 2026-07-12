// RopeSystem dogrulama testi: bir ucu tavana sabitlenen halat yercekimiyle
// sarkar (serbest uc, sabit ucun ALTINA iner), segmentler kopmaz (ardisik
// segmentler arasi mesafe segment boyunu asmaz), su durumu sorgusu
// (BuoyancySystem::QueryWaterState) da burada birlikte test edilir --
// karakter yuzme mekaniginin ihtiyaci olan sorgu.

#include "JoltTestWorld.h"
#include "buoyancy/BuoyancySystem.h"
#include "rope/RopeSystem.h"

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

    // "Tavan": y=10'da statik blok
    JPH::BodyID ceiling = world.AddStaticBox(JPH::RVec3(0, 10.0, 0), JPH::Vec3(1, 0.5f, 1), 0);

    printf("[RopeSystem]\n");

    RopeConfig cfg;
    cfg.segmentCount = 12;
    cfg.totalLength = 6.0f;
    RopeSystem rope(cfg);

    // Halati tavandan yana dogru YATAY kur (y=9.5 hizasinda) -- yercekimi
    // onu asagi sarkitmali.
    rope.Create(world.physics, Vec3{0, 9.5f, 0}, Vec3{6.0f, 9.5f, 0}, Layers::MOVING);
    rope.AttachStartTo(ceiling, Vec3{0, 9.5f, 0});

    CHECK(rope.SegmentCount() == 12, "12 segment olustu");

    for (int i = 0; i < 600; ++i)
        world.Step(1.0f / 60.0f);

    // Serbest uc, sabit noktanin belirgin ALTINA sarkmali
    const Transform tail = rope.GetSegmentTransform(rope.SegmentCount() - 1);
    printf("  serbest uc: (%.2f, %.2f, %.2f)\n", tail.position.x, tail.position.y, tail.position.z);
    CHECK(tail.position.y < 6.0f, "halat yercekimiyle asagi sarkti");

    // Sabit uc hala tavana yakin mi? (kopmadi)
    const Transform head = rope.GetSegmentTransform(0);
    CHECK(std::fabs(head.position.y - 9.5f) < 1.0f, "sabit uc tavanda kaldi (eklem kopmadi)");

    // Ardisik segmentler arasi mesafe segment boyunun ~2 katini asmasin
    const float segLen = cfg.totalLength / static_cast<float>(cfg.segmentCount);
    bool chainIntact = true;
    for (int i = 0; i + 1 < rope.SegmentCount(); ++i) {
        const Vec3 a = rope.GetSegmentTransform(i).position;
        const Vec3 b = rope.GetSegmentTransform(i + 1).position;
        const float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
        if (std::sqrt(dx * dx + dy * dy + dz * dz) > 2.0f * segLen) chainIntact = false;
    }
    CHECK(chainIntact, "zincir butun (hicbir eklem ayrilmadi)");

    rope.Destroy();

    // ── BuoyancySystem::QueryWaterState (karakter yuzme sorgusu) ────────
    printf("[QueryWaterState]\n");
    BuoyancySystem buoyancy;
    WaterVolume volume;
    volume.surfaceY = 5.0f;
    volume.boundsMin = Vec3{-10, 0, -10};
    volume.boundsMax = Vec3{10, 5.0f, 10};
    volume.flowVelocity = Vec3{1.5f, 0, 0};
    buoyancy.AddVolume(volume);

    const BuoyancySystem::WaterState under = buoyancy.QueryWaterState(Vec3{0, 3.0f, 0});
    CHECK(under.inWater, "su icindeki nokta inWater=true");
    CHECK(std::fabs(under.submergedDepth - 2.0f) < 1.0e-4f, "batiklik derinligi dogru (2m)");
    CHECK(std::fabs(under.flowVelocity.x - 1.5f) < 1.0e-4f, "akinti hizi dogru");

    const BuoyancySystem::WaterState dry = buoyancy.QueryWaterState(Vec3{0, 8.0f, 0});
    CHECK(!dry.inWater, "su disindaki nokta inWater=false");

    if (g_failCount == 0) {
        printf("TEST BASARILI: RopeSystem sarkma/zincir + QueryWaterState dogru.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
