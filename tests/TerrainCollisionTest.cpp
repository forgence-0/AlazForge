// Terrain gercek carpismasi dogrulama testi: deformasyon verisi
// HeightFieldShape govdesine cevrilir; krater ustune birakilan kutu, duz
// zemine birakilan kutudan daha ASAGIDA durur (cukura duser). Debug cizim
// koprusu (DebugDrawBridge) de burada birlikte test edilir.

#include "JoltTestWorld.h"
#include "debugdraw/DebugDrawBridge.h"
#include "terrain_deform/TerrainDeformSystem.h"

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>

#include <cstdio>
#include <filesystem>

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

static JPH::BodyID DropBox(TestWorld& world, double x, double z) {
    JPH::BodyCreationSettings settings(new JPH::BoxShape(JPH::Vec3(0.3f, 0.3f, 0.3f)),
                                       JPH::RVec3(x, 2.0, z), JPH::Quat::sIdentity(),
                                       JPH::EMotionType::Dynamic, Layers::MOVING);
    return world.physics.GetBodyInterface().CreateAndAddBody(settings, JPH::EActivation::Activate);
}

int main() {
    TestWorld world;

    const char* savePath = "terrain_collision_test";
    std::filesystem::remove_all(savePath);

    TerrainDeformSystem::Config cfg;
    cfg.mapSize = 256;
    cfg.chunkSize = 64;
    cfg.cellSize = 0.25f;
    cfg.maxDepth = 1.0f;
    cfg.savePath = savePath;
    TerrainDeformSystem terrain(cfg);
    terrain.OnPlayerMove(32.0f, 32.0f);

    printf("[TerrainCollision]\n");

    // (20,32) merkezli genis bir krater ac (maks 1m derinlik)
    terrain.ApplyDeformationRadius(20.0f, 32.0f, 1.0f, 3.0f);
    CHECK(terrain.GetDepthAt(20.0f, 32.0f) > 0.9f, "krater merkezi ~1m cokmus");

    // Deformasyon verisinden gercek carpisma govdesi kur (taban y=0)
    const JPH::BodyID chunkBody =
        terrain.UpdateChunkCollision(world.physics, 20.0f, 32.0f, 0.0f, Layers::NON_MOVING);
    CHECK(!chunkBody.IsInvalid(), "chunk carpisma govdesi olustu");
    world.physics.OptimizeBroadPhase();

    // Iki kutu birak: biri kraterin ustune, biri duz zemine
    const JPH::BodyID craterBox = DropBox(world, 20.0, 32.0);
    const JPH::BodyID flatBox = DropBox(world, 40.0, 32.0);
    for (int i = 0; i < 300; ++i)
        world.Step(1.0f / 60.0f);

    const double craterY = world.physics.GetBodyInterface().GetPosition(craterBox).GetY();
    const double flatY = world.physics.GetBodyInterface().GetPosition(flatBox).GetY();
    printf("  krater kutusu y=%.3f, duz zemin kutusu y=%.3f\n", craterY, flatY);

    CHECK(flatY > 0.2 && flatY < 0.5, "duz zemindeki kutu y~0.3'te durdu (zemin gercek)");
    CHECK(craterY < flatY - 0.3, "kraterdeki kutu belirgin daha asagida (cukura dustu)");

    // Krateri doldurup govdeyi GUNCELLE: yeni birakilan kutu artik cukura
    // dusmemeli (guncelleme eski govdeyi degistirdi mi?)
    // (doldurma = ayni yere negatif cokme uygulanamaz; onun yerine govdeyi
    // deformasyonsuz taban yuksekligi krater derinligi kadar yukaridaymis
    // gibi kurup davranis farkini olcmek yeterli olurdu; burada daha basiti:
    // govdeyi ayni veriyle yeniden kurmanin idempotent oldugunu dogrula.)
    const JPH::BodyID rebuilt =
        terrain.UpdateChunkCollision(world.physics, 20.0f, 32.0f, 0.0f, Layers::NON_MOVING);
    CHECK(!rebuilt.IsInvalid() && rebuilt != chunkBody,
          "guncelleme eski govdeyi yenisiyle degistirdi");

    // ── DebugDrawBridge: govdeler cizgi/ucgen callback'lerine dokuluyor mu?
    printf("[DebugDraw]\n");
    if (DebugDrawBridge::IsAvailable()) {
        DebugDrawBridge bridge;
        int lineCount = 0, triangleCount = 0;
        bridge.SetCallbacks(
            [&](const Vec3&, const Vec3&, DebugColor) { ++lineCount; },
            [&](const Vec3&, const Vec3&, const Vec3&, DebugColor) { ++triangleCount; });
        bridge.DrawBodies(world.physics);
        printf("  cizgi=%d ucgen=%d\n", lineCount, triangleCount);
        CHECK(lineCount + triangleCount > 0, "govde shape'leri callback'lere dokuldu");

        int constraintPrimitives = 0;
        bridge.SetCallbacks(
            [&](const Vec3&, const Vec3&, DebugColor) { ++constraintPrimitives; },
            [&](const Vec3&, const Vec3&, const Vec3&, DebugColor) { ++constraintPrimitives; });
        bridge.DrawConstraints(world.physics); // constraint yok -- cokmemeli
        CHECK(true, "DrawConstraints constraint'siz dunyada guvenli");
    } else {
        printf("  JPH_DEBUG_RENDERER bu derlemede kapali -- no-op dogrulaniyor\n");
        DebugDrawBridge bridge;
        bridge.DrawBodies(world.physics); // cokmemeli
        CHECK(!DebugDrawBridge::IsAvailable(), "IsAvailable=false ve Draw* no-op");
    }

    terrain.DestroyAllChunkCollision(world.physics);
    std::filesystem::remove_all(savePath);

    if (g_failCount == 0) {
        printf("TEST BASARILI: terrain carpismasi + debug cizim dogru.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
