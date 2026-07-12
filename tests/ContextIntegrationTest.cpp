// AlazForgeContext birlesik entegrasyon testi: Context'in artik yalnizca
// fizigi degil, world-scoped tum alt sistemleri (ruzgar, surtunme bolgeleri,
// yuzerlik, LOD, carpisma-ses olaylari, isteğe bagli terrain/destructible)
// TEK bir Step() arkasinda dogru sirayla yonettigini kanitlar -- AlazEngine
// bu sistemleri elle birbirine baglamak zorunda KALMAMALI.

#include "context/AlazForgeContext.h"

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <vector>

using namespace alazforge;

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

static JPH::BodyID AddDynamicBox(JPH::BodyInterface& bi, JPH::RVec3 pos) {
    JPH::BodyCreationSettings settings(new JPH::BoxShape(JPH::Vec3(0.5f, 0.5f, 0.5f)), pos,
                                       JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, 1);
    return bi.CreateAndAddBody(settings, JPH::EActivation::Activate);
}

int main() {
    AlazForgeContext context;
    JPH::BodyInterface& bi = context.Physics().GetBodyInterface();
    printf("[AlazForgeContext birlesik entegrasyon]\n");

    // ── Ruzgar: Context kendi WindSystem'ini kuruyor, Step() zamanini
    // ilerletiyor ──────────────────────────────────────────────────────
    const Vec3 windBefore = context.Wind().GetWindAt(Vec3{0, 0, 0});
    for (int i = 0; i < 60; ++i)
        context.Step(1.0f / 60.0f);
    const Vec3 windAfter = context.Wind().GetWindAt(Vec3{0, 0, 0});
    CHECK(context.Wind().Time() > 0.9f, "Step() cagrildikca WindSystem zamani ilerliyor");
    (void)windBefore;
    (void)windAfter;

    // ── Surtunme bolgeleri: Zones() uzerinden dogrudan erisim ────────────
    FrictionZone ice;
    ice.boundsMin = Vec3{-5, -5, -5};
    ice.boundsMax = Vec3{5, 5, 5};
    ice.friction = 0.05f;
    context.Zones().AddZone(ice);
    const float inZone = context.Zones().QueryFrictionAt(Vec3{0, 0, 0}, 0.8f);
    const float outZone = context.Zones().QueryFrictionAt(Vec3{500, 0, 0}, 0.8f);
    CHECK(inZone < 0.1f, "bolge icinde sorgu buz surtunmesini donduruyor");
    CHECK(outZone == 0.8f, "bolge disinda sorgu varsayilan surtunmeyi donduruyor");

    // ── Yuzerlik: Buoyancy() uzerinden hacim eklenip govde izleniyor,
    // Step() otomatik olarak buoyancy.Update()'i cagiriyor ──────────────
    bi.CreateAndAddBody(
        JPH::BodyCreationSettings(new JPH::BoxShape(JPH::Vec3(100, 1, 100)), JPH::RVec3(0, -20, 0),
                                  JPH::Quat::sIdentity(), JPH::EMotionType::Static, 0),
        JPH::EActivation::DontActivate);

    WaterVolume water;
    water.surfaceY = 0.0f;
    water.buoyancyFactor = 3.0f; // yuksek -> yuzer
    context.Buoyancy().AddVolume(water);

    JPH::BodyID floater = AddDynamicBox(bi, JPH::RVec3(0, -1.0, 0));
    context.Buoyancy().TrackBody(floater);
    for (int i = 0; i < 300; ++i)
        context.Step(1.0f / 60.0f);
    const float floaterY = static_cast<float>(bi.GetPosition(floater).GetY());
    CHECK(floaterY > -2.0f,
          "Context.Step() sirasinda otomatik buoyancy guncellemesi govdeyi yuzdurdu");

    // ── LOD: Step()'e referans nokta verilince otomatik uygulaniyor ─────
    JPH::BodyID farBody = AddDynamicBox(bi, JPH::RVec3(500, 50, 0));
    context.LOD().TrackBody(farBody);
    const Vec3 playerPos{0, 0, 0};
    for (int i = 0; i < 30; ++i)
        context.Step(1.0f / 60.0f, 1, &playerPos);
    CHECK(context.LOD().SleepingCount() == 1,
          "Step()'e verilen referans noktasi uzak govdeyi otomatik uyuttu");

    // ── Carpisma-ses olaylari: CollisionEventSystem Context kurulurken
    // otomatik olarak ContactListener olarak baglaniyor (elle Attach
    // cagirmaya gerek yok) ──────────────────────────────────────────────
    JPH::BodyID faller = AddDynamicBox(bi, JPH::RVec3(20, 20, 0));
    bi.SetLinearVelocity(faller, JPH::Vec3(0, -50.0f, 0));
    bi.CreateAndAddBody(
        JPH::BodyCreationSettings(new JPH::BoxShape(JPH::Vec3(5, 0.5f, 5)), JPH::RVec3(20, 0, 0),
                                  JPH::Quat::sIdentity(), JPH::EMotionType::Static, 0),
        JPH::EActivation::DontActivate);
    bool sawCollisionEvent = false;
    for (int i = 0; i < 60 && !sawCollisionEvent; ++i) {
        context.Step(1.0f / 60.0f);
        std::vector<CollisionEvent> events;
        context.DrainCollisionEvents(events);
        if (!events.empty()) sawCollisionEvent = true;
    }
    CHECK(sawCollisionEvent,
          "Context otomatik baglanan CollisionEventSystem ile carpisma olayi uretti");

    // ── Isteğe bagli terrain/destructible: yalnizca EnableX() cagrilinca
    // kuruluyor (disk I/O yan etkisi olmadan varsayilan Context hafif
    // kaliyor), tekrar cagrilinca ayni ornegi donduruyor (idempotent) ────
    CHECK(!context.HasTerrain() && !context.HasDestructible(),
          "varsayilan Context terrain/destructible kurmuyor (disk I/O yok)");

    const std::string terrainDir =
        (std::filesystem::temp_directory_path() / "alazforge_ctx_terrain_test").string();
    std::filesystem::remove_all(terrainDir);
    TerrainDeformSystem::Config terrainCfg;
    terrainCfg.savePath = terrainDir;
    TerrainDeformSystem& terrain1 = context.EnableTerrain(terrainCfg);
    TerrainDeformSystem& terrain2 = context.EnableTerrain(terrainCfg);
    CHECK(&terrain1 == &terrain2, "EnableTerrain tekrar cagrilinca ayni ornegi donduruyor");
    terrain1.ApplyDeformation(1.0f, 1.0f, 0.3f);
    CHECK(terrain1.GetDepthAt(1.0f, 1.0f) > 0.0f, "EnableTerrain sonrasi deformasyon calisiyor");

    const std::string destructibleDir =
        (std::filesystem::temp_directory_path() / "alazforge_ctx_destructible_test").string();
    std::filesystem::remove_all(destructibleDir);
    DestructibleStructureSystem::Config destructibleCfg;
    destructibleCfg.savePath = destructibleDir;
    DestructibleStructureSystem& destructible1 = context.EnableDestructible(destructibleCfg);
    DestructibleStructureSystem& destructible2 = context.EnableDestructible(destructibleCfg);
    CHECK(&destructible1 == &destructible2,
          "EnableDestructible tekrar cagrilinca ayni ornegi donduruyor");

    context.TerrainIfEnabled()->Flush();
    context.DestructibleIfEnabled()->Flush();
    std::filesystem::remove_all(terrainDir);
    std::filesystem::remove_all(destructibleDir);

    if (g_failCount == 0) {
        printf(
            "TEST BASARILI: AlazForgeContext world-scoped tum alt sistemleri birlesik "
            "yonetiyor.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
