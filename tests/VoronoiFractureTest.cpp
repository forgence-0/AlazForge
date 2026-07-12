// VoronoiFracture dogrulama testi: yari-uzay kirpma ile hesaplanan
// hucrelerin GERCEKTEN Voronoi ozelligini tasidigini (her hucrenin her
// kosesi, kendi tohumuna diger tum tohumlardan daha yakin ya da esit
// mesafede) dogrudan geometrik olarak kanitlar -- yaklaşık/gorsel bir
// kontrol degil. Ayrica FractureVoronoi'nin gercek dinamik govdeler
// uretip darbe noktasindan disari firlattigini fizik simulasyonuyla
// dogrular.

#include "JoltTestWorld.h"
#include "destructible/VoronoiFracture.h"

#include <Jolt/Physics/Body/BodyLock.h>

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

static float DistSq(const Vec3& a, const Vec3& b) {
    const float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return dx * dx + dy * dy + dz * dz;
}

int main() {
    printf("[VoronoiFracture]\n");

    const Vec3 boxMin{-5, -5, -5};
    const Vec3 boxMax{5, 5, 5};
    // Duzensiz yerlesim (kasitli olarak simetrik degil) -- kirpma sirasi
    // ve yon bagimliligini de sinamak icin.
    const std::vector<Vec3> seeds = {
        Vec3{-3.0f, -2.0f, 1.0f}, Vec3{2.5f, 3.0f, -1.5f}, Vec3{0.5f, -3.5f, -2.0f},
        Vec3{-1.0f, 1.5f, 3.0f},  Vec3{3.5f, -1.0f, 2.0f},
    };

    const std::vector<VoronoiCell> cells = ComputeVoronoiCells(boxMin, boxMax, seeds);
    CHECK(cells.size() == seeds.size(), "her tohum icin bir hucre uretildi");

    // Temel Voronoi ozelligi: hucre i'nin her kosesi, tohum i'ye diger
    // TUM tohumlardan daha yakin (ya da esit) olmali. Bu, kirpma
    // algoritmasinin doğru bisector duzlemlerini uyguladiginin dogrudan
    // kaniti -- yaklasik degil, matematiksel bir garanti.
    bool allVerticesCorrect = true;
    size_t totalVertices = 0;
    for (size_t i = 0; i < cells.size(); ++i) {
        for (const Vec3& v : cells[i].vertices) {
            ++totalVertices;
            const float dOwn = DistSq(v, seeds[i]);
            for (size_t j = 0; j < seeds.size(); ++j) {
                if (j == i) continue;
                const float dOther = DistSq(v, seeds[j]);
                if (dOwn > dOther + 1.0e-3f) {
                    allVerticesCorrect = false;
                    printf("    ihlal: hucre %zu kosesi tohum %zu'ye tohum %zu'den daha uzak\n", i,
                           i, j);
                }
            }
        }
    }
    CHECK(allVerticesCorrect, "her hucrenin tum koseleri kendi tohumuna en yakin (gercek Voronoi)");
    CHECK(totalVertices > 0, "hucreler bos degil");

    // Hucre koseleri kutu sinirlarini asmamali (kirpma kutuya gore de yapildi)
    bool allInsideBox = true;
    for (const VoronoiCell& c : cells) {
        for (const Vec3& v : c.vertices) {
            if (v.x < boxMin.x - 1.0e-3f || v.x > boxMax.x + 1.0e-3f || v.y < boxMin.y - 1.0e-3f ||
                v.y > boxMax.y + 1.0e-3f || v.z < boxMin.z - 1.0e-3f || v.z > boxMax.z + 1.0e-3f) {
                allInsideBox = false;
            }
        }
    }
    CHECK(allInsideBox, "tum hucre koseleri bolge kutusu icinde kaldi");

    // ── FractureVoronoi: gercek fizik govdeleri + darbe itmesi ──────────
    TestWorld world;
    const JPH::BodyID staticFloor = world.AddGround();
    (void)staticFloor;

    VoronoiFractureConfig cfg;
    // Parcalar gercek yogunluk*hacimden hesaplanan kutleye sahip (10 birimlik
    // kutu, varsayilan 1000 kg/m^3 yogunlukla -- parca basina ~1.5-2.3e5 kg).
    // Boyle agir parcalari 10 fizik adiminda yercekiminin (9.81 m/s^2) altina
    // ezilmeden gercekten disari itebilmek icin darbe impulsu da orantili
    // buyuklukte olmali.
    cfg.impactImpulseN = 5.0e6f;
    const Vec3 impactPoint{-5.0f, 0.0f, 0.0f}; // kutunun sol yuzunden vurulmus gibi

    const std::vector<JPH::BodyID> bodies =
        FractureVoronoi(world.physics, Layers::MOVING, boxMin, boxMax, seeds, impactPoint, cfg);

    CHECK(bodies.size() == seeds.size(), "her tohum icin gercek bir dinamik govde olusturuldu");
    for (JPH::BodyID id : bodies)
        CHECK(!id.IsInvalid(), "gecerli BodyID");

    // Baslangic mesafeleri (darbe noktasindan) kaydet
    std::vector<float> startDist;
    for (JPH::BodyID id : bodies) {
        JPH::BodyLockRead lock(world.physics.GetBodyLockInterface(), id);
        const JPH::RVec3 pos = lock.GetBody().GetPosition();
        const Vec3 p = Vec3{static_cast<float>(pos.GetX()), static_cast<float>(pos.GetY()),
                            static_cast<float>(pos.GetZ())};
        startDist.push_back(std::sqrt(DistSq(p, impactPoint)));
    }

    for (int i = 0; i < 10; ++i)
        world.Step(1.0f / 60.0f);

    bool allMovedAway = true;
    for (size_t i = 0; i < bodies.size(); ++i) {
        JPH::BodyLockRead lock(world.physics.GetBodyLockInterface(), bodies[i]);
        const JPH::RVec3 pos = lock.GetBody().GetPosition();
        const Vec3 p = Vec3{static_cast<float>(pos.GetX()), static_cast<float>(pos.GetY()),
                            static_cast<float>(pos.GetZ())};
        const float d = std::sqrt(DistSq(p, impactPoint));
        if (d <= startDist[i]) allMovedAway = false;
    }
    CHECK(allMovedAway, "darbe itmesiyle tum parcalar darbe noktasindan uzaklasti");

    printf("[VoronoiFracture] %s (%d hata)\n", g_failCount == 0 ? "GECTI" : "BASARISIZ",
           g_failCount);
    return g_failCount == 0 ? 0 : 1;
}
