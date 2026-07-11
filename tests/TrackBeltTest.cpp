// S3 doğrulama testi: VehicleSystem tekerlek transform sorgusu +
// TrackBeltSystem (genel kayış/tırtıl link yerleşimi geometrisi).

#include "JoltTestWorld.h"
#include "vehicle/TrackBeltSystem.h"
#include "vehicle/VehicleSystem.h"

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

static float Dist(const Vec3& a, const Vec3& b) {
    const float dx = a.x - b.x, dy = a.y - b.y, dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

static std::vector<AxleConfig> MakeTankAxles() {
    std::vector<AxleConfig> axles(4);
    const float z[4] = {1.8f, 0.6f, -0.6f, -1.8f};
    for (int i = 0; i < 4; ++i) {
        axles[i].positionZ = z[i];
        axles[i].attachHeight = -0.4f;
        axles[i].halfTrackWidth = 1.1f;
        axles[i].wheelRadius = 0.45f;
        axles[i].wheelWidth = 0.4f;
        axles[i].suspensionMinLength = 0.25f;
        axles[i].suspensionMaxLength = 0.5f;
        axles[i].suspensionFrequency = 2.0f;
        axles[i].suspensionDamping = 0.7f;
        axles[i].trackLateralFriction = 0.8f;
        axles[i].driven = true;
    }
    return axles;
}

int main() {
    TestWorld world;
    world.AddGround();
    printf("[VehicleSystem tekerlek transform]\n");

    VehicleChassisConfig chassis;
    chassis.halfExtents = Vec3{1.3f, 0.6f, 2.6f};
    chassis.mass = 12000.0f;
    chassis.position = Vec3{0, 2.0f, 0};
    EngineConfig engine;
    TransmissionConfig trans;

    VehicleSystem tank;
    tank.CreateTracked(world.physics, chassis, MakeTankAxles(), engine, trans, Layers::MOVING);

    CHECK(tank.WheelCount() == 8, "4 akslik paletli aracin 8 tekerlegi var (2/aks)");

    const Transform w0Before = tank.GetWheelTransform(0);
    CHECK(std::fabs(w0Before.position.y - chassis.position.y) < 3.0f,
          "ilk tekerlegin baslangic pozisyonu govdeye yakin");

    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 60; ++i) {
        tank.SetDriverInput(0.0f, 0.0f, 0.0f, 1.0f); // el freni: yerlessin
        world.Step(dt);
    }
    for (int i = 0; i < 120; ++i) {
        tank.SetDriverInput(1.0f, 0.0f, 0.0f, 0.0f);
        world.Step(dt);
    }
    const Transform w0After = tank.GetWheelTransform(0);
    CHECK(Dist(w0Before.position, w0After.position) > 0.5f,
          "tekerlek transform'u arac ilerledikce degisiyor (dondugu icin de sabit kalmiyor)");

    CHECK(tank.GetWheelTransform(999).position.x == 0.0f &&
              tank.GetWheelTransform(999).position.y == 0.0f,
          "gecersiz tekerlek indeksi bos transform donduruyor (cokme yok)");

    // ── TrackBeltSystem: iki ayni-yaricapli makara arasindaki kayis ──────
    printf("[TrackBeltSystem - iki makara]\n");
    {
        TrackBeltConfig cfg;
        cfg.wheelLocalPositions = {Vec3{0, 0, 0}, Vec3{0, 0, 5}};
        cfg.wheelRadius = 1.0f;
        cfg.linkLength = 1.0f;
        cfg.planeNormal = Vec3{1, 0, 0};
        TrackBeltSystem belt(cfg);

        // Beklenen: 2 duz parca (5m) + 2 yarim cember (pi*r her biri)
        const float expected = 2.0f * 5.0f + 2.0f * 3.14159265f * 1.0f;
        CHECK(std::fabs(belt.TotalLength() - expected) < 0.05f,
              "iki makarali kayisin toplam uzunlugu beklenen (duz+yarim cember) degere yakin");

        std::vector<Transform> links;
        belt.ComputeLinkTransforms(0.0f, links);
        CHECK(static_cast<int>(links.size()) == belt.LinkCount(),
              "ComputeLinkTransforms LinkCount() kadar link donduruyor");
        CHECK(!links.empty(), "en az bir link uretildi");

        // Ardisik linkler birbirine yaklasik linkLength kadar uzak olmali
        // (kayis boyunca esit araliklandirma).
        bool spacingOk = true;
        for (size_t i = 1; i < links.size() && i < 5; ++i) {
            const float d = Dist(links[i - 1].position, links[i].position);
            if (d < 0.5f || d > 1.5f) spacingOk = false;
        }
        CHECK(spacingOk, "ardisik linkler yaklasik linkLength araliginda");

        // Tum link pozisyonlari x=0 duzleminde kalmali (planeNormal=X).
        bool inPlane = true;
        for (const Transform& t : links)
            if (std::fabs(t.position.x) > 1.0e-3f) inPlane = false;
        CHECK(inPlane, "tum linkler planeNormal'e dik duzlemde (x=0) kaliyor");

        // beltOffset ilerletilince linkler kayis boyunca kayar.
        std::vector<Transform> linksShifted;
        belt.ComputeLinkTransforms(2.0f, linksShifted);
        const float shift = Dist(links[0].position, linksShifted[0].position);
        CHECK(shift > 0.1f, "beltOffset degisince ilk link pozisyonu kayiyor");
    }

    if (g_failCount == 0) {
        printf("TEST BASARILI: tekerlek transform + TrackBeltSystem dogru calisiyor.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
