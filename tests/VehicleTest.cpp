// Faz 5 doğrulama testi: araç fiziği (tekerlekli + paletli).
// Gerçek sürüş simülasyonu: zemine konan çok akslı vasıta gaz verilince
// ilerler/hızlanır, süspansiyon oturur, direksiyon/diferansiyel yön değiştirir.

#include "JoltTestWorld.h"
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

// 3 akslı ağır kamyon: ön aks direksiyonlu, arka iki aks sürülen
static std::vector<AxleConfig> MakeTruckAxles() {
    std::vector<AxleConfig> axles(3);
    const float z[3] = {3.5f, -1.0f, -3.0f};
    for (int i = 0; i < 3; ++i) {
        axles[i].positionZ = z[i];
        axles[i].attachHeight = -0.4f;
        axles[i].halfTrackWidth = 1.05f;
        axles[i].wheelRadius = 0.5f;
        axles[i].wheelWidth = 0.35f;
        axles[i].suspensionMinLength = 0.3f;
        axles[i].suspensionMaxLength = 0.6f;
        axles[i].suspensionFrequency = 1.8f;
        axles[i].suspensionDamping = 0.6f;
    }
    axles[0].maxSteerAngleDeg = 35.0f; // ön direksiyon
    axles[0].driven = false;
    axles[1].driven = true;
    axles[2].driven = true;
    axles[1].maxHandBrakeTorque = 8000.0f;
    axles[2].maxHandBrakeTorque = 8000.0f;
    return axles;
}

// Tank benzeri kompakt paletli: kısa gövde, çok akslı, dönebilsin diye
// düşük yanal sürtünme. Uzun/dar kamyon geometrisi skid-steer ile dönemez.
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
        axles[i].maxSteerAngleDeg = 0.0f;
        axles[i].trackLateralFriction = 0.8f;
        axles[i].driven = true;
    }
    return axles;
}

static void Settle(TestWorld& world, VehicleSystem& v, float seconds) {
    const float dt = 1.0f / 60.0f;
    const int steps = static_cast<int>(seconds / dt);
    for (int i = 0; i < steps; ++i) {
        v.SetDriverInput(0.0f, 0.0f, 0.0f, 1.0f); // el freni: yerinde otursun
        world.Step(dt);
    }
}

int main() {
    TestWorld world;
    world.AddGround();

    VehicleChassisConfig chassis;
    chassis.halfExtents = Vec3{1.25f, 0.7f, 5.0f};
    chassis.mass = 12000.0f;
    chassis.position = Vec3{0, 2.0f, 0};

    EngineConfig engine;
    TransmissionConfig trans;

    const JPH::ObjectLayer movingLayer = Layers::MOVING;
    const float dt = 1.0f / 60.0f;

    // ── Tekerlekli: süspansiyon oturması ────────────────────────────
    printf("[Tekerlekli - suspansiyon]\n");
    {
        VehicleSystem truck;
        truck.CreateWheeled(world.physics, chassis, MakeTruckAxles(), engine, trans, movingLayer);
        CHECK(truck.Kind() == VehicleKind::Wheeled, "tekerlekli arac olusturuldu");

        Settle(world, truck, 2.0f);
        const float restY = truck.GetPosition().y;
        // Gövde merkezi (COM aşağı alınmış) zeminin üstünde makul yükseklikte durmalı
        CHECK(restY > 0.2f && restY < 2.0f, "arac zemine oturdu (dusmedi/batmadi)");
        CHECK(std::fabs(truck.GetForwardSpeed()) < 0.5f, "el freniyle yerinde duruyor");
    }

    // ── Tekerlekli: ileri hızlanma ──────────────────────────────────
    printf("[Tekerlekli - hizlanma]\n");
    {
        VehicleSystem truck;
        truck.CreateWheeled(world.physics, chassis, MakeTruckAxles(), engine, trans, movingLayer);
        Settle(world, truck, 1.5f);

        const float startZ = truck.GetPosition().z;
        for (int i = 0; i < 60 * 6; ++i) { // 6 saniye tam gaz
            truck.SetDriverInput(1.0f, 0.0f, 0.0f, 0.0f);
            world.Step(dt);
        }
        const float speed = truck.GetForwardSpeed();
        const float traveled = truck.GetPosition().z - startZ;
        printf("  bilgi: ileri hiz = %.2f m/s, kat edilen = %.2f m\n", speed, traveled);

        CHECK(speed > 1.5f, "gaz verince ileri hizlandi (>1.5 m/s)");
        CHECK(traveled > 3.0f, "arac belirgin sekilde ilerledi (>3m)");
    }

    // ── Tekerlekli: direksiyon yönü değiştirir ──────────────────────
    printf("[Tekerlekli - direksiyon]\n");
    {
        VehicleSystem truck;
        truck.CreateWheeled(world.physics, chassis, MakeTruckAxles(), engine, trans, movingLayer);
        Settle(world, truck, 1.5f);

        const float startHeading = truck.GetHeading();
        for (int i = 0; i < 60 * 5; ++i) { // gaz + tam sağ direksiyon
            truck.SetDriverInput(0.6f, 1.0f, 0.0f, 0.0f);
            world.Step(dt);
        }
        const float headingDelta = std::fabs(truck.GetHeading() - startHeading);
        printf("  bilgi: yon degisimi = %.3f rad\n", headingDelta);
        CHECK(headingDelta > 0.1f, "direksiyon aracin yonunu degistirdi");
    }

    // ── Paletli: ileri hareket + diferansiyel dönüş ─────────────────
    printf("[Paletli - hareket ve donus]\n");
    {
        // Tank benzeri kompakt gövde: kısa ve dönebilir
        VehicleChassisConfig tankChassis = chassis;
        tankChassis.halfExtents = Vec3{1.3f, 0.6f, 2.6f};

        VehicleSystem tank;
        tank.CreateTracked(world.physics, tankChassis, MakeTankAxles(), engine, trans, movingLayer);
        CHECK(tank.Kind() == VehicleKind::Tracked, "paletli arac olusturuldu");

        Settle(world, tank, 1.5f);

        const float startZ = tank.GetPosition().z;
        for (int i = 0; i < 60 * 5; ++i) {
            tank.SetDriverInput(1.0f, 0.0f, 0.0f, 0.0f);
            world.Step(dt);
        }
        const float traveled = tank.GetPosition().z - startZ;
        printf("  bilgi: paletli kat edilen = %.2f m\n", traveled);
        CHECK(traveled > 2.0f, "paletli arac ileri hareket etti (>2m)");

        const float headingBefore = tank.GetHeading();
        for (int i = 0; i < 60 * 4; ++i) { // diferansiyel donus (bir palet yavas)
            tank.SetDriverInput(0.8f, 1.0f, 0.0f, 0.0f);
            world.Step(dt);
        }
        const float headingDelta = std::fabs(tank.GetHeading() - headingBefore);
        printf("  bilgi: paletli yon degisimi = %.3f rad\n", headingDelta);
        CHECK(headingDelta > 0.1f, "diferansiyel donus paletli yonunu degistirdi");
    }

    if (g_failCount == 0) {
        printf("TEST BASARILI: arac fizigi (tekerlekli + paletli) dogru calisiyor.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
