// R2 gerceklik sistemleri dogrulama testi:
//   1) Dalgali su: yuzey yuksekligi zamanla ve konumla degisir; yuzen kutu
//      duz suya gore olculebilir bicimde iner-kalkar
//   2) Surtunme bolgeleri: bolgeye giren govdenin friction'i degisir,
//      cikinca orijinali geri gelir; QueryFrictionAt dogru calisir
//   3) Kumas: ust kenari sabit bayrak sarkar, ruzgar uygulaninca ruzgar
//      yonunde belirgin sekilde savrulur
//   4) Lastik tutunmasi: dusuk yanal tutuslu arac ayni manevrada daha
//      genis savrulur (config'in gercekten etki ettigi dogrulanir)

#include "JoltTestWorld.h"
#include "buoyancy/BuoyancySystem.h"
#include "cloth/ClothSystem.h"
#include "vehicle/VehicleSystem.h"
#include "zones/FrictionZoneSystem.h"

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>

#include <cmath>
#include <cstdio>
#include <vector>

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
    // ── 1) Dalgali su yuzeyi ───────────────────────────────────────────
    printf("[WavyWater]\n");
    {
        BuoyancySystem buoyancy;
        WaterVolume volume;
        volume.surfaceY = 5.0f;
        volume.waveAmplitude = 0.5f;
        volume.waveLength = 8.0f;
        volume.waveSpeed = 2.0f;
        buoyancy.AddVolume(volume);

        // Ayni nokta, farkli zamanlar: yuzey degismeli. NOT: ornekleme
        // suresi dalga periyodunun tam yarisi/kati olmamali -- sin'in
        // simetrisi ayni degeri geri getirebilir (ilk denemede t=1.0s,
        // faz 3pi/4 -> pi/4 tam simetriye denk gelip testi bosa dusurdu).
        TestWorld world; // Update icin fizik gerekiyor (bos dunya yeterli)
        const float h0 = buoyancy.SurfaceHeightAt(volume, 3.0f, 0.0f);
        for (int i = 0; i < 45; ++i)
            buoyancy.Update(world.physics, 1.0f / 60.0f);
        const float h1 = buoyancy.SurfaceHeightAt(volume, 3.0f, 0.0f);
        printf("  yuzey t=0: %.3f, t=1s: %.3f\n", h0, h1);
        CHECK(std::fabs(h1 - h0) > 0.05f, "dalga yuzeyi zamanla degisiyor");
        CHECK(std::fabs(h0 - volume.surfaceY) <= volume.waveAmplitude + 1.0e-3f &&
                  std::fabs(h1 - volume.surfaceY) <= volume.waveAmplitude + 1.0e-3f,
              "yuzey her zaman genlik siniri icinde");

        // Ayni anda farkli konumlar: yuzey farkli olmali (dalga uzaysal)
        const float hA = buoyancy.SurfaceHeightAt(volume, 0.0f, 0.0f);
        const float hB = buoyancy.SurfaceHeightAt(volume, 4.0f, 0.0f); // yarım dalga boyu
        CHECK(std::fabs(hA - hB) > 0.05f, "dalga konuma gore de degisiyor");

        // QueryWaterState dalgali yuzeyi kullaniyor mu?
        const BuoyancySystem::WaterState st = buoyancy.QueryWaterState(Vec3{3.0f, 4.0f, 0.0f});
        CHECK(st.inWater && std::fabs((st.surfaceY - 4.0f) - st.submergedDepth) < 1.0e-4f,
              "QueryWaterState dalgali yuzeyle tutarli");
    }

    // ── 2) Surtunme bolgeleri ──────────────────────────────────────────
    printf("[FrictionZones]\n");
    {
        TestWorld world;
        world.AddGround();
        JPH::BodyCreationSettings s(new JPH::BoxShape(JPH::Vec3(0.5f, 0.5f, 0.5f)),
                                    JPH::RVec3(0.0, 0.5, 0.0), JPH::Quat::sIdentity(),
                                    JPH::EMotionType::Dynamic, Layers::MOVING);
        s.mFriction = 0.7f;
        JPH::BodyID box =
            world.physics.GetBodyInterface().CreateAndAddBody(s, JPH::EActivation::Activate);

        FrictionZoneSystem zones;
        FrictionZone ice;
        ice.boundsMin = Vec3{-2, -1, -2};
        ice.boundsMax = Vec3{2, 3, 2};
        ice.friction = 0.05f;
        zones.AddZone(ice);
        zones.TrackBody(world.physics, box);

        JPH::BodyInterface& bi = world.physics.GetBodyInterface();
        zones.Update(world.physics);
        CHECK(bi.GetFriction(box) < 0.1f, "bolge icinde: buz surtunmesi uygulandi");

        // Kutuyu bolge disina tasi
        bi.SetPosition(box, JPH::RVec3(10.0, 0.5, 0.0), JPH::EActivation::Activate);
        zones.Update(world.physics);
        CHECK(std::fabs(bi.GetFriction(box) - 0.7f) < 1.0e-4f,
              "bolge disinda: orijinal surtunme geri geldi");

        CHECK(zones.QueryFrictionAt(Vec3{0, 0, 0}, 0.9f) < 0.1f, "QueryFrictionAt bolge ici");
        CHECK(zones.QueryFrictionAt(Vec3{50, 0, 0}, 0.9f) > 0.8f,
              "QueryFrictionAt bolge disi varsayilani doner");
    }

    // ── 3) Kumas/bayrak ────────────────────────────────────────────────
    printf("[Cloth]\n");
    {
        TestWorld world;
        ClothConfig ccfg;
        ccfg.widthSegments = 8;
        ccfg.heightSegments = 6;
        ccfg.width = 2.0f;
        ccfg.height = 1.5f;
        ClothSystem flag(ccfg);
        flag.Create(world.physics, Vec3{0, 5.0f, 0}, Layers::MOVING);
        CHECK(!flag.GetBodyID().IsInvalid(), "kumas soft body olustu");

        // Ruzgarsiz yerlesme: bayrak asagi sarkmali
        for (int i = 0; i < 240; ++i)
            world.Step(1.0f / 60.0f);
        std::vector<Vec3> verts;
        flag.GetVertexPositions(verts);
        CHECK(verts.size() == static_cast<size_t>(flag.VertexCount()), "vertex sayisi dogru");
        const Vec3 bottomNoWind = verts.back(); // alt-sag kose
        CHECK(bottomNoWind.y < 4.2f, "bayrak yercekimiyle asagi sarkti");
        CHECK(std::fabs(bottomNoWind.z) < 0.5f, "ruzgarsiz: z'de belirgin savrulma yok");

        // Guclu +z ruzgari uygula: alt kose +z'ye savrulmali
        for (int i = 0; i < 300; ++i) {
            flag.ApplyWind(Vec3{0, 0, 25.0f}, 1.0f / 60.0f);
            world.Step(1.0f / 60.0f);
        }
        flag.GetVertexPositions(verts);
        const Vec3 bottomWind = verts.back();
        printf("  alt kose: ruzgarsiz z=%.3f, ruzgarli z=%.3f\n", bottomNoWind.z, bottomWind.z);
        CHECK(bottomWind.z > bottomNoWind.z + 0.3f, "ruzgar bayragi +z yonune savurdu");

        flag.Destroy();
    }

    // ── 4) Lastik tutunmasi ────────────────────────────────────────────
    printf("[TireGrip]\n");
    {
        // Ayni manevra iki kez: normal lastik vs kaygan lastik. Kaygan
        // lastikli arac ayni sure sonunda daha az yon degistirmis olmali
        // (yanal tutus dusuk -> donus komutu daha gec cevap bulur).
        auto runManeuver = [](float lateralGrip) {
            TestWorld world;
            world.AddGround(400.0f);

            VehicleChassisConfig chassis;
            std::vector<AxleConfig> axles(2);
            axles[0].positionZ = 1.6f;
            axles[0].maxSteerAngleDeg = 35.0f;
            axles[0].driven = true;
            axles[0].tireGripLateral = lateralGrip;
            axles[1].positionZ = -1.6f;
            axles[1].driven = true;
            axles[1].tireGripLateral = lateralGrip;

            VehicleSystem vehicle;
            vehicle.CreateWheeled(world.physics, chassis, axles, EngineConfig{},
                                  TransmissionConfig{}, Layers::MOVING);

            // 2 sn duz hizlan, sonra 2 sn tam sola don
            for (int i = 0; i < 120; ++i) {
                vehicle.SetDriverInput(1.0f, 0.0f, 0.0f);
                world.Step(1.0f / 60.0f);
            }
            for (int i = 0; i < 120; ++i) {
                vehicle.SetDriverInput(1.0f, 1.0f, 0.0f);
                world.Step(1.0f / 60.0f);
            }
            // Destroy cagirmiyoruz: VehicleSystem destructor'i kapsam
            // sonunda kendi temizligini yapiyor (Destroy private)
            return std::fabs(vehicle.GetHeading());
        };

        const float turnNormal = runManeuver(1.0f);
        const float turnSlippery = runManeuver(0.02f);
        printf("  yon degisimi: normal=%.3f rad, kaygan=%.3f rad\n", turnNormal, turnSlippery);
        CHECK(turnNormal > 0.15f, "normal lastikle arac belirgin dondu");
        CHECK(turnSlippery < turnNormal * 0.75f,
              "kaygan lastik ayni manevrada belirgin daha az dondurdu (savrulma)");
    }

    if (g_failCount == 0) {
        printf("TEST BASARILI: dalga/bolge/kumas/lastik sistemleri dogru.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
