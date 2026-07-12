// R4 gercekcilik dogrulama testi:
//   1) Arac hava direnci: dragCoefficient>0 olan gövde, ayni baslangic
//      hiziyla havada dragCoefficient=0 olana gore belirgin yavaslar.
//   2) Balistik spin drift: spinDriftAccel>0 olan mermi yanal sapar,
//      spinDriftAccel=0 olan sapmaz; aynı atis tekrar edilince ayni sonuc
//      (determinizm).
//   3) Su derinligine bagli direnc: derin suya batirilmis iki ayni kutudan
//      depthDragMultiplierMax'i yuksek olan hizini daha hizli kaybeder.
//   4) Halat kopmasi: ani guclu darbeyle gerginlik esigini asan eklem
//      kopar (maxTensionN>0); ayni darbe maxTensionN=0 (devre disi) halatta
//      hicbir eklemi kirmaz.

#include "JoltTestWorld.h"
#include "ballistics/BallisticsSystem.h"
#include "buoyancy/BuoyancySystem.h"
#include "material_db/MaterialDatabase.h"
#include "rope/RopeSystem.h"
#include "vehicle/VehicleSystem.h"

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>

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

static std::vector<AxleConfig> MakeSimpleAxles() {
    std::vector<AxleConfig> axles(2);
    const float z[2] = {2.0f, -2.0f};
    for (int i = 0; i < 2; ++i) {
        axles[i].positionZ = z[i];
        axles[i].attachHeight = -0.4f;
        axles[i].halfTrackWidth = 1.0f;
        axles[i].wheelRadius = 0.4f;
        axles[i].wheelWidth = 0.3f;
        axles[i].suspensionMinLength = 0.2f;
        axles[i].suspensionMaxLength = 0.4f;
        axles[i].driven = true;
    }
    return axles;
}

static JPH::BodyID AddDynamicBox(TestWorld& world, JPH::RVec3 pos) {
    JPH::BodyCreationSettings settings(new JPH::BoxShape(JPH::Vec3(0.5f, 0.5f, 0.5f)), pos,
                                       JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic,
                                       Layers::MOVING);
    return world.physics.GetBodyInterface().CreateAndAddBody(settings, JPH::EActivation::Activate);
}

int main() {
    TestWorld world;
    const float dt = 1.0f / 60.0f;

    // ── 1) Arac hava direnci ─────────────────────────────────────────
    printf("[VehicleSystem::ApplyAeroDrag]\n");
    {
        // Not: gercekci agir vasita kutlesiyle (12000 kg) hava direncinin
        // birkac saniyede yarattigi hiz farki olcum gurultusune yakin kalir
        // (fizigi dogru ama test icin yavas) -- burada hafif bir govde
        // kutlesiyle ayni Cd/A formulu izole edilip hizlica dogrulanir.
        VehicleChassisConfig chassisNoDrag;
        chassisNoDrag.position = Vec3{-200, 50, 0};
        chassisNoDrag.mass = 800.0f;
        chassisNoDrag.dragCoefficient = 0.0f;

        VehicleChassisConfig chassisDrag;
        chassisDrag.position = Vec3{200, 50, 0};
        chassisDrag.mass = 800.0f;
        chassisDrag.dragCoefficient = 1.5f;
        chassisDrag.frontalAreaM2 = 10.0f;

        EngineConfig engine;
        TransmissionConfig trans;

        VehicleSystem noDrag;
        noDrag.CreateWheeled(world.physics, chassisNoDrag, MakeSimpleAxles(), engine, trans,
                             Layers::MOVING);
        VehicleSystem withDrag;
        withDrag.CreateWheeled(world.physics, chassisDrag, MakeSimpleAxles(), engine, trans,
                               Layers::MOVING);

        world.physics.GetBodyInterface().SetLinearVelocity(noDrag.GetBodyID(),
                                                           JPH::Vec3(30.0f, 0, 0));
        world.physics.GetBodyInterface().SetLinearVelocity(withDrag.GetBodyID(),
                                                           JPH::Vec3(30.0f, 0, 0));

        for (int i = 0; i < 120; ++i) {
            noDrag.SetDriverInput(0, 0, 0, 0);
            withDrag.SetDriverInput(0, 0, 0, 0);
            noDrag.ApplyAeroDrag(dt);
            withDrag.ApplyAeroDrag(dt);
            world.Step(dt);
        }

        const float speedNoDrag = noDrag.GetLinearVelocity().x;
        const float speedWithDrag = withDrag.GetLinearVelocity().x;
        CHECK(speedWithDrag < speedNoDrag - 5.0f,
              "hava direnci olan govde belirgin sekilde daha yavas");
        CHECK(speedNoDrag > 25.0f, "surtunmesiz govde hizini byk oranda koruyor");
    }

    // ── 2) Balistik spin drift ────────────────────────────────────────
    printf("[BallisticsSystem spin drift]\n");
    {
        MaterialDatabase db = MaterialDatabase::CreateDefault();
        BallisticsSystem ballistics(world.physics, db);

        BulletParams baseline;
        baseline.spinDriftAccel = 0.0f;
        BulletParams drifting = baseline;
        drifting.spinDriftAccel = 3.0f;

        const Vec3 origin{-1000, 500, -1000}; // uzakta, carpisma olmasin
        const Vec3 dir{0, 0, 1};

        BulletSimResult resultBase = ballistics.Fire(baseline, origin, dir, 1.0f);
        BulletSimResult resultDrift1 = ballistics.Fire(drifting, origin, dir, 1.0f);
        BulletSimResult resultDrift2 = ballistics.Fire(drifting, origin, dir, 1.0f);

        const float baseDriftX = std::fabs(resultBase.finalPosition.x - origin.x);
        const float driftX = std::fabs(resultDrift1.finalPosition.x - origin.x);

        CHECK(baseDriftX < 0.01f, "spinDriftAccel=0 yanal sapma yok");
        CHECK(driftX > 0.5f, "spinDriftAccel>0 belirgin yanal sapma uretiyor");
        CHECK(resultDrift1.finalPosition.x == resultDrift2.finalPosition.x,
              "ayni atis tekrarlaninca bit-birebir ayni sapma (determinizm)");
    }

    // ── 3) Su derinligine bagli direnc ────────────────────────────────
    printf("[BuoyancySystem derinlik direnci]\n");
    {
        world.AddStaticBox(JPH::RVec3(0.0, -30.0, 0.0), JPH::Vec3(500.0f, 1.0f, 500.0f), 0);

        BuoyancySystem buoyancy;

        WaterVolume shallowDrag;
        shallowDrag.surfaceY = 0.0f;
        shallowDrag.boundsMin = Vec3{-300, -50, -50};
        shallowDrag.boundsMax = Vec3{-100, 50, 50};
        shallowDrag.buoyancyFactor = 1.0f;
        shallowDrag.linearDrag = 0.5f;
        shallowDrag.depthDragMultiplierMax = 1.0f; // devre disi (varsayilan)
        buoyancy.AddVolume(shallowDrag);

        WaterVolume deepDrag = shallowDrag;
        deepDrag.boundsMin = Vec3{100, -50, -50};
        deepDrag.boundsMax = Vec3{300, 50, 50};
        deepDrag.depthDragMultiplierMax = 6.0f;
        deepDrag.depthSaturationM = 2.0f;
        buoyancy.AddVolume(deepDrag);

        JPH::BodyID lowDragBox = AddDynamicBox(world, JPH::RVec3(-200, -5, 0));
        JPH::BodyID highDragBox = AddDynamicBox(world, JPH::RVec3(200, -5, 0));

        buoyancy.TrackBody(lowDragBox);
        buoyancy.TrackBody(highDragBox);

        JPH::BodyInterface& bi = world.physics.GetBodyInterface();
        bi.SetLinearVelocity(lowDragBox, JPH::Vec3(5, 0, 0));
        bi.SetLinearVelocity(highDragBox, JPH::Vec3(5, 0, 0));

        for (int i = 0; i < 60; ++i) {
            buoyancy.Update(world.physics, dt);
            world.Step(dt);
        }

        const float lowDragSpeed = bi.GetLinearVelocity(lowDragBox).Length();
        const float highDragSpeed = bi.GetLinearVelocity(highDragBox).Length();
        CHECK(highDragSpeed < lowDragSpeed - 0.2f,
              "derinlik direnci yuksek hacimde kutu belirgin daha cok yavasliyor");
    }

    // ── 4) Halat kopmasi ───────────────────────────────────────────────
    printf("[RopeSystem kopma]\n");
    {
        JPH::BodyID ceilingA =
            world.AddStaticBox(JPH::RVec3(0, 100.0, 0), JPH::Vec3(1, 0.5f, 1), 0);
        JPH::BodyID ceilingB =
            world.AddStaticBox(JPH::RVec3(20, 100.0, 0), JPH::Vec3(1, 0.5f, 1), 0);

        RopeConfig breakableCfg;
        breakableCfg.segmentCount = 4;
        breakableCfg.totalLength = 2.0f;
        breakableCfg.maxTensionN = 500.0f;
        RopeSystem breakable(breakableCfg);
        breakable.Create(world.physics, Vec3{0, 99.5f, 0}, Vec3{2.0f, 99.5f, 0}, Layers::MOVING);
        breakable.AttachStartTo(ceilingA, Vec3{0, 99.5f, 0});

        RopeConfig unbreakableCfg = breakableCfg;
        unbreakableCfg.maxTensionN = 0.0f; // devre disi
        RopeSystem unbreakable(unbreakableCfg);
        unbreakable.Create(world.physics, Vec3{20, 99.5f, 0}, Vec3{22.0f, 99.5f, 0},
                           Layers::MOVING);
        unbreakable.AttachStartTo(ceilingB, Vec3{20, 99.5f, 0});

        JPH::BodyInterface& bi = world.physics.GetBodyInterface();
        const JPH::Vec3 shock(0, -2000.0f, 0);
        bi.AddImpulse(breakable.GetSegmentBodyID(breakable.SegmentCount() - 1), shock);
        bi.AddImpulse(unbreakable.GetSegmentBodyID(unbreakable.SegmentCount() - 1), shock);

        world.Step(dt);
        std::vector<int> broken;
        breakable.Update(dt, &broken);
        unbreakable.Update(dt);

        CHECK(!broken.empty(), "ani guclu darbeyle gerginlik esigini asan eklem kopuyor");
        bool anyUnbreakableBroken = false;
        for (int i = 0; i < breakableCfg.segmentCount - 1; ++i)
            if (unbreakable.IsJointBroken(i)) anyUnbreakableBroken = true;
        CHECK(!anyUnbreakableBroken, "maxTensionN=0 (devre disi) halat asla kopmuyor");
    }

    if (g_failCount == 0) {
        printf("TEST BASARILI: R4 -- arac aero, spin drift, derinlik direnci, halat kopmasi.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
