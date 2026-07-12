// Sistem-bazlı performans ölçümü (ctest'e kayıtlı DEĞİL -- başarısızlık
// üretmez). Her fizik alt sistemi KENDİ gerçekçi senaryosunda, kendi
// AlazForgeContext'inde (Jolt Factory ref-sayımlı olduğu için sıralı --
// iç içe değil -- kurulup yıkılır) ayrı ayrı ölçülür ve sonuçlar bir özet
// tabloda raporlanır. CI loglarında sürelerin zaman içinde izlenmesi
// performans regresyonlarını erken yakalar.
//
// NOT: Bu sayılar CI'nin JENERİK runner CPU'sunda ölçülür -- gerçek oyun
// donanımında (özellikle çok çekirdekli/daha hızlı tek çekirdek performanslı
// CPU'larda) farklı olacaktır. Buradaki değer MUTLAK FPS değil, sistemler
// ARASI GÖRECELİ maliyet karşılaştırmasıdır -- "hangi sistem hangi ölçekte
// pahalı" sorusuna cevap verir.

#include "buoyancy/BuoyancySystem.h"
#include "character/CharacterController.h"
#include "cloth/ClothSystem.h"
#include "context/AlazForgeContext.h"
#include "destructible/DestructibleStructureSystem.h"
#include "material_db/MaterialDatabase.h"
#include "ragdoll/RagdollInstance.h"
#include "ragdoll/RagdollSkeleton.h"
#include "rope/RopeSystem.h"
#include "vehicle/VehicleSystem.h"
#include "weapons/ExplosionSystem.h"
#include "zones/FrictionZoneSystem.h"

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

using namespace alazforge;

namespace {

struct BenchResult {
    std::string system; // hangi .dll / alt sistem
    std::string scenario;
    int entityCount = 0;
    double avgMs = 0.0; // olculen cagri basina ortalama sure
    double maxMs = 0.0;
};

std::vector<BenchResult> g_results;

template <typename Fn>
BenchResult Measure(const std::string& system, const std::string& scenario, int entityCount,
                    int iterations, Fn&& fn) {
    double totalMs = 0.0, maxMs = 0.0;
    for (int i = 0; i < iterations; ++i) {
        auto t0 = std::chrono::steady_clock::now();
        fn();
        auto t1 = std::chrono::steady_clock::now();
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        totalMs += ms;
        maxMs = std::max(maxMs, ms);
    }
    BenchResult r{system, scenario, entityCount, totalMs / iterations, maxMs};
    printf("[%s] %s (varlik=%d) -> ort=%.4f ms, maks=%.4f ms (~%.0f cagri/sn)\n", r.system.c_str(),
           r.scenario.c_str(), r.entityCount, r.avgMs, r.maxMs,
           r.avgMs > 0.0 ? 1000.0 / r.avgMs : 0.0);
    g_results.push_back(r);
    return r;
}

JPH::BodyID AddGround(JPH::BodyInterface& bi, JPH::ObjectLayer nonMoving, float halfSize = 300.0f) {
    JPH::BodyCreationSettings s(new JPH::BoxShape(JPH::Vec3(halfSize, 1.0f, halfSize)),
                                JPH::RVec3(0, -1, 0), JPH::Quat::sIdentity(),
                                JPH::EMotionType::Static, nonMoving);
    return bi.CreateAndAddBody(s, JPH::EActivation::DontActivate);
}

std::vector<AxleConfig> SimpleTruckAxles() {
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

// ── physics.dll: genel rijit gövde (referans/taban çizgi) ────────────────
void BenchRigidBody() {
    AlazForgeContext::Config cfg;
    cfg.maxBodies = 8192;
    cfg.maxBodyPairs = 16384;
    cfg.maxContactConstraints = 16384;
    AlazForgeContext ctx(cfg);
    JPH::BodyInterface& bi = ctx.Physics().GetBodyInterface();

    AddGround(bi, cfg.nonMovingLayer, 100.0f);

    constexpr int kCountX = 10, kCountY = 12, kCountZ = 10;
    JPH::Ref<JPH::BoxShape> boxShape = new JPH::BoxShape(JPH::Vec3(0.4f, 0.4f, 0.4f));
    for (int y = 0; y < kCountY; ++y)
        for (int z = 0; z < kCountZ; ++z)
            for (int x = 0; x < kCountX; ++x) {
                JPH::BodyCreationSettings s(
                    boxShape, JPH::RVec3(x * 1.0 - 5.0, 1.0 + y * 1.1, z * 1.0 - 5.0),
                    JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, cfg.movingLayer);
                bi.CreateAndAddBody(s, JPH::EActivation::Activate);
            }
    ctx.Physics().OptimizeBroadPhase();

    Measure("physics.dll", "1200 dinamik kutu yigini (Step)", kCountX * kCountY * kCountZ, 300,
            [&] { ctx.Step(1.0f / 60.0f); });
}

// ── physics.dll: VehicleSystem (araç fiziği) ──────────────────────────────
void BenchVehicle() {
    AlazForgeContext::Config cfg;
    cfg.maxBodies = 4096;
    AlazForgeContext ctx(cfg);
    AddGround(ctx.Physics().GetBodyInterface(), cfg.nonMovingLayer);

    constexpr int kCount = 20;
    std::vector<VehicleSystem> vehicles(kCount);
    EngineConfig engine;
    TransmissionConfig trans;
    for (int i = 0; i < kCount; ++i) {
        VehicleChassisConfig chassis;
        chassis.position = Vec3{static_cast<float>(i) * 15.0f - 150.0f, 2.0f, 0};
        vehicles[static_cast<size_t>(i)].CreateWheeled(ctx.Physics(), chassis, SimpleTruckAxles(),
                                                       engine, trans, cfg.movingLayer);
    }
    ctx.Physics().OptimizeBroadPhase();

    Measure("physics.dll", "20 tekerlekli arac (tam gaz, Step dahil)", kCount, 180, [&] {
        for (auto& v : vehicles)
            v.SetDriverInput(1.0f, 0.0f, 0.0f, 0.0f);
        ctx.Step(1.0f / 60.0f);
    });
}

// ── physics.dll: BallisticsSystem (mermi ateşleme) ────────────────────────
void BenchBallistics() {
    AlazForgeContext::Config cfg;
    AlazForgeContext ctx(cfg);
    AddGround(ctx.Physics().GetBodyInterface(), cfg.nonMovingLayer);
    ctx.Physics().OptimizeBroadPhase();

    constexpr int kShotsPerCall = 500;
    BulletParams bullet;
    const auto result =
        Measure("physics.dll", "500 mermi atisi (tek toplu cagri)", kShotsPerCall, 20, [&] {
            for (int i = 0; i < kShotsPerCall; ++i) {
                ctx.Ballistics().Fire(bullet, Vec3{0, 50, 0}, Vec3{0, -1, 0}, 3.0f);
            }
        });
    printf("  -> mermi basina ortalama: %.5f ms\n", result.avgMs / kShotsPerCall);
}

// ── character.dll: CharacterController ────────────────────────────────────
void BenchCharacter() {
    AlazForgeContext::Config cfg;
    cfg.maxBodies = 4096;
    AlazForgeContext ctx(cfg);
    AddGround(ctx.Physics().GetBodyInterface(), cfg.nonMovingLayer);

    constexpr int kCount = 100;
    CharacterControllerConfig charCfg;
    std::vector<std::unique_ptr<CharacterController>> characters;
    characters.reserve(kCount);
    for (int i = 0; i < kCount; ++i) {
        characters.push_back(std::make_unique<CharacterController>(charCfg));
        const float x = static_cast<float>(i % 10) * 2.0f - 10.0f;
        const float z = static_cast<float>(i / 10) * 2.0f - 10.0f;
        characters.back()->Spawn(ctx.Physics(), Vec3{x, 1.0f, z}, cfg.movingLayer);
    }
    ctx.Physics().OptimizeBroadPhase();

    Measure("character.dll", "100 karakter yurume (Update + Step)", kCount, 180, [&] {
        for (auto& c : characters) {
            c->SetMoveInput(Vec3{0, 0, 1}, false);
            c->Update(1.0f / 60.0f);
        }
        ctx.Step(1.0f / 60.0f);
    });
}

// ── cloth.dll: ClothSystem (SoftBody) ─────────────────────────────────────
void BenchCloth() {
    AlazForgeContext::Config cfg;
    cfg.maxBodies = 4096;
    AlazForgeContext ctx(cfg);

    constexpr int kCount = 20;
    ClothConfig clothCfg; // 12x10 = 120 vertex / kumas
    std::vector<std::unique_ptr<ClothSystem>> cloths;
    cloths.reserve(kCount);
    for (int i = 0; i < kCount; ++i) {
        cloths.push_back(std::make_unique<ClothSystem>(clothCfg));
        const float x = static_cast<float>(i) * 4.0f - 40.0f;
        cloths.back()->Create(ctx.Physics(), Vec3{x, 10.0f, 0}, cfg.movingLayer);
    }
    ctx.Physics().OptimizeBroadPhase();

    Measure("cloth.dll", "20 kumas parcasi (12x10=2400 vertex, ApplyWind+Step)",
            kCount * clothCfg.widthSegments * clothCfg.heightSegments, 120, [&] {
                for (auto& c : cloths)
                    c->ApplyWind(Vec3{3, 0, 0}, 1.0f / 60.0f);
                ctx.Step(1.0f / 60.0f);
            });
}

// ── rope.dll: RopeSystem ───────────────────────────────────────────────────
void BenchRope() {
    AlazForgeContext::Config cfg;
    cfg.maxBodies = 4096;
    AlazForgeContext ctx(cfg);

    constexpr int kCount = 30;
    RopeConfig ropeCfg; // 16 segment
    std::vector<std::unique_ptr<RopeSystem>> ropes;
    ropes.reserve(kCount);
    for (int i = 0; i < kCount; ++i) {
        ropes.push_back(std::make_unique<RopeSystem>(ropeCfg));
        const float x = static_cast<float>(i) * 2.0f - 30.0f;
        ropes.back()->Create(ctx.Physics(), Vec3{x, 20.0f, 0}, Vec3{x + 1.5f, 20.0f, 0},
                             cfg.movingLayer);
    }
    ctx.Physics().OptimizeBroadPhase();

    Measure("rope.dll", "30 halat (16 segment = 480 govde, Step)", kCount * ropeCfg.segmentCount,
            120, [&] { ctx.Step(1.0f / 60.0f); });
}

// ── ragdoll.dll: RagdollInstance ──────────────────────────────────────────
void BenchRagdoll() {
    AlazForgeContext::Config cfg;
    cfg.maxBodies = 4096;
    AlazForgeContext ctx(cfg);
    AddGround(ctx.Physics().GetBodyInterface(), cfg.nonMovingLayer);

    RagdollSkeletonConfig skelCfg;
    RagdollBoneConfig pelvis;
    pelvis.name = "pelvis";
    pelvis.parentIndex = -1;
    pelvis.radius = 0.2f;
    pelvis.halfHeight = 0.25f;
    skelCfg.bones.push_back(pelvis);
    RagdollBoneConfig torso;
    torso.name = "torso";
    torso.parentIndex = 0;
    torso.radius = 0.2f;
    torso.halfHeight = 0.3f;
    torso.localPosition = Vec3{0, 0.55f, 0};
    skelCfg.bones.push_back(torso);
    RagdollBoneConfig limb;
    limb.name = "limb";
    limb.parentIndex = 1;
    limb.radius = 0.12f;
    limb.halfHeight = 0.25f;
    limb.localPosition = Vec3{0, 0.55f, 0};
    skelCfg.bones.push_back(limb);
    RagdollSkeleton skeleton(skelCfg);

    constexpr int kCount = 50;
    std::vector<RagdollInstance> ragdolls(kCount);
    for (int i = 0; i < kCount; ++i) {
        const float x = static_cast<float>(i % 10) * 3.0f - 15.0f;
        const float z = static_cast<float>(i / 10) * 3.0f - 7.5f;
        Transform pose;
        pose.position = Vec3{x, 5.0f, z};
        ragdolls[static_cast<size_t>(i)].Activate(ctx.Physics(), skeleton, pose, cfg.movingLayer);
    }
    ctx.Physics().OptimizeBroadPhase();

    Measure("ragdoll.dll", "50 ragdoll (3 kemik = 150 govde, Step)", kCount * 3, 120,
            [&] { ctx.Step(1.0f / 60.0f); });
}

// ── buoyancy.dll: BuoyancySystem ──────────────────────────────────────────
void BenchBuoyancy() {
    AlazForgeContext::Config cfg;
    cfg.maxBodies = 4096;
    AlazForgeContext ctx(cfg);
    JPH::BodyInterface& bi = ctx.Physics().GetBodyInterface();

    WaterVolume water;
    water.surfaceY = 0.0f;
    water.waveAmplitude = 0.3f;
    ctx.Buoyancy().AddVolume(water);

    constexpr int kCount = 300;
    for (int i = 0; i < kCount; ++i) {
        const float x = static_cast<float>(i % 20) * 3.0f - 30.0f;
        const float z = static_cast<float>(i / 20) * 3.0f - 22.5f;
        JPH::BodyCreationSettings s(new JPH::BoxShape(JPH::Vec3(0.5f, 0.5f, 0.5f)),
                                    JPH::RVec3(x, 1.0, z), JPH::Quat::sIdentity(),
                                    JPH::EMotionType::Dynamic, cfg.movingLayer);
        JPH::BodyID id = bi.CreateAndAddBody(s, JPH::EActivation::Activate);
        ctx.Buoyancy().TrackBody(id);
    }
    ctx.Physics().OptimizeBroadPhase();

    Measure("buoyancy.dll", "300 yuzen kutu (dalgali yuzey, Step)", kCount, 180,
            [&] { ctx.Step(1.0f / 60.0f); });
}

// ── zones.dll: FrictionZoneSystem ─────────────────────────────────────────
void BenchZones() {
    AlazForgeContext::Config cfg;
    cfg.maxBodies = 4096;
    AlazForgeContext ctx(cfg);
    JPH::BodyInterface& bi = ctx.Physics().GetBodyInterface();
    AddGround(bi, cfg.nonMovingLayer);

    FrictionZone ice;
    ice.boundsMin = Vec3{-200, -5, -200};
    ice.boundsMax = Vec3{200, 5, 200};
    ice.friction = 0.05f;
    ctx.Zones().AddZone(ice);

    constexpr int kCount = 500;
    for (int i = 0; i < kCount; ++i) {
        const float x = static_cast<float>(i % 25) * 2.0f - 25.0f;
        const float z = static_cast<float>(i / 25) * 2.0f - 20.0f;
        JPH::BodyCreationSettings s(new JPH::BoxShape(JPH::Vec3(0.4f, 0.4f, 0.4f)),
                                    JPH::RVec3(x, 1.0, z), JPH::Quat::sIdentity(),
                                    JPH::EMotionType::Dynamic, cfg.movingLayer);
        JPH::BodyID id = bi.CreateAndAddBody(s, JPH::EActivation::Activate);
        ctx.Zones().TrackBody(ctx.Physics(), id);
    }
    ctx.Physics().OptimizeBroadPhase();

    Measure("zones.dll", "500 izlenen govde (bolge giris/cikis, Step)", kCount, 120, [&] {
        ctx.Zones().Update(ctx.Physics());
        ctx.Step(1.0f / 60.0f);
    });
}

// ── destructible.dll: DestructibleStructureSystem ─────────────────────────
void BenchDestructible() {
    AlazForgeContext::Config cfg;
    cfg.maxBodies = 4096;
    AlazForgeContext ctx(cfg);

    const std::string saveDir =
        (std::filesystem::temp_directory_path() / "alazforge_bench_destructible").string();
    std::filesystem::remove_all(saveDir);

    MaterialDatabase materials = MaterialDatabase::CreateDefault();
    const MaterialId concrete = materials.FindByName("concrete");

    {
        DestructibleStructureSystem::Config sysCfg;
        sysCfg.savePath = saveDir;
        DestructibleStructureSystem destructible(sysCfg);

        constexpr int kSide = 30; // 30x30 = 900 parca
        StructureConfig wall;
        wall.structureId = 1;
        wall.pieces.resize(static_cast<size_t>(kSide * kSide));
        for (int row = 0; row < kSide; ++row) {
            for (int col = 0; col < kSide; ++col) {
                PieceConfig& p = wall.pieces[static_cast<size_t>(row * kSide + col)];
                p.localCenter = Vec3{static_cast<float>(col), static_cast<float>(row), 0.0f};
                p.halfExtents = Vec3{0.45f, 0.45f, 0.1f};
                p.material = concrete;
            }
        }
        destructible.RegisterStructure(ctx.Physics(), cfg.nonMovingLayer, materials, wall);
        ctx.Physics().OptimizeBroadPhase();

        // Steady-state: 900 statik parca dururken normal fizik adimi maliyeti.
        Measure("destructible.dll", "900 parcali yapi (steady-state Step)", kSide * kSide, 120,
                [&] { ctx.Step(1.0f / 60.0f); });

        // Toplu hasar: her cagrida farkli bir noktaya genis yaricapli hasar --
        // kademeli cokme (BFS) maliyetini olcer.
        int callIndex = 0;
        Measure("destructible.dll", "genis yaricapli hasar + kademeli cokme (toplu)", kSide * kSide,
                20, [&] {
                    std::vector<PieceBrokenEvent> events;
                    const float cx = static_cast<float>(callIndex % kSide);
                    destructible.ApplyDamageRadius(1, Vec3{cx, 0, 0}, 500.0f, 3.0f, events);
                    ++callIndex;
                });

        destructible.Flush();
    } // destructible burada yikilir (Flush zaten yaptik ama dtor guvenlik agi)

    std::filesystem::remove_all(saveDir);
}

// ── weapons.dll: ExplosionSystem ──────────────────────────────────────────
void BenchExplosion() {
    AlazForgeContext::Config cfg;
    cfg.maxBodies = 4096;
    AlazForgeContext ctx(cfg);
    JPH::BodyInterface& bi = ctx.Physics().GetBodyInterface();
    AddGround(bi, cfg.nonMovingLayer);

    constexpr int kCount = 500;
    for (int i = 0; i < kCount; ++i) {
        const float x = static_cast<float>(i % 25) * 1.5f - 18.0f;
        const float z = static_cast<float>(i / 25) * 1.5f - 15.0f;
        JPH::BodyCreationSettings s(new JPH::BoxShape(JPH::Vec3(0.3f, 0.3f, 0.3f)),
                                    JPH::RVec3(x, 1.0, z), JPH::Quat::sIdentity(),
                                    JPH::EMotionType::Dynamic, cfg.movingLayer);
        bi.CreateAndAddBody(s, JPH::EActivation::Activate);
    }
    ctx.Physics().OptimizeBroadPhase();

    ExplosionConfig explCfg;
    explCfg.radius = 15.0f;
    Measure("weapons.dll", "500 govdeye patlama (ExplosionSystem::Detonate, toplu)", kCount, 20,
            [&] {
                std::vector<ExplosionHit> hits;
                ExplosionSystem::Detonate(ctx.Physics(), Vec3{0, 1, 0}, explCfg, hits);
            });
}

// ── lod.dll: LODSystem (maliyet degil, TASARRUF sistemi) ─────────────────
void BenchLOD() {
    constexpr int kCount = 2000;

    // (a) LOD YOK: 2000 uzak govde her adim tam simule edilir
    double baselineMs;
    {
        AlazForgeContext::Config cfg;
        cfg.maxBodies = 8192;
        AlazForgeContext ctx(cfg);
        JPH::BodyInterface& bi = ctx.Physics().GetBodyInterface();
        AddGround(bi, cfg.nonMovingLayer, 2000.0f);
        for (int i = 0; i < kCount; ++i) {
            const float x = 1000.0f + static_cast<float>(i % 45) * 2.0f;
            const float z = static_cast<float>(i / 45) * 2.0f;
            JPH::BodyCreationSettings s(new JPH::BoxShape(JPH::Vec3(0.3f, 0.3f, 0.3f)),
                                        JPH::RVec3(x, 5.0, z), JPH::Quat::sIdentity(),
                                        JPH::EMotionType::Dynamic, cfg.movingLayer);
            bi.CreateAndAddBody(s, JPH::EActivation::Activate);
        }
        ctx.Physics().OptimizeBroadPhase();
        for (int i = 0; i < 30; ++i)
            ctx.Step(1.0f / 60.0f); // dusup yerlessinler (ilk yigilma disi birak)
        const auto r = Measure("lod.dll", "2000 uzak govde (LOD YOK, Step)", kCount, 120,
                               [&] { ctx.Step(1.0f / 60.0f); });
        baselineMs = r.avgMs;
    }

    // (b) LOD VAR: ayni sahne, ama referans noktasi (oyuncu) uzakta --
    // tum govdeler zorla uyutulur.
    double lodMs;
    {
        AlazForgeContext::Config cfg;
        cfg.maxBodies = 8192;
        AlazForgeContext ctx(cfg);
        JPH::BodyInterface& bi = ctx.Physics().GetBodyInterface();
        AddGround(bi, cfg.nonMovingLayer, 2000.0f);
        for (int i = 0; i < kCount; ++i) {
            const float x = 1000.0f + static_cast<float>(i % 45) * 2.0f;
            const float z = static_cast<float>(i / 45) * 2.0f;
            JPH::BodyCreationSettings s(new JPH::BoxShape(JPH::Vec3(0.3f, 0.3f, 0.3f)),
                                        JPH::RVec3(x, 5.0, z), JPH::Quat::sIdentity(),
                                        JPH::EMotionType::Dynamic, cfg.movingLayer);
            JPH::BodyID id = bi.CreateAndAddBody(s, JPH::EActivation::Activate);
            ctx.LOD().TrackBody(id);
        }
        ctx.Physics().OptimizeBroadPhase();
        const Vec3 playerAtOrigin{0, 0, 0}; // 1000m+ uzakta -- hepsi LOD sinirinin disinda
        for (int i = 0; i < 10; ++i)
            ctx.Step(1.0f / 60.0f, 1, &playerAtOrigin); // uyusunlar
        const auto r = Measure("lod.dll", "2000 uzak govde (LOD VAR, Step)", kCount, 120,
                               [&] { ctx.Step(1.0f / 60.0f, 1, &playerAtOrigin); });
        lodMs = r.avgMs;
    }

    printf("  -> LOD hizlanma: %.1fx (%.4f ms -> %.4f ms)\n", baselineMs / lodMs, baselineMs,
           lodMs);
}

// ── HEPSI BIRLIKTE: tek sahnede tum sistemler ayni anda ───────────────────
// Izole olcumlerin toplamı degil -- GERCEK tek Step() maliyeti (paylasilan
// broad-phase/job-sistemi overhead'i izole olculdugunde her senaryoda ayri
// ayri sayilirdi, burada bir kez).
void BenchCombinedAll() {
    AlazForgeContext::Config cfg;
    cfg.maxBodies = 8192;
    cfg.maxBodyPairs = 16384;
    cfg.maxContactConstraints = 16384;
    AlazForgeContext ctx(cfg);
    JPH::BodyInterface& bi = ctx.Physics().GetBodyInterface();

    AddGround(bi, cfg.nonMovingLayer, 300.0f);

    // Arac (20)
    constexpr int kVehicles = 20;
    std::vector<VehicleSystem> vehicles(kVehicles);
    EngineConfig engine;
    TransmissionConfig trans;
    for (int i = 0; i < kVehicles; ++i) {
        VehicleChassisConfig chassis;
        chassis.position = Vec3{static_cast<float>(i) * 15.0f - 150.0f, 2.0f, -100.0f};
        vehicles[static_cast<size_t>(i)].CreateWheeled(ctx.Physics(), chassis, SimpleTruckAxles(),
                                                       engine, trans, cfg.movingLayer);
    }

    // Karakter (100)
    constexpr int kCharacters = 100;
    CharacterControllerConfig charCfg;
    std::vector<std::unique_ptr<CharacterController>> characters;
    characters.reserve(kCharacters);
    for (int i = 0; i < kCharacters; ++i) {
        characters.push_back(std::make_unique<CharacterController>(charCfg));
        const float x = static_cast<float>(i % 10) * 2.0f - 10.0f;
        const float z = static_cast<float>(i / 10) * 2.0f + 50.0f;
        characters.back()->Spawn(ctx.Physics(), Vec3{x, 1.0f, z}, cfg.movingLayer);
    }

    // Kumas (20)
    constexpr int kCloths = 20;
    ClothConfig clothCfg;
    std::vector<std::unique_ptr<ClothSystem>> cloths;
    cloths.reserve(kCloths);
    for (int i = 0; i < kCloths; ++i) {
        cloths.push_back(std::make_unique<ClothSystem>(clothCfg));
        const float x = static_cast<float>(i) * 4.0f - 40.0f;
        cloths.back()->Create(ctx.Physics(), Vec3{x, 10.0f, 100.0f}, cfg.movingLayer);
    }

    // Halat (30)
    constexpr int kRopes = 30;
    RopeConfig ropeCfg;
    std::vector<std::unique_ptr<RopeSystem>> ropes;
    ropes.reserve(kRopes);
    for (int i = 0; i < kRopes; ++i) {
        ropes.push_back(std::make_unique<RopeSystem>(ropeCfg));
        const float x = static_cast<float>(i) * 2.0f - 30.0f;
        ropes.back()->Create(ctx.Physics(), Vec3{x, 20.0f, 150.0f}, Vec3{x + 1.5f, 20.0f, 150.0f},
                             cfg.movingLayer);
    }

    // Ragdoll (50)
    RagdollSkeletonConfig skelCfg;
    RagdollBoneConfig pelvis;
    pelvis.name = "pelvis";
    pelvis.parentIndex = -1;
    pelvis.radius = 0.2f;
    pelvis.halfHeight = 0.25f;
    skelCfg.bones.push_back(pelvis);
    RagdollBoneConfig torso;
    torso.name = "torso";
    torso.parentIndex = 0;
    torso.radius = 0.2f;
    torso.halfHeight = 0.3f;
    torso.localPosition = Vec3{0, 0.55f, 0};
    skelCfg.bones.push_back(torso);
    RagdollSkeleton skeleton(skelCfg);
    constexpr int kRagdolls = 50;
    std::vector<RagdollInstance> ragdolls(kRagdolls);
    for (int i = 0; i < kRagdolls; ++i) {
        const float x = static_cast<float>(i % 10) * 3.0f - 15.0f;
        const float z = static_cast<float>(i / 10) * 3.0f + 200.0f;
        Transform pose;
        pose.position = Vec3{x, 5.0f, z};
        ragdolls[static_cast<size_t>(i)].Activate(ctx.Physics(), skeleton, pose, cfg.movingLayer);
    }

    // Yuzerlik (300)
    WaterVolume water;
    water.surfaceY = -50.0f;
    water.boundsMin = Vec3{-300, -60, 250};
    water.boundsMax = Vec3{300, 0, 350};
    ctx.Buoyancy().AddVolume(water);
    constexpr int kFloaters = 300;
    for (int i = 0; i < kFloaters; ++i) {
        const float x = static_cast<float>(i % 20) * 3.0f - 30.0f;
        const float z = static_cast<float>(i / 20) * 3.0f + 250.0f;
        JPH::BodyCreationSettings s(new JPH::BoxShape(JPH::Vec3(0.5f, 0.5f, 0.5f)),
                                    JPH::RVec3(x, -51.0, z), JPH::Quat::sIdentity(),
                                    JPH::EMotionType::Dynamic, cfg.movingLayer);
        JPH::BodyID id = bi.CreateAndAddBody(s, JPH::EActivation::Activate);
        ctx.Buoyancy().TrackBody(id);
    }

    // Bolge (500 izlenen govde, buz)
    FrictionZone ice;
    ice.boundsMin = Vec3{-300, -5, 300};
    ice.boundsMax = Vec3{300, 5, 400};
    ctx.Zones().AddZone(ice);
    constexpr int kZoneBodies = 500;
    for (int i = 0; i < kZoneBodies; ++i) {
        const float x = static_cast<float>(i % 25) * 2.0f - 25.0f;
        const float z = static_cast<float>(i / 25) * 2.0f + 320.0f;
        JPH::BodyCreationSettings s(new JPH::BoxShape(JPH::Vec3(0.4f, 0.4f, 0.4f)),
                                    JPH::RVec3(x, 1.0, z), JPH::Quat::sIdentity(),
                                    JPH::EMotionType::Dynamic, cfg.movingLayer);
        JPH::BodyID id = bi.CreateAndAddBody(s, JPH::EActivation::Activate);
        ctx.Zones().TrackBody(ctx.Physics(), id);
    }

    // Yikilabilir yapi (900 parca)
    const std::string saveDir =
        (std::filesystem::temp_directory_path() / "alazforge_bench_combined_destructible").string();
    std::filesystem::remove_all(saveDir);
    MaterialDatabase materials = MaterialDatabase::CreateDefault();
    const MaterialId concrete = materials.FindByName("concrete");
    DestructibleStructureSystem::Config destCfg;
    destCfg.savePath = saveDir;
    DestructibleStructureSystem destructible(destCfg);
    constexpr int kSide = 30;
    StructureConfig wall;
    wall.structureId = 1;
    wall.worldOrigin = Vec3{-500, 0, 0};
    wall.pieces.resize(static_cast<size_t>(kSide * kSide));
    for (int row = 0; row < kSide; ++row) {
        for (int col = 0; col < kSide; ++col) {
            PieceConfig& p = wall.pieces[static_cast<size_t>(row * kSide + col)];
            p.localCenter = Vec3{static_cast<float>(col), static_cast<float>(row), 0.0f};
            p.halfExtents = Vec3{0.45f, 0.45f, 0.1f};
            p.material = concrete;
        }
    }
    destructible.RegisterStructure(ctx.Physics(), cfg.nonMovingLayer, materials, wall);

    // LOD: uzakta 500 govde ekleyip izlemeye alalim (referans oyuncu (0,0,0))
    constexpr int kLodBodies = 500;
    for (int i = 0; i < kLodBodies; ++i) {
        const float x = 1000.0f + static_cast<float>(i % 25) * 2.0f;
        const float z = static_cast<float>(i / 25) * 2.0f;
        JPH::BodyCreationSettings s(new JPH::BoxShape(JPH::Vec3(0.3f, 0.3f, 0.3f)),
                                    JPH::RVec3(x, 5.0, z), JPH::Quat::sIdentity(),
                                    JPH::EMotionType::Dynamic, cfg.movingLayer);
        JPH::BodyID id = bi.CreateAndAddBody(s, JPH::EActivation::Activate);
        ctx.LOD().TrackBody(id);
    }

    ctx.Physics().OptimizeBroadPhase();

    const int totalBodies = 1 + kVehicles + kCloths + kRopes * ropeCfg.segmentCount +
                            kRagdolls * skeleton.BoneCount() + kFloaters + kZoneBodies +
                            kSide * kSide + kLodBodies;
    printf(
        "[HEPSI BIRLIKTE] toplam govde ~%d (arac=%d, karakter=%d, kumas=%d, halat=%d, "
        "ragdoll=%d, yuzen=%d, bolge=%d, yikim-parca=%d, lod-uzak=%d)\n",
        totalBodies, kVehicles, kCharacters, kCloths, kRopes, kRagdolls, kFloaters, kZoneBodies,
        kSide * kSide, kLodBodies);

    const Vec3 player{0, 0, 0};
    int explosionCounter = 0;
    Measure("HEPSI BIRLIKTE",
            "tam sahne: arac+karakter+kumas+halat+ragdoll+yuzerlik+bolge+"
            "yikim+LOD (tek Step)",
            totalBodies, 180, [&] {
                for (auto& v : vehicles)
                    v.SetDriverInput(1.0f, 0.0f, 0.0f, 0.0f);
                for (auto& c : characters) {
                    c->SetMoveInput(Vec3{0, 0, 1}, false);
                    c->Update(1.0f / 60.0f);
                }
                for (auto& c : cloths)
                    c->ApplyWind(Vec3{3, 0, 0}, 1.0f / 60.0f);

                // Her 30 adimda bir patlama + birkac mermi (surekli savas sahnesi)
                if (++explosionCounter % 30 == 0) {
                    std::vector<ExplosionHit> hits;
                    ExplosionConfig explCfg;
                    explCfg.radius = 10.0f;
                    ExplosionSystem::Detonate(ctx.Physics(), Vec3{0, 1, -100}, explCfg, hits);
                    BulletParams bullet;
                    for (int i = 0; i < 20; ++i)
                        ctx.Ballistics().Fire(bullet, Vec3{0, 50, -100}, Vec3{0, -1, 0}, 2.0f);
                }

                ctx.Step(1.0f / 60.0f, 1, &player);
            });

    destructible.Flush();
    std::filesystem::remove_all(saveDir);
}

} // namespace

int main() {
    printf("====================================================================\n");
    printf(" AlazForge sistem-bazli performans olcumu\n");
    printf(" (CI'nin jenerik CPU'sunda -- MUTLAK degil GORECELI karsilastirma)\n");
    printf("====================================================================\n\n");

    BenchRigidBody();
    BenchVehicle();
    BenchBallistics();
    BenchCharacter();
    BenchCloth();
    BenchRope();
    BenchRagdoll();
    BenchBuoyancy();
    BenchZones();
    BenchDestructible();
    BenchExplosion();
    BenchLOD();
    BenchCombinedAll();

    printf("\n==================== OZET ====================\n");
    printf("%-14s %-46s %8s %10s %10s\n", "sistem", "senaryo", "varlik", "ort(ms)", "maks(ms)");
    for (const BenchResult& r : g_results) {
        printf("%-14s %-46s %8d %10.4f %10.4f\n", r.system.c_str(), r.scenario.c_str(),
               r.entityCount, r.avgMs, r.maxMs);
    }
    return 0;
}
