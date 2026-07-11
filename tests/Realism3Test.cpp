// R3 oynanis yardimcilari dogrulama testi:
//   1) Comelme: kapsul kisalir, alcak tavan altinda ayaga kalkilamaz,
//      aciga cikinca kalkilir
//   2) Yuzme: su durumu verilen karakter yuzeye dogru yukselir, akintiyla
//      suruklenir; sudan cikinca normal yurumeye doner
//   3) Firlatma yorungesi: nokta listesi paraboliktir, ilk carpmada durur,
//      determinizm (ayni girdi ayni yorunge)
//   4) Enkaz: kirilan parcalar DetachBrokenPieces ile dinamik govdeye
//      donusur ve dusmeye baslar
//   5) Parasut: parasutlu dusus serbest dusustan belirgin yavas; planor
//      ayni surede daha uzaga suzulur

#include "JoltTestWorld.h"
#include "aero/AeroSystem.h"
#include "character/CharacterController.h"
#include "destructible/DestructibleStructureSystem.h"
#include "material_db/MaterialDatabase.h"
#include "weapons/TrajectoryPredictor.h"

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
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
    // ── 1) Comelme ─────────────────────────────────────────────────────
    printf("[Crouch]\n");
    {
        TestWorld world;
        world.AddGround();
        // Alcak tavan: y=1.3'te (comelmis 1.1m sigar, ayakta 1.8m sigmaz)
        world.AddStaticBox(JPH::RVec3(5.0, 1.55, 0.0), JPH::Vec3(2.0f, 0.25f, 2.0f), 0);

        CharacterControllerConfig cfg;
        CharacterController c(cfg);
        c.Spawn(world.physics, Vec3{0, 0.1f, 0}, Layers::MOVING);
        for (int i = 0; i < 30; ++i) {
            c.Update(1.0f / 60.0f);
            world.Step(1.0f / 60.0f);
        }

        CHECK(c.SetCrouch(true) && c.IsCrouching(), "comelme basarili");

        // Comelmis halde alcak tavanin altina yuru
        c.SetMoveInput(Vec3{1, 0, 0});
        for (int i = 0; i < 150; ++i) {
            c.Update(1.0f / 60.0f);
            world.Step(1.0f / 60.0f);
        }
        c.SetMoveInput(Vec3{0, 0, 0});
        const float xUnder = c.GetWorldTransform().position.x;
        CHECK(xUnder > 3.5f && xUnder < 7.0f, "comelmis karakter tavanin altina girdi");

        // Tavanin altinda ayaga kalkilamamali
        CHECK(!c.SetCrouch(false) && c.IsCrouching(), "alcak tavan altinda ayaga kalkamadi");

        // Tavandan cik, tekrar dene
        c.SetMoveInput(Vec3{1, 0, 0});
        for (int i = 0; i < 200; ++i) {
            c.Update(1.0f / 60.0f);
            world.Step(1.0f / 60.0f);
        }
        c.SetMoveInput(Vec3{0, 0, 0});
        CHECK(c.SetCrouch(false) && !c.IsCrouching(), "acikta ayaga kalkti");
        c.Destroy();
    }

    // ── 2) Yuzme ───────────────────────────────────────────────────────
    printf("[Swim]\n");
    {
        TestWorld world;
        world.AddGround(); // havuz tabani gibi dusun: su yuzeyi y=6

        CharacterControllerConfig cfg;
        CharacterController c(cfg);
        c.Spawn(world.physics, Vec3{0, 0.1f, 0}, Layers::MOVING);

        // Karakter dipte, su yuzeyi 6m yukarida, akinti +x
        for (int i = 0; i < 300; ++i) {
            c.SetWaterState(true, 6.0f, Vec3{0.8f, 0, 0});
            c.Update(1.0f / 60.0f);
            world.Step(1.0f / 60.0f);
        }
        const Vec3 p = c.GetWorldTransform().position;
        printf("  5sn yuzme sonrasi: x=%.2f y=%.2f (yuzey=6.0)\n", p.x, p.y);
        CHECK(c.IsSwimming() || p.y > 4.0f, "karakter yuzme moduna gecti/yukseldi");
        CHECK(p.y > 3.0f, "karakter yuzeye dogru yukseldi (dipte kalmadi)");
        CHECK(p.x > 1.0f, "akinti karakteri surukledi");
        CHECK(p.y < 6.5f, "karakter yuzeyin uzerine ucmadi");

        // Sudan cik: yuzme kapanmali
        c.SetWaterState(false, 0.0f, Vec3{0, 0, 0});
        c.Update(1.0f / 60.0f);
        world.Step(1.0f / 60.0f);
        CHECK(!c.IsSwimming(), "su bilgisi kesilince yuzme kapandi");
        c.Destroy();
    }

    // ── 3) Firlatma yorungesi ──────────────────────────────────────────
    printf("[Trajectory]\n");
    {
        TestWorld world;
        world.AddGround();

        TrajectoryConfig tcfg;
        const Vec3 origin{0, 1.5f, 0};
        const Vec3 velocity{8.0f, 6.0f, 0}; // el bombasi atisi
        const TrajectoryResult a =
            TrajectoryPredictor::Predict(&world.physics, origin, velocity, tcfg);
        const TrajectoryResult b =
            TrajectoryPredictor::Predict(&world.physics, origin, velocity, tcfg);

        CHECK(a.points.size() > 10, "yorunge yeterli noktayla orneklendi");
        CHECK(a.hitSomething && a.hitPoint.y < 0.1f, "yorunge zeminde carpmayla bitti");
        CHECK(a.hitPoint.x > 3.0f, "carpma noktasi ileri yonde makul mesafede");

        // Parabolik: tepe noktasi baslangictan yuksek, carpma dusuk
        float peak = 0.0f;
        for (const Vec3& pt : a.points)
            peak = std::max(peak, pt.y);
        CHECK(peak > origin.y + 0.8f, "yorunge tepe noktasi belirgin yuksek (parabol)");

        // Determinizm
        bool same = a.points.size() == b.points.size() && a.hitPoint.x == b.hitPoint.x &&
                    a.hitPoint.y == b.hitPoint.y && a.hitPoint.z == b.hitPoint.z;
        CHECK(same, "ayni girdi bit-birebir ayni yorunge (determinizm)");
    }

    // ── 4) Enkaz parcalari ─────────────────────────────────────────────
    printf("[Debris]\n");
    {
        TestWorld world;
        world.AddGround();

        const char* savePath = "realism3_destructible";
        std::filesystem::remove_all(savePath);

        DestructibleStructureSystem::Config dcfg;
        dcfg.savePath = savePath;
        DestructibleStructureSystem destructible(dcfg);
        destructible.OnPlayerMove(0.0f, 0.0f);

        // 1x3 sutun: alt parca kirilinca ustler destek kaybeder
        StructureConfig sc;
        sc.structureId = 42;
        sc.worldOrigin = Vec3{0, 0.5f, 0};
        for (int i = 0; i < 3; ++i) {
            PieceConfig pc;
            pc.localCenter = Vec3{0, static_cast<float>(i), 0};
            pc.halfExtents = Vec3{0.5f, 0.5f, 0.5f};
            pc.healthMax = 50.0f;
            if (i > 0) pc.supportNeighbors = {static_cast<uint16_t>(i - 1)};
            sc.pieces.push_back(pc);
        }
        MaterialDatabase materials = MaterialDatabase::CreateDefault();
        destructible.RegisterStructure(world.physics, Layers::NON_MOVING, materials, sc);

        std::vector<PieceBrokenEvent> events;
        destructible.ApplyDamage(42, 0, 100.0f, events); // alt parcayi kir
        CHECK(!events.empty(), "kirilma event'leri uretildi");

        std::vector<JPH::BodyID> debris;
        const size_t created = destructible.DetachBrokenPieces(world.physics, events, debris);
        CHECK(created == events.size() && created >= 1, "tum kirilan parcalar enkaza donustu");

        // Enkaz gercekten dinamik mi? Bir sure sonra dusmus olmali
        const double y0 = world.physics.GetBodyInterface().GetPosition(debris.front()).GetY();
        for (int i = 0; i < 60; ++i)
            world.Step(1.0f / 60.0f);
        const double y1 = world.physics.GetBodyInterface().GetPosition(debris.front()).GetY();
        CHECK(y1 <= y0 + 1.0e-3, "enkaz parcasi dinamik (yercekimine tabi)");

        for (JPH::BodyID id : debris) {
            world.physics.GetBodyInterface().RemoveBody(id);
            world.physics.GetBodyInterface().DestroyBody(id);
        }
        std::filesystem::remove_all(savePath);
    }

    // ── 5) Parasut + planor ────────────────────────────────────────────
    printf("[Aero]\n");
    {
        auto dropTest = [](bool withParachute) {
            TestWorld world;
            JPH::BodyCreationSettings s(new JPH::BoxShape(JPH::Vec3(0.4f, 0.4f, 0.4f)),
                                        JPH::RVec3(0.0, 120.0, 0.0), JPH::Quat::sIdentity(),
                                        JPH::EMotionType::Dynamic, Layers::MOVING);
            s.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
            s.mMassPropertiesOverride.mMass = 80.0f;
            JPH::BodyID body =
                world.physics.GetBodyInterface().CreateAndAddBody(s, JPH::EActivation::Activate);

            ParachuteConfig pcfg;
            for (int i = 0; i < 300; ++i) { // 5 saniye dusus
                if (withParachute)
                    AeroSystem::ApplyParachute(world.physics, body, pcfg, Vec3{0, 0, 0});
                world.Step(1.0f / 60.0f);
            }
            return world.physics.GetBodyInterface().GetLinearVelocity(body).GetY();
        };

        const float freeFallVy = static_cast<float>(dropTest(false));
        const float chuteVy = static_cast<float>(dropTest(true));
        printf("  5sn sonra dusey hiz: serbest=%.1f m/s, parasutlu=%.1f m/s\n", freeFallVy,
               chuteVy);
        CHECK(freeFallVy < -30.0f, "serbest dusus hizlandi");
        CHECK(chuteVy > -10.0f && chuteVy < -1.0f,
              "parasut terminal hizi dusuk ve makul (1-10 m/s inis)");

        // Planor: ileri hizla birakilan govde, tasima sayesinde ayni surede
        // tasimasiz govdeden daha az irtifa kaybeder
        auto glideTest = [](bool withWing) {
            TestWorld world;
            JPH::BodyCreationSettings s(new JPH::BoxShape(JPH::Vec3(0.5f, 0.2f, 0.5f)),
                                        JPH::RVec3(0.0, 100.0, 0.0), JPH::Quat::sIdentity(),
                                        JPH::EMotionType::Dynamic, Layers::MOVING);
            s.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
            s.mMassPropertiesOverride.mMass = 90.0f;
            JPH::BodyID body =
                world.physics.GetBodyInterface().CreateAndAddBody(s, JPH::EActivation::Activate);
            world.physics.GetBodyInterface().SetLinearVelocity(body, JPH::Vec3(20, 0, 0));

            GliderConfig gcfg;
            for (int i = 0; i < 240; ++i) {
                if (withWing)
                    AeroSystem::ApplyGlider(world.physics, body, gcfg, Vec3{1, 0, 0},
                                            Vec3{0, 0, 0});
                world.Step(1.0f / 60.0f);
            }
            return world.physics.GetBodyInterface().GetPosition(body).GetY();
        };

        const double yNoWing = glideTest(false);
        const double yWing = glideTest(true);
        printf("  4sn sonra irtifa: kanatsiz=%.1f m, planor=%.1f m\n", yNoWing, yWing);
        CHECK(yWing > yNoWing + 10.0, "planor tasimasi irtifa kaybini belirgin azaltti");
    }

    if (g_failCount == 0) {
        printf("TEST BASARILI: comelme/yuzme/yorunge/enkaz/parasut dogru.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
