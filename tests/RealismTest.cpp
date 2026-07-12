// R1 gerceklik katmani dogrulama testi:
//   1) WindSystem determinizmi + turbulansin gercekten degismesi
//   2) Balistikte yan ruzgar surüklenmesi (ruzgarsiz atisla karsilastirma)
//   3) Patlama siperi: duvar arkasindaki kutu impulse almaz
//   4) Malzeme surtunme hatti: ApplyToBody friction/restitution/userdata

#include "JoltTestWorld.h"
#include "ballistics/BallisticsSystem.h"
#include "material_db/MaterialDatabase.h"
#include "weapons/ExplosionSystem.h"
#include "wind/WindSystem.h"

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

static JPH::BodyID AddDynamicBox(TestWorld& world, JPH::RVec3 pos) {
    JPH::BodyCreationSettings settings(new JPH::BoxShape(JPH::Vec3(0.5f, 0.5f, 0.5f)), pos,
                                       JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic,
                                       Layers::MOVING);
    return world.physics.GetBodyInterface().CreateAndAddBody(settings, JPH::EActivation::Activate);
}

int main() {
    // ── 1) WindSystem ──────────────────────────────────────────────────
    printf("[WindSystem]\n");
    WindConfig wcfg;
    wcfg.direction = Vec3{1, 0, 0};
    wcfg.baseSpeed = 8.0f;
    wcfg.gustAmplitude = 3.0f;
    WindSystem windA(wcfg);
    WindSystem windB(wcfg);
    windA.Update(1.234f);
    windB.Update(1.234f);

    const Vec3 wA = windA.GetWindAt(Vec3{10, 0, 5});
    const Vec3 wB = windB.GetWindAt(Vec3{10, 0, 5});
    CHECK(wA.x == wB.x && wA.y == wB.y && wA.z == wB.z,
          "ayni zaman+pozisyon bit-birebir ayni ruzgar (determinizm)");
    CHECK(std::fabs(wA.x - wcfg.baseSpeed) < wcfg.gustAmplitude + 0.5f,
          "ruzgar siddeti taban hizin makul komsulugunda");

    windA.Update(0.7f);
    const Vec3 wLater = windA.GetWindAt(Vec3{10, 0, 5});
    CHECK(std::fabs(wLater.x - wA.x) > 1.0e-4f, "turbulans zamanla gercekten degisiyor");

    // ── 2) Balistikte ruzgar suruklenmesi ─────────────────────────────
    printf("[BallisticsWind]\n");
    {
        TestWorld world;
        // Uzak hedef duvari: mermi bir seye carpsin diye (z duzleminde genis)
        world.AddStaticBox(JPH::RVec3(300.0, 0.0, 0.0), JPH::Vec3(1.0f, 50.0f, 50.0f), 0);

        MaterialDatabase materials = MaterialDatabase::CreateDefault();
        BulletParams bullet; // 7.62 varsayilani

        BallisticsSystem::Config noWind;
        BallisticsSystem calm(world.physics, materials, noWind);
        const BulletSimResult calmShot = calm.Fire(bullet, Vec3{0, 0, 0}, Vec3{1, 0, 0}, 2.0f);

        BallisticsSystem::Config crossWind;
        crossWind.wind = Vec3{0, 0, 12.0f}; // 12 m/s tam yan ruzgar (+z)
        BallisticsSystem windy(world.physics, materials, crossWind);
        const BulletSimResult windyShot = windy.Fire(bullet, Vec3{0, 0, 0}, Vec3{1, 0, 0}, 2.0f);

        printf("  ruzgarsiz z=%.3f, ruzgarli z=%.3f\n", calmShot.finalPosition.z,
               windyShot.finalPosition.z);
        CHECK(std::fabs(calmShot.finalPosition.z) < 0.01f, "ruzgarsiz atis duz gitti");
        CHECK(windyShot.finalPosition.z > 0.05f, "yan ruzgar mermiyi ruzgar yonune surukledi (+z)");
    }

    // ── 3) Patlama siperi ──────────────────────────────────────────────
    printf("[ExplosionOcclusion]\n");
    {
        TestWorld world;
        world.AddGround();

        // Acik hedef ve duvar arkasi hedef (ikisi de ayni mesafede)
        const JPH::BodyID openBox = AddDynamicBox(world, JPH::RVec3(0.0, 0.5, 5.0));
        const JPH::BodyID hiddenBox = AddDynamicBox(world, JPH::RVec3(5.0, 0.5, 0.0));
        // Siper duvari: merkez ile hiddenBox arasinda
        world.AddStaticBox(JPH::RVec3(2.5, 1.5, 0.0), JPH::Vec3(0.2f, 1.5f, 2.0f), 0);
        world.physics.OptimizeBroadPhase();

        ExplosionConfig ecfg;
        ecfg.radius = 12.0f;
        ecfg.baseImpulseNs = 2000.0f;

        std::vector<ExplosionHit> hits;
        ExplosionSystem::Detonate(world.physics, Vec3{0, 0.5f, 0}, ecfg, hits);

        bool openHit = false, hiddenOccluded = false;
        float openImpulse = 0.0f, hiddenImpulse = 0.0f;
        for (const ExplosionHit& h : hits) {
            const float mag = std::sqrt(h.impulse.x * h.impulse.x + h.impulse.y * h.impulse.y +
                                        h.impulse.z * h.impulse.z);
            if (h.body == openBox) {
                openHit = !h.occluded;
                openImpulse = mag;
            }
            if (h.body == hiddenBox) {
                hiddenOccluded = h.occluded;
                hiddenImpulse = mag;
            }
        }
        CHECK(openHit && openImpulse > 1.0f, "acik hedef impulse aldi");
        CHECK(hiddenOccluded && hiddenImpulse < 1.0e-3f,
              "duvar arkasindaki hedef siperde: impulse yok");
    }

    // ── 4) Malzeme surtunme hatti ──────────────────────────────────────
    printf("[MaterialFriction]\n");
    {
        TestWorld world;
        const JPH::BodyID box = AddDynamicBox(world, JPH::RVec3(0.0, 0.5, 0.0));

        MaterialDatabase materials = MaterialDatabase::CreateDefault();
        const MaterialId ice = materials.FindByName("ice");
        const MaterialId rubber = materials.FindByName("rubber");
        CHECK(ice != kInvalidMaterial && rubber != kInvalidMaterial,
              "ice ve rubber varsayilan sette kayitli");

        JPH::BodyInterface& bi = world.physics.GetBodyInterface();
        materials.ApplyToBody(bi, box, ice);
        CHECK(bi.GetFriction(box) < 0.1f, "buz: dusuk surtunme govdeye uygulandi");
        CHECK(bi.GetUserData(box) == ice, "MaterialId user data'ya yazildi");

        materials.ApplyToBody(bi, box, rubber);
        CHECK(bi.GetFriction(box) > 0.9f && bi.GetRestitution(box) > 0.5f,
              "kaucuk: yuksek surtunme + sekme uygulandi");
    }

    if (g_failCount == 0) {
        printf("TEST BASARILI: ruzgar/balistik/siper/malzeme gercekciligi dogru.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
