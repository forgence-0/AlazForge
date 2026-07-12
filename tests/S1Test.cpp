// S1 dogrulama testi:
//   1) CollisionEventSystem: yeterince sert temas olay uretir, hafif temas
//      (esik alti) uretmez, malzeme id'leri dogru tasinir
//   2) Arac hasar modeli: sert carpma motor gucunu dusurur, cok sert carpma
//      araci surulemez hale getirir; hafif carpma hasar vermez

#include "JoltTestWorld.h"
#include "audio/CollisionEventSystem.h"
#include "material_db/MaterialDatabase.h"
#include "vehicle/VehicleSystem.h"

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>

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
    // ── 1) CollisionEventSystem ─────────────────────────────────────────
    printf("[CollisionEvents]\n");
    {
        TestWorld world;
        CollisionEventSystem events(2.0f); // esik: 2 m/s
        events.Attach(world.physics);

        MaterialDatabase materials = MaterialDatabase::CreateDefault();
        const MaterialId steel = materials.FindByName("steel");

        world.AddGround();

        // Sert dusus: zeminle carpisma esigin cok ustunde olmali
        JPH::BodyCreationSettings s(new JPH::BoxShape(JPH::Vec3(0.3f, 0.3f, 0.3f)),
                                    JPH::RVec3(0.0, 10.0, 0.0), JPH::Quat::sIdentity(),
                                    JPH::EMotionType::Dynamic, Layers::MOVING);
        JPH::BodyID fallingBox =
            world.physics.GetBodyInterface().CreateAndAddBody(s, JPH::EActivation::Activate);
        materials.ApplyToBody(world.physics.GetBodyInterface(), fallingBox, steel);

        for (int i = 0; i < 120; ++i)
            world.Step(1.0f / 60.0f);

        std::vector<CollisionEvent> collected;
        events.DrainEvents(collected);
        bool found = false;
        for (const CollisionEvent& e : collected) {
            if (e.bodyA == fallingBox || e.bodyB == fallingBox) {
                found = true;
                CHECK(e.impactSpeed > 2.0f, "carpma hizi esigin ustunde kaydedildi");
                CHECK(e.materialA == steel || e.materialB == steel,
                      "carpan govdenin malzemesi olayda tasindi");
                break;
            }
        }
        CHECK(found, "sert dusus carpisma olayi uretti");

        // Ikinci drenaj: kuyruk bosalmis olmali
        std::vector<CollisionEvent> secondDrain;
        events.DrainEvents(secondDrain);
        for (int i = 0; i < 5; ++i)
            world.Step(1.0f / 60.0f); // artik durgun, yeni olay yok
        std::vector<CollisionEvent> afterRest;
        events.DrainEvents(afterRest);
        CHECK(afterRest.empty(), "govde yerlesince yeni carpisma olayi uretmiyor");
    }

    // ── 2) Arac hasar modeli ─────────────────────────────────────────────
    printf("[VehicleDamage]\n");
    {
        TestWorld world;
        world.AddGround(400.0f);

        VehicleChassisConfig chassis;
        std::vector<AxleConfig> axles(2);
        axles[0].positionZ = 1.6f;
        axles[0].driven = true;
        axles[1].positionZ = -1.6f;
        axles[1].driven = true;

        VehicleSystem vehicle;
        EngineConfig engineCfg;
        vehicle.CreateWheeled(world.physics, chassis, axles, engineCfg, TransmissionConfig{},
                              Layers::MOVING);

        CHECK(vehicle.GetHealth() == 1.0f, "baslangicta arac tam saglikli");
        CHECK(vehicle.IsDrivable(), "baslangicta surulebilir");

        vehicle.ApplyImpactDamage(2.0f); // esik (4 m/s) altinda
        CHECK(vehicle.GetHealth() == 1.0f, "hafif carpma hasar vermedi");

        vehicle.ApplyImpactDamage(20.0f); // esigin 16 m/s ustunde -> %32 hasar
        printf("  20 m/s carpma sonrasi saglik: %.2f\n", vehicle.GetHealth());
        CHECK(vehicle.GetHealth() < 0.75f && vehicle.GetHealth() > 0.6f,
              "sert carpma olculebilir hasar verdi");
        CHECK(vehicle.IsDrivable(), "kismi hasarda hala surulebilir");

        vehicle.ApplyImpactDamage(60.0f); // agir darbe
        printf("  ikinci agir carpma sonrasi saglik: %.2f\n", vehicle.GetHealth());
        CHECK(!vehicle.IsDrivable(), "biriken agir hasar araci surulemez yapti");
    }

    if (g_failCount == 0) {
        printf("TEST BASARILI: carpisma olaylari + arac hasar modeli dogru.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
