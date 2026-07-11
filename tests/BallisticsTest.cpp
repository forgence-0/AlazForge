// Faz 4 doğrulama testi: material_db + balistik sistem.
// Mermi düşüşü, hava direnci, penetrasyon, saplanma ve sekme senaryoları.

#include "JoltTestWorld.h"
#include "ballistics/BallisticsSystem.h"

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

int main() {
    // ── MaterialDatabase ────────────────────────────────────────────
    printf("[MaterialDatabase]\n");
    MaterialDatabase db = MaterialDatabase::CreateDefault();
    const MaterialId steel = db.FindByName("steel");
    const MaterialId wood = db.FindByName("wood");

    CHECK(db.Count() == 8, "varsayilan 8 malzeme kayitli (ice/rubber dahil)");
    CHECK(steel != kInvalidMaterial && wood != kInvalidMaterial, "steel/wood bulundu");
    CHECK(db.FindByName("adamantium") == kInvalidMaterial, "olmayan malzeme kInvalidMaterial");
    CHECK(db.Get(steel).rhaEquivalentPerCm > db.Get(wood).rhaEquivalentPerCm,
          "celik penetrasyon direnci ahsaptan yuksek");

    const MaterialId custom =
        db.Register({"brick", SurfaceType::Custom, 1800.0f, 30.0f, 0.9f, 0.0f, 0.7f});
    CHECK(db.FindByName("brick") == custom && db.Count() == 9,
          "oyun tarafi yeni malzeme kaydedebiliyor");

    TestWorld world;
    BallisticsSystem ballistics(world.physics, db);
    BulletParams rifle; // 7.62 varsayilanlari

    // ── Mermi düşüşü + hava direnci (boş dünya) ─────────────────────
    printf("[Mermi dususu]\n");
    {
        BulletParams slow = rifle;
        slow.muzzleVelocity = 100.0f;
        BulletSimResult r = ballistics.Fire(slow, Vec3{0, 0, 0}, Vec3{1, 0, 0}, 1.0f);

        CHECK(r.impacts.empty(), "bos dunyada temas yok");
        // 1 saniyede serbest dusme ~4.9m (surtunme dikeyde payi kucuk)
        CHECK(r.finalPosition.y < -4.0f && r.finalPosition.y > -5.5f,
              "1s ucusta ~4.9m mermi dususu");
        const float finalSpeed = std::sqrt(r.finalVelocity.x * r.finalVelocity.x +
                                           r.finalVelocity.y * r.finalVelocity.y +
                                           r.finalVelocity.z * r.finalVelocity.z);
        CHECK(finalSpeed < 100.0f, "hava direnci hizi dusurdu (menzil kaybi)");
        CHECK(r.finalPosition.x > 90.0f && r.finalPosition.x < 100.0f,
              "yatay menzil beklenen aralikta");
    }

    // ── Penetrasyon: 10cm ahşap duvar ───────────────────────────────
    printf("[Penetrasyon - ahsap]\n");
    {
        JPH::BodyID wall =
            world.AddStaticBox(JPH::RVec3(20.0, 0.0, 0.0), JPH::Vec3(0.05f, 2.0f, 2.0f), wood);

        BulletSimResult r = ballistics.Fire(rifle, Vec3{0, 0, 0}, Vec3{1, 0, 0}, 0.5f);

        CHECK(r.impacts.size() >= 1, "ahsap duvara temas kaydedildi");
        if (!r.impacts.empty()) {
            const ImpactEvent& ev = r.impacts.front();
            CHECK(ev.type == ImpactType::Penetration, "10cm ahsabi 7.62 deldi");
            CHECK(ev.material == wood, "temas malzemesi wood");
            CHECK(ev.remainingSpeed < ev.impactSpeed && ev.remainingSpeed > 0.0f,
                  "cikis hizi giris hizindan dusuk ama > 0");
        }
        CHECK(r.finalPosition.x > 20.1f, "mermi duvarin arkasina gecti");

        world.physics.GetBodyInterface().RemoveBody(wall);
        world.physics.GetBodyInterface().DestroyBody(wall);
    }

    // ── Saplanma: 5cm çelik duvar ───────────────────────────────────
    printf("[Saplanma - celik]\n");
    {
        JPH::BodyID wall =
            world.AddStaticBox(JPH::RVec3(20.0, 0.0, 0.0), JPH::Vec3(0.025f, 2.0f, 2.0f), steel);

        BulletSimResult r = ballistics.Fire(rifle, Vec3{0, 0, 0}, Vec3{1, 0, 0}, 0.5f);

        CHECK(r.impacts.size() == 1, "celik duvara tek temas");
        if (!r.impacts.empty()) {
            CHECK(r.impacts.front().type == ImpactType::Embed, "5cm celige saplandi");
            CHECK(r.impacts.front().remainingSpeed == 0.0f, "saplanmada hiz 0");
        }
        CHECK(r.stopped, "mermi durdu");
        CHECK(r.finalPosition.x < 20.1f, "mermi duvari gecemedi");

        world.physics.GetBodyInterface().RemoveBody(wall);
        world.physics.GetBodyInterface().DestroyBody(wall);
    }

    // ── Sekme: çelik plakaya sığ açı ────────────────────────────────
    printf("[Sekme - celik]\n");
    {
        // Buyuk celik plaka, ust yuzeyi y=0
        JPH::BodyID plate =
            world.AddStaticBox(JPH::RVec3(30.0, -0.5, 0.0), JPH::Vec3(30.0f, 0.5f, 5.0f), steel);

        // ~8 derece siyirma acisiyla atis (celik kritik acisi 17)
        const float angleRad = 8.0f * 0.0174533f;
        Vec3 dir{std::cos(angleRad), -std::sin(angleRad), 0.0f};
        BulletSimResult r = ballistics.Fire(rifle, Vec3{0, 1, 0}, dir, 0.5f);

        CHECK(!r.impacts.empty(), "plakaya temas kaydedildi");
        if (!r.impacts.empty()) {
            const ImpactEvent& ev = r.impacts.front();
            CHECK(ev.type == ImpactType::Ricochet, "sig acida mermi sekti");
            CHECK(ev.grazingAngleDeg < 17.0f, "siyirma acisi kritik acinin altinda");
            CHECK(ev.remainingSpeed < ev.impactSpeed, "sekme hiz kaybettirdi");
        }
        CHECK(r.finalVelocity.y > 0.0f || r.finalPosition.y > 0.0f, "mermi sekip yukari yon aldi");

        // Ayni plakaya dik atis sekmemeli
        BulletSimResult r2 = ballistics.Fire(rifle, Vec3{30, 5, 0}, Vec3{0, -1, 0}, 0.5f);
        CHECK(!r2.impacts.empty() && r2.impacts.front().type != ImpactType::Ricochet,
              "dik atis sekmedi");

        world.physics.GetBodyInterface().RemoveBody(plate);
        world.physics.GetBodyInterface().DestroyBody(plate);
    }

    // ── Sifir yon vektoru: Normalized() NaN'a gitmemeli, gecersiz atis
    // olarak hemen durmali ──────────────────────────────────────────────
    printf("[Sifir yon vektoru]\n");
    {
        BulletSimResult r = ballistics.Fire(rifle, Vec3{0, 0, 0}, Vec3{0, 0, 0}, 1.0f);
        CHECK(r.stopped, "sifir yon vektoru gecersiz atis olarak hemen duruyor");
        CHECK(r.finalVelocity.x == 0.0f && r.finalVelocity.y == 0.0f && r.finalVelocity.z == 0.0f,
              "sifir yon vektorunde NaN hiz uretilmiyor");
    }

    if (g_failCount == 0) {
        printf("TEST BASARILI: balistik sistem ve malzeme veritabani dogru calisiyor.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
