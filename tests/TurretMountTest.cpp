// Faz B.3 doğrulama testi: TurretMount.
// Sabit (statik) bir "şasi" gövdesine bağlanan türetin hedef yaw/pitch
// açılarına motorlu constraint ile yaklaştığı, pitch limitinin uygulandığı
// ve namlu ucunun pivot'tan sabit uzaklıkta kaldığı doğrulanır.

#include "JoltTestWorld.h"
#include "weapons/TurretMount.h"

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

static float DegToRad(float deg) { return deg * 3.14159265f / 180.0f; }
static float RadToDeg(float rad) { return rad * 180.0f / 3.14159265f; }

int main() {
    TestWorld world;

    // Sabit "şasi": türetin oturduğu statik gövde
    JPH::BodyID chassis =
        world.AddStaticBox(JPH::RVec3(0.0, 0.0, 0.0), JPH::Vec3(1.0f, 1.0f, 1.0f), 0);

    TurretMountConfig cfg;
    cfg.localOffset = Vec3{0, 1.0f, 0};
    cfg.localMuzzleOffset = Vec3{0, 0, 1.0f};
    cfg.yawRateDegPerSec = 90.0f;
    cfg.pitchRateDegPerSec = 45.0f;
    cfg.minPitchDeg = -10.0f;
    cfg.maxPitchDeg = 20.0f;

    TurretMount turret(cfg);
    turret.AttachToBody(world.physics, chassis, Layers::MOVING);

    printf("[TurretMount]\n");

    const float targetYaw = DegToRad(90.0f);
    const float targetPitch = DegToRad(30.0f); // limitin (20) uzerinde -> kenetlenmeli

    float maxYawSeen = 0.0f;
    for (int i = 0; i < 400; ++i) {
        turret.SetTargetAngles(targetYaw, targetPitch);
        world.Step(1.0f / 60.0f);
        maxYawSeen = std::max(maxYawSeen, turret.CurrentYaw());
    }

    const float finalYawDeg = RadToDeg(turret.CurrentYaw());
    const float finalPitchDeg = RadToDeg(turret.CurrentPitch());

    printf("  yaw = %.2f derece, pitch = %.2f derece\n", finalYawDeg, finalPitchDeg);

    CHECK(finalYawDeg > 80.0f && finalYawDeg < 100.0f, "yaw hedef 90 dereceye yaklasti");
    CHECK(finalPitchDeg < 25.0f, "pitch, limitin (20) belirgin uzerine cikmadi");
    CHECK(finalPitchDeg > 10.0f, "pitch limite dogru ilerledi (0'da kalmadi)");
    CHECK(RadToDeg(maxYawSeen) < 100.0f, "yaw hedefi asiri asmadi (buyuk overshoot yok)");

    // Namlu ucu, oryantasyondan bagimsiz olarak pivot'tan sabit uzaklikta olmali
    // (yaw+pitch ayni pivot noktasindan gecen iki eksenli bir donel zincir --
    // rijit govde donusu pivot'tan uzakligi korur). Sabit mesafe: pitch
    // govdesinin pivot'tan one kaydirma miktari (yaw+pitch kutu yari-boylarinin
    // toplami, TurretMount.cpp'deki kPitchForwardOffset = 0.9) artı
    // localMuzzleOffset (1.0) = 1.9.
    const Transform muzzle = turret.GetMuzzleWorldTransform();
    const float dx = muzzle.position.x - 0.0f;
    const float dy = muzzle.position.y - 1.0f;
    const float dz = muzzle.position.z - 0.0f;
    const float distFromPivot = std::sqrt(dx * dx + dy * dy + dz * dz);
    CHECK(std::fabs(distFromPivot - 1.9f) < 0.2f, "namlu ucu pivottan sabit uzaklikta");

    turret.Destroy();
    world.physics.GetBodyInterface().RemoveBody(chassis);
    world.physics.GetBodyInterface().DestroyBody(chassis);

    if (g_failCount == 0) {
        printf("TEST BASARILI: TurretMount motorlu constraint ile hedefe yaklasiyor.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
