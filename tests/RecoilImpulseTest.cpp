// Faz B.1 doğrulama testi: RecoilImpulse.
// Serbest (yerçekimsiz olmayan ama havada, zeminden uzak) dinamik bir
// "taşıyıcı" gövdeye bilinen kütle/hızla geri tepme uygulanır; sonucun
// momentum korunumuna uyduğu doğrulanır.

#include "JoltTestWorld.h"
#include "weapons/RecoilImpulse.h"

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

int main() {
    TestWorld world;

    // 500kg taşıyıcı gövde, havada serbest (zeminden uzak, çarpışma yok)
    const float carrierMass = 500.0f;
    JPH::BodyCreationSettings carrierSettings(new JPH::BoxShape(JPH::Vec3(0.5f, 0.5f, 0.5f)),
                                              JPH::RVec3(0.0, 100.0, 0.0), JPH::Quat::sIdentity(),
                                              JPH::EMotionType::Dynamic, Layers::MOVING);
    carrierSettings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
    carrierSettings.mMassPropertiesOverride.mMass = carrierMass;
    JPH::BodyID carrier = world.physics.GetBodyInterface().CreateAndAddBody(
        carrierSettings, JPH::EActivation::Activate);
    world.physics.OptimizeBroadPhase();

    printf("[RecoilImpulse]\n");

    RecoilParams params;
    params.bulletMassKg = 0.0095f;
    params.muzzleVelocity = 830.0f;
    params.recoilMultiplier = 1.0f;

    const Vec3 fireDirection{0, 0, 1}; // +z'ye ateş
    const Vec3 firePoint{0, 100, 0};   // gövdenin merkezinden ateşleniyor (tork yaratmasın)

    RecoilImpulse::ApplyToBody(world.physics, carrier, params, fireDirection, firePoint);

    const Vec3 vel = FromJolt(world.physics.GetBodyInterface().GetLinearVelocity(carrier));
    const float expectedSpeed =
        params.bulletMassKg * params.muzzleVelocity * params.recoilMultiplier / carrierMass;

    CHECK(vel.z < 0.0f, "geri tepme, ates yonunun tersine hiz kazandirdi");
    CHECK(std::fabs(vel.x) < 1e-4f && std::fabs(vel.y) < 1e-4f,
          "geri tepme yalnizca ates ekseninde etki etti");
    CHECK(std::fabs(std::fabs(vel.z) - expectedSpeed) < expectedSpeed * 0.01f,
          "geri tepme hizi momentum korunumu ile eslesiyor");

    world.physics.GetBodyInterface().RemoveBody(carrier);
    world.physics.GetBodyInterface().DestroyBody(carrier);

    if (g_failCount == 0) {
        printf("TEST BASARILI: RecoilImpulse momentum korunumuna uygun calisiyor.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
