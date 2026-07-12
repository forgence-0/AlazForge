// ExplosionSystem dogrulama testi: yakin govde uzaktakinden daha buyuk
// impulse alir, yaricap disindaki hic etkilenmez, statik govdeler listede
// gorunur ama impulse almaz.

#include "JoltTestWorld.h"
#include "weapons/ExplosionSystem.h"

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
    TestWorld world;
    world.AddGround();

    JPH::BodyID nearBox = AddDynamicBox(world, JPH::RVec3(2.0, 0.5, 0.0));
    // farBox farkli dogrultuda: nearBox ile ayni hizada olursa yeni siper
    // (losOcclusion) kontrolu onu hakli olarak "siperde" sayar
    JPH::BodyID farBox = AddDynamicBox(world, JPH::RVec3(0.0, 0.5, 7.0));
    JPH::BodyID outsideBox = AddDynamicBox(world, JPH::RVec3(25.0, 0.5, 0.0));

    printf("[ExplosionSystem]\n");

    ExplosionConfig cfg;
    cfg.radius = 10.0f;
    cfg.baseImpulseNs = 2000.0f;

    std::vector<ExplosionHit> hits;
    ExplosionSystem::Detonate(world.physics, Vec3{0, 0.5f, 0}, cfg, hits);

    float nearImpulse = -1.0f, farImpulse = -1.0f;
    bool outsideHit = false, groundListed = false, groundImpulseZero = true;
    for (const ExplosionHit& h : hits) {
        const float mag = std::sqrt(h.impulse.x * h.impulse.x + h.impulse.y * h.impulse.y +
                                    h.impulse.z * h.impulse.z);
        if (h.body == nearBox) nearImpulse = mag;
        if (h.body == farBox) farImpulse = mag;
        if (h.body == outsideBox) outsideHit = true;
        if (h.body != nearBox && h.body != farBox && h.body != outsideBox) {
            groundListed = true; // zemin statik govdesi
            if (mag > 1.0e-3f) groundImpulseZero = false;
        }
    }

    CHECK(nearImpulse > 0.0f && farImpulse > 0.0f, "yaricap icindeki iki kutu da etkilendi");
    CHECK(nearImpulse > farImpulse, "yakin kutu uzaktakinden buyuk impulse aldi");
    CHECK(!outsideHit, "yaricap disindaki kutu listede yok");
    CHECK(groundListed, "statik zemin listede (hasar hesabi icin)");
    CHECK(groundImpulseZero, "statik zemine impulse uygulanmadi");

    // Impulse'lar gercekten hiz uretmis mi? Bir adim ilerletip bak
    world.Step(1.0f / 60.0f);
    const JPH::Vec3 nearVel = world.physics.GetBodyInterface().GetLinearVelocity(nearBox);
    CHECK(nearVel.GetX() > 1.0f, "yakin kutu patlamadan disari savruldu (+x hizi)");

    if (g_failCount == 0) {
        printf("TEST BASARILI: ExplosionSystem falloff/yaricap/statik davranisi dogru.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
