// Top fizigi dogrulama testi: BallPhysicsSystem.
// Ayni hizla firlatilan iki ozdes top -- biri spin'li, biri spin'siz --
// karsilastirilir: spin'li top Magnus etkisiyle yanal olarak belirgin
// sapmali, spin'siz top surukleme (drag) nedeniyle yavaslamali. Ayrica
// uzatilmis (elips, Amerikan futbolu topu gibi) bir topun cokmeden
// olusturulup simule edilebildigi sanity-check edilir.

#include "JoltTestWorld.h"
#include "sportsball/BallPhysicsSystem.h"

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
    world.AddStaticBox(JPH::RVec3(0.0, -50.0, 0.0), JPH::Vec3(200.0f, 1.0f, 200.0f), 0);

    BallPhysicsSystem balls;

    BallConfig cfg;
    cfg.mass = 0.45f;
    cfg.radius = 0.11f;
    cfg.dragCoefficient = 0.47f;
    cfg.restitution = 0.6f;
    cfg.magnusCoefficient = 5.0f; // testte belirgin sapma gorulsun diye buyutulmus

    const Vec3 start{0, 10, 0};
    const Quat noRot;

    JPH::BodyID spinBall = balls.CreateBall(world.physics, cfg, start, noRot, Layers::MOVING);
    JPH::BodyID plainBall = balls.CreateBall(world.physics, cfg, start, noRot, Layers::MOVING);

    JPH::BodyInterface& bi = world.physics.GetBodyInterface();
    bi.SetLinearVelocity(spinBall, JPH::Vec3(15, 0, 0));
    bi.SetLinearVelocity(plainBall, JPH::Vec3(15, 0, 0));
    bi.SetAngularVelocity(spinBall, JPH::Vec3(0, 25, 0)); // dikey eksen spin -> yanal (Z) sapma

    world.physics.OptimizeBroadPhase();

    printf("[BallPhysicsSystem]\n");

    const float dt = 1.0f / 60.0f;
    for (int i = 0; i < 90; ++i) {
        balls.Update(world.physics, dt);
        world.Step(dt);
    }

    const float spinZ = FromJolt(JPH::Vec3(bi.GetPosition(spinBall))).z;
    const float plainZ = FromJolt(JPH::Vec3(bi.GetPosition(plainBall))).z;
    const float spinX = FromJolt(JPH::Vec3(bi.GetPosition(spinBall))).x;
    const float plainSpeed = bi.GetLinearVelocity(plainBall).Length();

    printf("  spin topu z=%.3f, spinsiz top z=%.3f, spinsiz topun kalan hizi=%.2f m/s\n", spinZ,
           plainZ, plainSpeed);

    CHECK(std::fabs(spinZ - plainZ) > 0.5f, "spin topu Magnus etkisiyle yanal olarak sapti");
    CHECK(plainSpeed < 15.0f, "surukleme (drag) topun hizini azaltti");
    CHECK(spinX > 5.0f, "top ileri hareketine devam etti");

    // Amerikan futbolu topu gibi uzatilmis (elips) sekil sanity-check.
    BallConfig ovalCfg = cfg;
    ovalCfg.elongation = 1.8f;
    JPH::BodyID oval =
        balls.CreateBall(world.physics, ovalCfg, Vec3{0, 10, 20}, noRot, Layers::MOVING);
    for (int i = 0; i < 60; ++i) {
        balls.Update(world.physics, dt);
        world.Step(dt);
    }
    const float ovalY = FromJolt(JPH::Vec3(bi.GetPosition(oval))).y;
    CHECK(ovalY < 10.0f && ovalY > -40.0f, "elips (Amerikan futbolu) topu dusup makul yukseklikte");

    balls.RemoveBall(world.physics, spinBall);
    balls.RemoveBall(world.physics, plainBall);
    balls.RemoveBall(world.physics, oval);

    if (g_failCount == 0) {
        printf("TEST BASARILI: BallPhysicsSystem surukleme+Magnus etkisini uyguluyor.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
