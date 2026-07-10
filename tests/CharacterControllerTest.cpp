// CharacterController dogrulama testi: duz zeminde yurume hizina ulasma,
// ziplayip geri inme, dik duvara karsi ilerleyememe.

#include "JoltTestWorld.h"
#include "character/CharacterController.h"

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
    world.AddGround();

    // Karakterin yoluna dik bir duvar (x=+6'da, z ekseni boyunca genis)
    world.AddStaticBox(JPH::RVec3(6.0, 2.0, 0.0), JPH::Vec3(0.25f, 2.0f, 4.0f), 0);

    CharacterControllerConfig cfg;
    CharacterController controller(cfg);
    controller.Spawn(world.physics, Vec3{0, 0.1f, 0}, Layers::MOVING);

    printf("[CharacterController]\n");

    // ── 1) Duz zeminde +x yonune yuru: hiz walkSpeed'e yaklasmali ──────
    controller.SetMoveInput(Vec3{1, 0, 0}, /*running=*/false);
    for (int i = 0; i < 120; ++i) {
        controller.Update(1.0f / 60.0f);
        world.Step(1.0f / 60.0f);
    }
    const Vec3 walkVel = controller.GetVelocity();
    CHECK(controller.IsOnGround(), "yurume sirasinda zeminde");
    CHECK(std::fabs(walkVel.x - cfg.walkSpeed) < 0.5f, "yurume hizina ulasti (~4 m/s)");

    const float xBeforeWall = controller.GetWorldTransform().position.x;
    CHECK(xBeforeWall > 3.0f, "2 saniyede belirgin yol katetti");

    // ── 2) Duvara kadar yurumeye devam: duvarin arkasina GECEMEMELI ────
    for (int i = 0; i < 240; ++i) {
        controller.Update(1.0f / 60.0f);
        world.Step(1.0f / 60.0f);
    }
    const float xAtWall = controller.GetWorldTransform().position.x;
    CHECK(xAtWall < 6.0f, "dik duvarin arkasina gecemedi");

    // ── 3) Ziplama: havalanmali, sonra tekrar zemine inmeli ────────────
    controller.SetMoveInput(Vec3{0, 0, 0});
    controller.Jump();
    controller.Update(1.0f / 60.0f);
    world.Step(1.0f / 60.0f);
    CHECK(controller.GetVelocity().y > 3.0f, "ziplama ile yukari hiz kazandi");

    float maxHeight = 0.0f;
    bool landed = false;
    for (int i = 0; i < 300; ++i) {
        controller.Update(1.0f / 60.0f);
        world.Step(1.0f / 60.0f);
        maxHeight = std::max(maxHeight, controller.GetWorldTransform().position.y);
        if (i > 30 && controller.IsOnGround()) {
            landed = true;
            break;
        }
    }
    CHECK(maxHeight > 0.8f, "ziplama belirgin yukseklige ulasti");
    CHECK(landed, "ziplamadan sonra zemine geri indi");

    controller.Destroy();

    if (g_failCount == 0) {
        printf("TEST BASARILI: CharacterController yurume/duvar/ziplama dogru.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
