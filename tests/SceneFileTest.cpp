// SceneFile dogrulama testi: bir sahne tanimi (box/sphere/capsule, statik
// + dinamik, farkli malzeme/katman/kutle) diske JSON olarak yazilip geri
// okunur; tum alanlarin bit-birebir (kayan nokta icin epsilon ile)
// korundugu dogrulanir. Ardindan InstantiateScene ile GERCEK fizik
// govdeleri kurulup statigin sabit kaldigi, dinamigin dustugu kanitlanir.

#include "JoltTestWorld.h"
#include "scene/SceneFile.h"

#include <Jolt/Physics/Body/BodyLock.h>

#include <cmath>
#include <cstdio>
#include <filesystem>

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

static bool NearlyEqual(float a, float b, float eps = 1.0e-4f) { return std::abs(a - b) < eps; }
static bool NearlyEqual(const Vec3& a, const Vec3& b) {
    return NearlyEqual(a.x, b.x) && NearlyEqual(a.y, b.y) && NearlyEqual(a.z, b.z);
}

int main() {
    printf("[SceneFile]\n");

    const std::string path =
        (std::filesystem::temp_directory_path() / "alazforge_scene_test.json").string();
    std::filesystem::remove(path);

    SceneDefinition scene;
    scene.gravity = Vec3{0, -12.0f, 0};

    SceneBodyDef ground;
    ground.shape = SceneShapeType::Box;
    ground.halfExtents = Vec3{50, 1, 50};
    ground.position = Vec3{0, -1, 0};
    ground.isStatic = true;
    ground.layer = Layers::NON_MOVING;
    ground.material = 7;
    scene.bodies.push_back(ground);

    SceneBodyDef ball;
    ball.shape = SceneShapeType::Sphere;
    ball.radius = 0.4f;
    ball.position = Vec3{1.0f, 8.0f, -2.0f};
    ball.rotation = Quat{0.0f, 0.7071f, 0.0f, 0.7071f};
    ball.isStatic = false;
    ball.layer = Layers::MOVING;
    ball.material = 3;
    ball.massOverride = 5.0f;
    scene.bodies.push_back(ball);

    SceneBodyDef pillar;
    pillar.shape = SceneShapeType::Capsule;
    pillar.radius = 0.3f;
    pillar.halfHeight = 0.8f;
    pillar.position = Vec3{-3.0f, 8.0f, 0.0f};
    pillar.isStatic = false;
    pillar.layer = Layers::MOVING;
    scene.bodies.push_back(pillar);

    CHECK(SaveSceneFile(path, scene), "sahne diske yazildi");
    CHECK(std::filesystem::exists(path), "sahne dosyasi gercekten olusturuldu");

    SceneDefinition loaded;
    CHECK(LoadSceneFile(path, loaded), "sahne diskten okundu");
    CHECK(loaded.bodies.size() == 3, "3 govde geri yuklendi");
    CHECK(NearlyEqual(loaded.gravity, scene.gravity), "yercekimi korunmus");

    if (loaded.bodies.size() == 3) {
        CHECK(loaded.bodies[0].shape == SceneShapeType::Box, "1. govde box");
        CHECK(loaded.bodies[0].isStatic, "1. govde statik");
        CHECK(loaded.bodies[0].material == 7, "1. govde malzemesi korunmus");
        CHECK(NearlyEqual(loaded.bodies[0].halfExtents, ground.halfExtents),
              "box halfExtents korunmus");

        CHECK(loaded.bodies[1].shape == SceneShapeType::Sphere, "2. govde sphere");
        CHECK(NearlyEqual(loaded.bodies[1].radius, ball.radius), "sphere radius korunmus");
        CHECK(NearlyEqual(loaded.bodies[1].position, ball.position), "sphere konumu korunmus");
        CHECK(NearlyEqual(loaded.bodies[1].rotation.x, ball.rotation.x) &&
                  NearlyEqual(loaded.bodies[1].rotation.w, ball.rotation.w),
              "sphere rotasyonu korunmus");
        CHECK(NearlyEqual(loaded.bodies[1].massOverride, 5.0f), "kutle override korunmus");

        CHECK(loaded.bodies[2].shape == SceneShapeType::Capsule, "3. govde capsule");
        CHECK(NearlyEqual(loaded.bodies[2].halfHeight, pillar.halfHeight),
              "capsule halfHeight korunmus");
    }

    // ── InstantiateScene: gercek fizik dunyasi kurulumu ─────────────────
    TestWorld world;
    const std::vector<JPH::BodyID> ids = InstantiateScene(world.physics, loaded);
    CHECK(ids.size() == 3, "3 gercek Jolt govdesi olusturuldu");
    for (JPH::BodyID id : ids)
        CHECK(!id.IsInvalid(), "gecerli BodyID");
    CHECK(NearlyEqual(world.physics.GetGravity().GetY(), loaded.gravity.y),
          "sahne yercekimi fizik dunyasina uygulandi");

    const float groundStartY = [&] {
        JPH::BodyLockRead lock(world.physics.GetBodyLockInterface(), ids[0]);
        return static_cast<float>(lock.GetBody().GetPosition().GetY());
    }();
    const float ballStartY = [&] {
        JPH::BodyLockRead lock(world.physics.GetBodyLockInterface(), ids[1]);
        return static_cast<float>(lock.GetBody().GetPosition().GetY());
    }();

    for (int i = 0; i < 60; ++i)
        world.Step(1.0f / 60.0f);

    const float groundEndY = [&] {
        JPH::BodyLockRead lock(world.physics.GetBodyLockInterface(), ids[0]);
        return static_cast<float>(lock.GetBody().GetPosition().GetY());
    }();
    const float ballEndY = [&] {
        JPH::BodyLockRead lock(world.physics.GetBodyLockInterface(), ids[1]);
        return static_cast<float>(lock.GetBody().GetPosition().GetY());
    }();

    CHECK(NearlyEqual(groundStartY, groundEndY, 1.0e-5f), "statik zemin hic hareket etmedi");
    CHECK(ballEndY < ballStartY - 0.5f, "dinamik top yercekimiyle gercekten dustu");

    printf("[SceneFile] %s (%d hata)\n", g_failCount == 0 ? "GECTI" : "BASARISIZ", g_failCount);
    return g_failCount == 0 ? 0 : 1;
}
