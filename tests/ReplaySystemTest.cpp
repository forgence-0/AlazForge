// ReplaySystem dogrulama testi: fizigin deterministik oldugu (bkz.
// world_state testi) varsayimina dayanarak, KAYDEDILEN girdileri (impulse
// + dt) sifirdan tekrar oynatmanin, girdilerin DOGRUDAN uygulandigi bir
// referans simulasyonla BIT-BIREBIR ayni sonucu urettigini kanitlar.
// Ayrica bir checkpoint'ten hizli ileri sarmanin da (daha az adim
// simule ederek) ayni sonuca ulastigini dogrular.

#include "JoltTestWorld.h"
#include "core/JoltAdapter.h"
#include "scene/ReplaySystem.h"

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

static JPH::BodyID AddDynamicBox(TestWorld& world, JPH::RVec3 pos) {
    JPH::BodyCreationSettings settings(new JPH::BoxShape(JPH::Vec3(0.3f, 0.3f, 0.3f)), pos,
                                       JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic,
                                       Layers::MOVING);
    return world.physics.GetBodyInterface().CreateAndAddBody(settings, JPH::EActivation::Activate);
}

static Vec3 GetPosition(JPH::PhysicsSystem& physics, JPH::BodyID id) {
    JPH::BodyLockRead lock(physics.GetBodyLockInterface(), id);
    return FromJolt(JPH::Vec3(lock.GetBody().GetPosition()));
}

int main() {
    printf("[ReplaySystem]\n");

    const std::string path =
        (std::filesystem::temp_directory_path() / "alazforge_replay_test.bin").string();
    std::filesystem::remove(path);

    const float dt = 1.0f / 60.0f;
    const uint32_t totalFrames = 60;

    // ── Referans: girdiler dogrudan uygulanir, kayit yok ────────────────
    Vec3 refPosA, refPosB;
    {
        TestWorld world;
        world.physics.SetGravity(JPH::Vec3::sZero());
        JPH::BodyID a = AddDynamicBox(world, JPH::RVec3(0, 0, 0));
        JPH::BodyID b = AddDynamicBox(world, JPH::RVec3(5, 0, 0));
        for (uint32_t f = 0; f < totalFrames; ++f) {
            if (f == 10) world.physics.GetBodyInterface().AddImpulse(a, JPH::Vec3(20, 0, 0));
            if (f == 30) world.physics.GetBodyInterface().AddImpulse(b, JPH::Vec3(0, 15, 0));
            world.Step(dt);
        }
        refPosA = GetPosition(world.physics, a);
        refPosB = GetPosition(world.physics, b);
    }

    // ── Kayit: ayni sekilde adimlanir, impulse + checkpoint kaydedilir ──
    {
        TestWorld world;
        world.physics.SetGravity(JPH::Vec3::sZero());
        JPH::BodyID a = AddDynamicBox(world, JPH::RVec3(0, 0, 0));
        JPH::BodyID b = AddDynamicBox(world, JPH::RVec3(5, 0, 0));
        const std::vector<JPH::BodyID> tracked = {a, b};

        ReplayRecorder recorder;
        for (uint32_t f = 0; f < totalFrames; ++f) {
            if (f == 10) {
                world.physics.GetBodyInterface().AddImpulse(a, JPH::Vec3(20, 0, 0));
                recorder.RecordImpulse(a, Vec3{20, 0, 0});
            }
            if (f == 30) {
                world.physics.GetBodyInterface().AddImpulse(b, JPH::Vec3(0, 15, 0));
                recorder.RecordImpulse(b, Vec3{0, 15, 0});
            }
            world.Step(dt);
            recorder.EndFrame(dt);
            if (f == 19 || f == 39) recorder.RecordCheckpoint(world.physics, tracked);
        }

        CHECK(recorder.FrameCount() == totalFrames, "60 frame kaydedildi");
        CHECK(recorder.Checkpoints().size() == 2, "2 checkpoint kaydedildi");
        CHECK(recorder.SaveToFile(path), "kayit diske yazildi");
    }

    // ── Tam tekrar oynatma (frame 0'dan): referansla BIT-BIREBIR eslesmeli ──
    {
        TestWorld world;
        world.physics.SetGravity(JPH::Vec3::sZero());
        JPH::BodyID a = AddDynamicBox(world, JPH::RVec3(0, 0, 0));
        JPH::BodyID b = AddDynamicBox(world, JPH::RVec3(5, 0, 0));

        ReplayPlayback playback;
        CHECK(playback.LoadFromFile(path), "kayit diskten okundu");
        CHECK(playback.FrameCount() == totalFrames, "frame sayisi korunmus");

        for (uint32_t f = 0; f < playback.FrameCount(); ++f) {
            playback.ApplyFrame(world.physics, f);
            world.Step(playback.FrameDeltaTime(f));
        }

        CHECK(NearlyEqual(GetPosition(world.physics, a), refPosA),
              "tam tekrar oynatma referansla eslesti (A govdesi)");
        CHECK(NearlyEqual(GetPosition(world.physics, b), refPosB),
              "tam tekrar oynatma referansla eslesti (B govdesi)");
    }

    // ── Checkpoint'ten hizli ileri sarma: daha az adimla ayni sonuc ─────
    {
        TestWorld world;
        world.physics.SetGravity(JPH::Vec3::sZero());
        JPH::BodyID a = AddDynamicBox(world, JPH::RVec3(0, 0, 0));
        JPH::BodyID b = AddDynamicBox(world, JPH::RVec3(5, 0, 0));

        ReplayPlayback playback;
        CHECK(playback.LoadFromFile(path), "kayit tekrar okundu (checkpoint testi icin)");

        const uint32_t restoredFrame = playback.RestoreNearestCheckpoint(world.physics, 45);
        CHECK(restoredFrame == 40, "frame 45 icin en yakin checkpoint (40) bulundu");

        uint32_t stepsSimulated = 0;
        for (uint32_t f = restoredFrame; f < playback.FrameCount(); ++f) {
            playback.ApplyFrame(world.physics, f);
            world.Step(playback.FrameDeltaTime(f));
            ++stepsSimulated;
        }
        CHECK(stepsSimulated == 20, "yalnizca 20 adim simule edilerek 60 adimlik sonuca ulasildi");

        CHECK(NearlyEqual(GetPosition(world.physics, a), refPosA),
              "checkpoint'ten hizli ileri sarma referansla eslesti (A govdesi)");
        CHECK(NearlyEqual(GetPosition(world.physics, b), refPosB),
              "checkpoint'ten hizli ileri sarma referansla eslesti (B govdesi)");
    }

    printf("[ReplaySystem] %s (%d hata)\n", g_failCount == 0 ? "GECTI" : "BASARISIZ", g_failCount);
    return g_failCount == 0 ? 0 : 1;
}
