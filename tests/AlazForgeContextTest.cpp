// Faz B.7 doğrulama testi: AlazForgeContext.
// Context kurup düşen bir küp ekler; SnapshotActiveBodies'in düşerken kübü
// içerdiğini, yerde uyuduktan sonra listenin boşaldığını doğrular.

#include "context/AlazForgeContext.h"

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>

#include <cstdio>
#include <vector>

using namespace alazforge;

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
    AlazForgeContext context;
    JPH::BodyInterface& bi = context.Physics().GetBodyInterface();

    JPH::BodyCreationSettings floorSettings(new JPH::BoxShape(JPH::Vec3(50.0f, 0.5f, 50.0f)),
                                            JPH::RVec3(0.0, -0.5, 0.0), JPH::Quat::sIdentity(),
                                            JPH::EMotionType::Static, 0);
    bi.CreateAndAddBody(floorSettings, JPH::EActivation::DontActivate);

    JPH::BodyCreationSettings cubeSettings(new JPH::BoxShape(JPH::Vec3(0.5f, 0.5f, 0.5f)),
                                           JPH::RVec3(0.0, 10.0, 0.0), JPH::Quat::sIdentity(),
                                           JPH::EMotionType::Dynamic, 1);
    JPH::BodyID cubeId = bi.CreateAndAddBody(cubeSettings, JPH::EActivation::Activate);
    context.Physics().OptimizeBroadPhase();

    printf("[AlazForgeContext]\n");

    std::vector<AlazForgeContext::BodySnapshot> snapshot;
    bool sawActiveWhileFalling = false;
    int step = 0;
    for (; step < 600 && bi.IsActive(cubeId); ++step) {
        context.Step(1.0f / 60.0f);
        const size_t count = context.SnapshotActiveBodies(snapshot);
        if (step == 5 && count > 0) sawActiveWhileFalling = true;
    }

    CHECK(sawActiveWhileFalling, "dusen kup ilk adimlarda aktif govdeler arasinda gorundu");
    CHECK(!bi.IsActive(cubeId), "kup sonunda uykuya gecti");

    const size_t finalCount = context.SnapshotActiveBodies(snapshot);
    CHECK(finalCount == 0, "kup uykuya gectikten sonra aktif govde listesi bos");

    bi.RemoveBody(cubeId);
    bi.DestroyBody(cubeId);

    if (g_failCount == 0) {
        printf("TEST BASARILI: AlazForgeContext Jolt yasam donguleyip snapshot uretiyor.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
