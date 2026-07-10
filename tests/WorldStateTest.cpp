// Dunya kaydet/yukle + determinizm dogrulama testi.
//
// 1) Save/load: sahne kur, durumu kaydet, N adim ilerlet, durumu geri yukle
//    -- pozisyonlar kayit anindakiyle birebir ayni olmali.
// 2) Determinizm: geri yuklenen durumdan ayni N adim tekrar kosulur --
//    ilk kosumla BIT-BIREBIR ayni sonucu vermeli (multiplayer lockstep'in
//    on kosulu; Jolt ayni binary'de deterministiktir).

#include "context/AlazForgeContext.h"

#include <Jolt/Jolt.h>

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

namespace {

std::vector<JPH::BodyID> BuildScene(AlazForgeContext& ctx) {
    JPH::BodyInterface& bi = ctx.Physics().GetBodyInterface();

    JPH::BodyCreationSettings ground(new JPH::BoxShape(JPH::Vec3(50, 1, 50)), JPH::RVec3(0, -1, 0),
                                     JPH::Quat::sIdentity(), JPH::EMotionType::Static, 0);
    bi.CreateAndAddBody(ground, JPH::EActivation::DontActivate);

    std::vector<JPH::BodyID> boxes;
    for (int i = 0; i < 20; ++i) {
        JPH::BodyCreationSettings s(new JPH::BoxShape(JPH::Vec3(0.5f, 0.5f, 0.5f)),
                                    JPH::RVec3(i % 5 * 1.2, 3.0 + i / 5 * 1.2, 0.0),
                                    JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, 1);
        boxes.push_back(bi.CreateAndAddBody(s, JPH::EActivation::Activate));
    }
    return boxes;
}

std::vector<JPH::RVec3> Positions(AlazForgeContext& ctx, const std::vector<JPH::BodyID>& ids) {
    std::vector<JPH::RVec3> out;
    for (JPH::BodyID id : ids)
        out.push_back(ctx.Physics().GetBodyInterface().GetPosition(id));
    return out;
}

bool BitExact(const std::vector<JPH::RVec3>& a, const std::vector<JPH::RVec3>& b) {
    if (a.size() != b.size()) return false;
    for (size_t i = 0; i < a.size(); ++i)
        if (a[i].GetX() != b[i].GetX() || a[i].GetY() != b[i].GetY() || a[i].GetZ() != b[i].GetZ())
            return false;
    return true;
}

} // namespace

int main() {
    AlazForgeContext ctx;
    std::vector<JPH::BodyID> boxes = BuildScene(ctx);

    printf("[WorldState]\n");

    // Kutu yigini yari-yerlesmis bir ara duruma getir
    for (int i = 0; i < 30; ++i)
        ctx.Step(1.0f / 60.0f);

    // ── 1) Kaydet ─────────────────────────────────────────────────────
    std::vector<uint8_t> snapshot;
    const size_t bytes = ctx.SaveWorldState(snapshot);
    printf("  snapshot: %zu bayt\n", bytes);
    CHECK(bytes > 0, "SaveWorldState veri uretti");

    const std::vector<JPH::RVec3> atSave = Positions(ctx, boxes);

    // ── 2) Ilerlet, sonra geri yukle: pozisyonlar kayit anina donmeli ──
    for (int i = 0; i < 60; ++i)
        ctx.Step(1.0f / 60.0f);
    const std::vector<JPH::RVec3> afterSim = Positions(ctx, boxes);
    CHECK(!BitExact(atSave, afterSim), "60 adimda dunya gercekten degisti (test anlamli)");

    CHECK(ctx.RestoreWorldState(snapshot), "RestoreWorldState basarili");
    const std::vector<JPH::RVec3> afterRestore = Positions(ctx, boxes);
    CHECK(BitExact(atSave, afterRestore), "geri yukleme kayit anini birebir geri getirdi");

    // ── 3) Determinizm: ayni durumdan ayni adimlar ayni sonucu vermeli ─
    for (int i = 0; i < 60; ++i)
        ctx.Step(1.0f / 60.0f);
    const std::vector<JPH::RVec3> replay = Positions(ctx, boxes);
    CHECK(BitExact(afterSim, replay),
          "ayni durumdan ayni 60 adim bit-birebir ayni sonucu verdi (determinizm)");

    // ── 4) Bos/gecersiz veri reddedilmeli ──────────────────────────────
    CHECK(!ctx.RestoreWorldState({}), "bos snapshot reddedildi");

    if (g_failCount == 0) {
        printf("TEST BASARILI: dunya kaydet/yukle + determinizm dogru.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
