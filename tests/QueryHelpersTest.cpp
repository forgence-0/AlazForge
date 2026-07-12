// QueryHelpers dogrulama testi: raycast en yakin carpismayi ve dogru
// yuzey normalini buluyor mu, overlap sorgulari yaricap/kutu icindeki
// govdeleri dogru ayikliyor mu (uzaktakileri disliyor mu), sweep bir ince
// raycast'in kacirdigi bir engeli (kure hacmi sayesinde) yakaliyor mu --
// hepsi gercek Jolt NarrowPhaseQuery cagrilariyla dogrulanir.

#include "JoltTestWorld.h"
#include "scene/QueryHelpers.h"

#include <Jolt/Physics/Collision/Shape/SphereShape.h>

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

static bool NearlyEqual(float a, float b, float eps = 1.0e-3f) { return std::abs(a - b) < eps; }

static JPH::BodyID AddStaticSphere(TestWorld& world, JPH::RVec3 pos, float radius) {
    JPH::BodyCreationSettings settings(new JPH::SphereShape(radius), pos, JPH::Quat::sIdentity(),
                                       JPH::EMotionType::Static, Layers::NON_MOVING);
    return world.physics.GetBodyInterface().CreateAndAddBody(settings,
                                                             JPH::EActivation::DontActivate);
}

int main() {
    TestWorld world;
    printf("[QueryHelpers]\n");

    // Zeminde bir kutu (ustu y=1'de), ondan uzakta ayri bir kure
    const JPH::BodyID box = world.AddStaticBox(JPH::RVec3(0, 0.5, 0), JPH::Vec3(1, 0.5f, 1), 0);
    const JPH::BodyID farSphere = AddStaticSphere(world, JPH::RVec3(50, 0, 50), 1.0f);
    world.physics.OptimizeBroadPhase();

    // ── RaycastClosest ──────────────────────────────────────────────────
    {
        const RaycastHit hit = RaycastClosest(world.physics, Vec3{0, 10, 0}, Vec3{0, -1, 0}, 20.0f);
        CHECK(hit.hit, "kutunun ustune dogru atilan isin carpisma buldu");
        CHECK(hit.body == box, "carpisan govde dogru kutu");
        CHECK(NearlyEqual(hit.point.y, 1.0f), "carpisma noktasi kutunun ust yuzeyinde (y=1)");
        CHECK(hit.normal.y > 0.9f, "yuzey normali yukari yonlu");

        const RaycastHit miss =
            RaycastClosest(world.physics, Vec3{20, 10, 20}, Vec3{0, -1, 0}, 5.0f);
        CHECK(!miss.hit, "hicbir seye carpmayan isin hit=false donuyor");
    }

    // ── OverlapSphere / OverlapBox ───────────────────────────────────────
    {
        const std::vector<JPH::BodyID> near = OverlapSphere(world.physics, Vec3{0, 0.5f, 0}, 3.0f);
        bool foundBox = false, foundFar = false;
        for (JPH::BodyID id : near) {
            if (id == box) foundBox = true;
            if (id == farSphere) foundFar = true;
        }
        CHECK(foundBox, "yakin kure sorgusu kutuyu buldu");
        CHECK(!foundFar, "yakin kure sorgusu uzaktaki kureyi DISLADI");

        const std::vector<JPH::BodyID> boxOverlap =
            OverlapBox(world.physics, Vec3{0, 0.5f, 0}, Vec3{2, 2, 2});
        bool foundBox2 = false;
        for (JPH::BodyID id : boxOverlap)
            if (id == box) foundBox2 = true;
        CHECK(foundBox2, "kutu-hacim sorgusu kutuyu buldu");

        const std::vector<JPH::BodyID> emptyRegion =
            OverlapSphere(world.physics, Vec3{-100, 0, -100}, 1.0f);
        CHECK(emptyRegion.empty(), "bos bolgede overlap sorgusu sonuc uretmiyor");
    }

    // ── SweepSphere ───────────────────────────────────────────────────
    {
        // Ince bir isin kutunun tam kenarindan (x=1.4, kutunun disindan)
        // gecerse kacirir, ama yeterince buyuk bir kure sweep'i onu
        // yakalamali -- "hacimli isin" avantajinin kaniti.
        const RaycastHit thinMiss =
            RaycastClosest(world.physics, Vec3{1.4f, 10, 0}, Vec3{0, -1, 0}, 20.0f);
        CHECK(!thinMiss.hit, "ince isin kutunun kenarindan gecip kacirdi (referans)");

        RaycastHit sweepHit;
        const bool swept =
            SweepSphere(world.physics, Vec3{1.4f, 10, 0}, Vec3{0, -1, 0}, 20.0f, 0.5f, sweepHit);
        CHECK(swept, "kure sweep'i (yaricap sayesinde) kutuyu yakaladi");
        if (swept) CHECK(sweepHit.body == box, "sweep carpismasi dogru govdeyi buldu");

        RaycastHit noHit;
        const bool sweptMiss =
            SweepSphere(world.physics, Vec3{20, 10, 20}, Vec3{0, -1, 0}, 5.0f, 0.3f, noHit);
        CHECK(!sweptMiss, "bos bolgede kure sweep'i carpisma bulmuyor");
    }

    printf("[QueryHelpers] %s (%d hata)\n", g_failCount == 0 ? "GECTI" : "BASARISIZ", g_failCount);
    return g_failCount == 0 ? 0 : 1;
}
