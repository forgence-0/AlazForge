// Bilgilendirici performans olcumu (ctest'e kayitli DEGIL -- basarisizlik
// uretmez): 1200 dinamik kutunun yigin halinde yere dokulmesi simule edilir,
// adim basina ortalama/maksimum sure raporlanir. CI loglarinda surelerin
// zaman icinde izlenmesi performans regresyonlarini erken yakalar.

#include "context/AlazForgeContext.h"

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>

#include <algorithm>
#include <chrono>
#include <cstdio>

using namespace alazforge;

int main() {
    AlazForgeContext::Config cfg;
    cfg.maxBodies = 8192;
    cfg.maxBodyPairs = 16384;
    cfg.maxContactConstraints = 16384;
    AlazForgeContext ctx(cfg);

    JPH::BodyInterface& bi = ctx.Physics().GetBodyInterface();

    // Zemin
    JPH::BodyCreationSettings groundSettings(new JPH::BoxShape(JPH::Vec3(100.0f, 1.0f, 100.0f)),
                                             JPH::RVec3(0, -1, 0), JPH::Quat::sIdentity(),
                                             JPH::EMotionType::Static, cfg.nonMovingLayer);
    bi.CreateAndAddBody(groundSettings, JPH::EActivation::DontActivate);

    // 1200 dinamik kutu: 10x12x10'luk bir yigin (aralarinda bosluklu, dusup
    // carpisarak yerlesirler -- hem broad phase hem contact solver zorlanir)
    constexpr int kCountX = 10, kCountY = 12, kCountZ = 10;
    JPH::Ref<JPH::BoxShape> boxShape = new JPH::BoxShape(JPH::Vec3(0.4f, 0.4f, 0.4f));
    for (int y = 0; y < kCountY; ++y)
        for (int z = 0; z < kCountZ; ++z)
            for (int x = 0; x < kCountX; ++x) {
                JPH::BodyCreationSettings s(
                    boxShape, JPH::RVec3(x * 1.0 - 5.0, 1.0 + y * 1.1, z * 1.0 - 5.0),
                    JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic, cfg.movingLayer);
                bi.CreateAndAddBody(s, JPH::EActivation::Activate);
            }
    ctx.Physics().OptimizeBroadPhase();

    constexpr int kSteps = 300;
    constexpr float kDt = 1.0f / 60.0f;
    double totalMs = 0.0, maxMs = 0.0;
    for (int i = 0; i < kSteps; ++i) {
        auto t0 = std::chrono::steady_clock::now();
        ctx.Step(kDt);
        auto t1 = std::chrono::steady_clock::now();
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        totalMs += ms;
        maxMs = std::max(maxMs, ms);
    }

    printf("[Benchmark] %d dinamik kutu, %d adim (dt=%.4f)\n", kCountX * kCountY * kCountZ, kSteps,
           kDt);
    printf("  toplam  : %.1f ms\n", totalMs);
    printf("  ortalama: %.3f ms/adim\n", totalMs / kSteps);
    printf("  maksimum: %.3f ms/adim\n", maxMs);
    return 0;
}
