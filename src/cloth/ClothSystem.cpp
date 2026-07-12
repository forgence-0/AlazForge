#include "cloth/ClothSystem.h"

#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>

#include <algorithm>
#include <cfloat>

namespace alazforge {

ClothSystem::ClothSystem(const ClothConfig& inConfig) : config(inConfig) {
    config.widthSegments = std::max(2, config.widthSegments);
    config.heightSegments = std::max(2, config.heightSegments);
}

ClothSystem::~ClothSystem() { Destroy(); }

void ClothSystem::Create(JPH::PhysicsSystem& inPhysics, const Vec3& topLeft,
                         JPH::ObjectLayer layer) {
    Destroy();
    physics = &inPhysics;

    const int w = config.widthSegments;
    const int h = config.heightSegments;
    const float dx = config.width / static_cast<float>(w - 1);
    const float dy = config.height / static_cast<float>(h - 1);
    const float invMass = 1.0f / std::max(1.0e-4f, config.massPerVertex);

    // Vertex grid'i: govde yerel uzayinda, (0,0,0) ust-sol kose; +x'e
    // genisler, -y'ye sarkar. Ust sira (y=0) pinTopEdge ile sabitlenir
    // (invMass=0 -> sonsuz kutle = hareketsiz).
    JPH::Ref<JPH::SoftBodySharedSettings> settings = new JPH::SoftBodySharedSettings;
    for (int y = 0; y < h; ++y)
        for (int x = 0; x < w; ++x) {
            JPH::SoftBodySharedSettings::Vertex v;
            v.mPosition =
                JPH::Float3(static_cast<float>(x) * dx, -static_cast<float>(y) * dy, 0.0f);
            v.mInvMass = (config.pinTopEdge && y == 0) ? 0.0f : invMass;
            settings->mVertices.push_back(v);
        }

    // Her grid huceresi icin iki ucgen -- kenar constraint'leri
    // CreateConstraints tarafindan yuzeylerden turetilir.
    auto index = [w](int x, int y) { return static_cast<JPH::uint32>(y * w + x); };
    for (int y = 0; y + 1 < h; ++y)
        for (int x = 0; x + 1 < w; ++x) {
            settings->AddFace(JPH::SoftBodySharedSettings::Face(index(x, y), index(x, y + 1),
                                                                index(x + 1, y + 1)));
            settings->AddFace(JPH::SoftBodySharedSettings::Face(index(x, y), index(x + 1, y + 1),
                                                                index(x + 1, y)));
        }

    const JPH::SoftBodySharedSettings::VertexAttributes attributes(config.compliance,
                                                                   config.compliance, FLT_MAX);
    settings->CreateConstraints(&attributes, 1);
    settings->Optimize();

    JPH::SoftBodyCreationSettings bodySettings(settings, ToJoltR(topLeft), JPH::Quat::sIdentity(),
                                               layer);
    body =
        physics->GetBodyInterface().CreateAndAddSoftBody(bodySettings, JPH::EActivation::Activate);
}

void ClothSystem::ApplyWind(const Vec3& wind, float dt) {
    if (!physics || body.IsInvalid()) return;
    {
        JPH::BodyLockWrite lock(physics->GetBodyLockInterface(), body);
        if (!lock.Succeeded()) return;
        JPH::Body& b = lock.GetBody();
        if (!b.IsSoftBody()) return;

        auto* motion = static_cast<JPH::SoftBodyMotionProperties*>(b.GetMotionProperties());
        const JPH::Vec3 windJ = ToJolt(wind);
        for (JPH::SoftBodyVertex& v : motion->GetVertices()) {
            if (v.mInvMass <= 0.0f) continue; // sabit vertex'ler ruzgardan etkilenmez
            // Basit surukleme: kuvvet ruzgara gore bagil hizla orantili
            const JPH::Vec3 rel = windJ - v.mVelocity;
            v.mVelocity += rel * (config.windDragPerVertex * v.mInvMass * dt);
        }
    } // ActivateBody ayni govde kilidini tekrar almaya calisir -- kilit
    // burada birakilmis olmali (gercek testte "Resource deadlock avoided"
    // olarak yakalandi)
    physics->GetBodyInterface().ActivateBody(body);
}

size_t ClothSystem::GetVertexPositions(std::vector<Vec3>& out) const {
    out.clear();
    if (!physics || body.IsInvalid()) return 0;
    JPH::BodyLockRead lock(physics->GetBodyLockInterface(), body);
    if (!lock.Succeeded()) return 0;
    const JPH::Body& b = lock.GetBody();
    if (!b.IsSoftBody()) return 0;

    const auto* motion = static_cast<const JPH::SoftBodyMotionProperties*>(b.GetMotionProperties());
    // SoftBody vertex'leri govdenin kutle merkezine goreli tutulur
    const JPH::RMat44 comTransform = b.GetCenterOfMassTransform();
    out.reserve(motion->GetVertices().size());
    for (const JPH::SoftBodyVertex& v : motion->GetVertices()) {
        const JPH::RVec3 worldPos = comTransform * v.mPosition;
        out.push_back(FromJolt(JPH::Vec3(worldPos)));
    }
    return out.size();
}

void ClothSystem::Destroy() {
    if (!physics) return;
    if (!body.IsInvalid()) {
        JPH::BodyInterface& bi = physics->GetBodyInterface();
        bi.RemoveBody(body);
        bi.DestroyBody(body);
        body = JPH::BodyID();
    }
    physics = nullptr;
}

} // namespace alazforge
