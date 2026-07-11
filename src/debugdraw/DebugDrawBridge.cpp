#include "debugdraw/DebugDrawBridge.h"

#ifdef JPH_DEBUG_RENDERER
#include <Jolt/Core/Color.h>
#include <Jolt/Renderer/DebugRendererSimple.h>
#endif

namespace alazforge {

namespace {

#ifdef JPH_DEBUG_RENDERER
DebugColor FromJoltColor(JPH::ColorArg c) {
    return DebugColor{static_cast<float>(c.r) / 255.0f, static_cast<float>(c.g) / 255.0f,
                      static_cast<float>(c.b) / 255.0f, static_cast<float>(c.a) / 255.0f};
}
#endif

} // namespace

#ifdef JPH_DEBUG_RENDERER

// DebugRendererSimple, kompleks geometriyi (shape mesh'leri) DrawLine/
// DrawTriangle cagrilarina kendisi indirger -- biz yalnizca bu iki
// primitifi callback'lere aktariyoruz.
struct DebugDrawBridge::Impl final : public JPH::DebugRendererSimple {
    DebugLineCallback lineCb;
    DebugTriangleCallback triangleCb;

    void DrawLine(JPH::RVec3Arg inFrom, JPH::RVec3Arg inTo, JPH::ColorArg inColor) override {
        if (lineCb)
            lineCb(Vec3{static_cast<float>(inFrom.GetX()), static_cast<float>(inFrom.GetY()),
                        static_cast<float>(inFrom.GetZ())},
                   Vec3{static_cast<float>(inTo.GetX()), static_cast<float>(inTo.GetY()),
                        static_cast<float>(inTo.GetZ())},
                   FromJoltColor(inColor));
    }

    void DrawTriangle(JPH::RVec3Arg inV1, JPH::RVec3Arg inV2, JPH::RVec3Arg inV3,
                      JPH::ColorArg inColor, ECastShadow) override {
        if (triangleCb)
            triangleCb(Vec3{static_cast<float>(inV1.GetX()), static_cast<float>(inV1.GetY()),
                            static_cast<float>(inV1.GetZ())},
                       Vec3{static_cast<float>(inV2.GetX()), static_cast<float>(inV2.GetY()),
                            static_cast<float>(inV2.GetZ())},
                       Vec3{static_cast<float>(inV3.GetX()), static_cast<float>(inV3.GetY()),
                            static_cast<float>(inV3.GetZ())},
                       FromJoltColor(inColor));
    }

    void DrawText3D(JPH::RVec3Arg, const JPH::string_view&, JPH::ColorArg, float) override {
        // Metin cizimi AlazEngine'in UI katmanina ait -- bilerek bos
    }
};

DebugDrawBridge::DebugDrawBridge() : impl(std::make_unique<Impl>()) {}
DebugDrawBridge::~DebugDrawBridge() = default;

void DebugDrawBridge::SetCallbacks(DebugLineCallback lineCb, DebugTriangleCallback triangleCb) {
    impl->lineCb = std::move(lineCb);
    impl->triangleCb = std::move(triangleCb);
}

void DebugDrawBridge::DrawBodies(JPH::PhysicsSystem& physics) {
    JPH::BodyManager::DrawSettings settings;
    settings.mDrawShapeWireframe = true;
    physics.DrawBodies(settings, impl.get());
}

void DebugDrawBridge::DrawConstraints(JPH::PhysicsSystem& physics) {
    physics.DrawConstraints(impl.get());
}

bool DebugDrawBridge::IsAvailable() { return true; }

#else // JPH_DEBUG_RENDERER tanimli degil: no-op implementasyon

struct DebugDrawBridge::Impl {};

DebugDrawBridge::DebugDrawBridge() : impl(std::make_unique<Impl>()) {}
DebugDrawBridge::~DebugDrawBridge() = default;

void DebugDrawBridge::SetCallbacks(DebugLineCallback, DebugTriangleCallback) {}
void DebugDrawBridge::DrawBodies(JPH::PhysicsSystem&) {}
void DebugDrawBridge::DrawConstraints(JPH::PhysicsSystem&) {}
bool DebugDrawBridge::IsAvailable() { return false; }

#endif

} // namespace alazforge
