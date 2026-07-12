#pragma once
// Debug görselleştirme köprüsü: Jolt'un DebugRenderer altyapısını AlazEngine'in
// çizim katmanına callback'lerle aktarır. AlazEngine tarafı yalnızca iki
// fonksiyon verir (çizgi + üçgen); collision shape'ler, constraint'ler ve
// AABB'ler bu iki primitife indirgenmiş olarak gelir (DebugRendererSimple
// karmaşık geometriyi kendisi üçgenlere/çizgilere ayırır).
//
// Kullanım (her frame, render'dan önce):
//   bridge.SetCallbacks(cizgiCb, ucgenCb);
//   bridge.DrawBodies(physics);       // tüm gövde shape'leri
//   bridge.DrawConstraints(physics);  // constraint noktaları/eksenleri
//
// NOT: Jolt'un DebugRenderer'ı yalnızca JPH_DEBUG_RENDERER tanımlıyken
// derlenir (Jolt CMake'i bunu Debug/Release config'lerinde PUBLIC olarak
// tanımlar — bkz. Jolt.cmake DEBUG_RENDERER_IN_DEBUG_AND_RELEASE). Tanımlı
// olmayan config'lerde bu sınıfın metodları no-op'tur, API değişmez.

#include "core/AlazMath.h"

#include <debugdraw/export.h>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/PhysicsSystem.h>

#include <functional>
#include <memory>

namespace alazforge {

// r,g,b,a: [0..1]
struct DebugColor {
    float r, g, b, a;
};

using DebugLineCallback = std::function<void(const Vec3& from, const Vec3& to, DebugColor color)>;
using DebugTriangleCallback =
    std::function<void(const Vec3& v1, const Vec3& v2, const Vec3& v3, DebugColor color)>;

class DEBUGDRAW_API DebugDrawBridge {
public:
    DebugDrawBridge();
    ~DebugDrawBridge();

    DebugDrawBridge(const DebugDrawBridge&) = delete;
    DebugDrawBridge& operator=(const DebugDrawBridge&) = delete;

    void SetCallbacks(DebugLineCallback lineCb, DebugTriangleCallback triangleCb);

    // Tüm gövdelerin collision shape'lerini (wireframe) callback'lere döker
    void DrawBodies(JPH::PhysicsSystem& physics);

    // Tüm constraint'leri (eklem noktaları/eksenleri) callback'lere döker
    void DrawConstraints(JPH::PhysicsSystem& physics);

    // Bu derlemede Jolt debug renderer'ı var mı? (yoksa Draw* no-op)
    static bool IsAvailable();

private:
    struct Impl; // JPH_DEBUG_RENDERER'a bağımlı kısım .cpp'de gizli
    std::unique_ptr<Impl> impl;
};

} // namespace alazforge
