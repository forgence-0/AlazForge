#pragma once
// Sorgu API'si kolaylıkları: raycast/sweep/overlap için yüksek seviye,
// kullanımı kolay sarmalayıcı fonksiyonlar. Jolt'un kendi
// `NarrowPhaseQuery` API'si güçlü ama düşük seviyeli (collector şablonları,
// elle kurulan RRayCast/CollideShapeSettings) -- bu dosya, AlazEngine
// tarafının en sık ihtiyaç duyacağı üç sorguyu (en yakın ışın çarpması,
// küre/kutu örtüşmesi, küre süpürmesi) tek fonksiyon çağrısına indirger.

#include "core/AlazMath.h"

#include <physics/export.h>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <vector>

namespace alazforge {

struct RaycastHit {
    bool hit = false;
    JPH::BodyID body;
    Vec3 point{0, 0, 0};
    Vec3 normal{0, 0, 0};
    float fraction = 1.0f; // [0,1] -- point = origin + fraction * (direction * maxDistance)
};

// origin'den direction yönünde (otomatik normalize edilir) en fazla
// maxDistance uzaklığa kadar en yakın çarpışmayı bulur. Çarpışma yoksa
// RaycastHit::hit == false döner.
PHYSICS_API RaycastHit RaycastClosest(JPH::PhysicsSystem& physics, const Vec3& origin,
                                      const Vec3& direction, float maxDistance);

// center merkezli radius yarıçaplı küreyle örtüşen tüm gövdelerin
// BodyID'lerini döner (tekrarsız).
PHYSICS_API std::vector<JPH::BodyID> OverlapSphere(JPH::PhysicsSystem& physics, const Vec3& center,
                                                   float radius);

// center merkezli, eksene hizalı halfExtents boyutlu kutuyla örtüşen tüm
// gövdelerin BodyID'lerini döner.
PHYSICS_API std::vector<JPH::BodyID> OverlapBox(JPH::PhysicsSystem& physics, const Vec3& center,
                                                const Vec3& halfExtents);

// radius yarıçaplı bir küreyi start'tan direction yönünde (otomatik
// normalize edilir) maxDistance mesafeye kadar süpürüp ilk çarpışmayı
// bulur (mermi/karakter hareketi gibi "hacimli ışın" sorguları için --
// ince bir raycast'in atlayabileceği dar engelleri yakalar). Jolt'un
// gerçek `NarrowPhaseQuery::CastShape` API'sini kullanır -- ayrık
// örnekleme/yaklaşıklık YAPILMAZ.
PHYSICS_API bool SweepSphere(JPH::PhysicsSystem& physics, const Vec3& start, const Vec3& direction,
                             float maxDistance, float radius, RaycastHit& outHit);

} // namespace alazforge
