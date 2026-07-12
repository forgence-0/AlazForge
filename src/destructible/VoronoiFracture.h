#pragma once
// Bir bölgeyi (AABB) verilen tohum noktalarına göre GERÇEK Voronoi
// hücrelerine ayırır (yarı-uzay kırpma / half-space clipping ile tam doğru
// dışbükey çokyüzlü sınırları — basitleştirilmiş/sahte bir bölünme değil)
// ve her hücre için gerçek bir JPH::ConvexHullShape gövdesi oluşturur.
//
// Mevcut önceden-tanımlı parça grafiği tabanlı DestructibleStructureSystem'e
// EK, bağımsız/alternatif bir kırılma modudur — darbe noktasından gerçek
// zamanlı, düzensiz şekilli parçalanma istendiğinde kullanılır (cam,
// taş, beton gibi kırılgan malzemeler için önceden tanımlı parça grafiği
// yerine).
//
// AlazForge kendi RNG'sini üretmez (kod tabanının genelindeki determinizm
// kuralı): tohum noktaları ÇAĞIRAN tarafından sağlanır.

#include "core/AlazMath.h"
#include "material_db/MaterialDatabase.h"

#include <destructible/export.h>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <vector>

namespace alazforge {

struct VoronoiCell {
    std::vector<Vec3> vertices; // hücre sınır noktaları (JPH::ConvexHullShape girdisi)
    Vec3 centroid{0, 0, 0};
};

// boxMin/boxMax bölgesini tohum noktalarına göre Voronoi hücrelerine böler.
// Saf geometri -- fizik motoruna veya gövde oluşturmaya dokunmaz, bu yüzden
// bağımsız test edilebilir.
DESTRUCTIBLE_API std::vector<VoronoiCell> ComputeVoronoiCells(const Vec3& boxMin,
                                                              const Vec3& boxMax,
                                                              const std::vector<Vec3>& seeds);

struct VoronoiFractureConfig {
    MaterialId material = kInvalidMaterial;
    float density = 1000.0f;        // kg/m^3 (parça kütlesi hücre hacmine göre ölçeklenir)
    float impactImpulseN = 2000.0f; // darbe noktasından dışarı doğru itme büyüklüğü
};

// ComputeVoronoiCells + her hücre için gerçek JPH::ConvexHullShape gövdesi
// oluşturma + darbe noktasından radyal dışa itme. Dönen BodyID listesi
// çağırana aittir -- normal gövdeler gibi yönetilir (AlazForge kendi
// kaydını tutmaz, DestructibleStructureSystem'in parça grafiğinden
// bağımsızdır).
DESTRUCTIBLE_API std::vector<JPH::BodyID> FractureVoronoi(JPH::PhysicsSystem& physics,
                                                          JPH::ObjectLayer layer,
                                                          const Vec3& boxMin, const Vec3& boxMax,
                                                          const std::vector<Vec3>& seeds,
                                                          const Vec3& impactPoint,
                                                          const VoronoiFractureConfig& config = {});

} // namespace alazforge
