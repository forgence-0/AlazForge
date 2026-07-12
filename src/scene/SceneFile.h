#pragma once
// Sahne dosya formatı (S2): gövde listesini (şekil, konum, malzeme, hareket
// tipi) insan-okunabilir JSON dosyasına kaydedip geri yükler. AlazEngine
// entegrasyonunun ihtiyaç duyduğu "bir seviye/sahneyi diskten kur" akışının
// fizik tarafı -- render/oyun-mantığı verisi (mesh, texture, script vb.)
// bu formatın kapsamı DIŞINDA, yalnızca fizik gövdeleri tanımlanır.
//
// Kalıcılık formatı olarak `ChunkStreamSystem`'in kullandığı ikili/LZ4
// sıkıştırmalı chunk formatı YERİNE düz JSON seçildi çünkü sahne dosyaları
// (terrain/destructible chunk verisinin aksine) tipik olarak küçük, elle
// düzenlenebilir/versiyon kontrolüne uygun olması istenen içeriktir.

#include "core/AlazMath.h"
#include "material_db/MaterialDatabase.h"

#include <physics/export.h>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <string>
#include <vector>

namespace alazforge {

enum class SceneShapeType : uint8_t { Box, Sphere, Capsule };

struct SceneBodyDef {
    SceneShapeType shape = SceneShapeType::Box;
    Vec3 halfExtents{0.5f, 0.5f, 0.5f}; // Box
    float radius = 0.5f;                // Sphere/Capsule
    float halfHeight = 0.5f;            // Capsule (silindirik kısmın yarı boyu)

    Vec3 position{0, 0, 0};
    Quat rotation{0, 0, 0, 1};

    bool isStatic = false;
    JPH::ObjectLayer layer = 1;
    MaterialId material = kInvalidMaterial;
    float massOverride = -1.0f; // <=0: Jolt şekilden otomatik hesaplar
};

struct SceneDefinition {
    Vec3 gravity{0, -9.81f, 0};
    std::vector<SceneBodyDef> bodies;
};

// scene'i insan-okunabilir JSON olarak path'e yazar. Başarısızsa false.
PHYSICS_API bool SaveSceneFile(const std::string& path, const SceneDefinition& scene);

// path'ten sahneyi okur. Dosya yoksa/bozuksa false döner, outScene
// değiştirilmez.
PHYSICS_API bool LoadSceneFile(const std::string& path, SceneDefinition& outScene);

// scene'deki her gövdeyi fizik dünyasında gerçekten oluşturur (Jolt
// gövdeleri kurar); oluşturulma sırasıyla eşleşen BodyID listesini döner.
PHYSICS_API std::vector<JPH::BodyID> InstantiateScene(JPH::PhysicsSystem& physics,
                                                      const SceneDefinition& scene);

} // namespace alazforge
