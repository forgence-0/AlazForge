#pragma once
// Basit hacimsel yumuşak-cisim: ClothSystem'in (2B yüzey ağı) 3B/hacimsel
// genellemesi. Bir dikdörtgen prizma, vertex grid'ine bölünüp her hücre 6
// tetrahedrona ayrılır; her tetrahedron için Jolt'un GERÇEK hacim koruma
// kısıtı (JPH::SoftBodySharedSettings::Volume) eklenir -- bu yüzden sıkışan
// bir cisim (ör. yere düşünce) yanlara doğru şişer/hacmini korur, salt
// yüzey ağı (kumaş) gibi çökmez. Dış yüzey CreateConstraints ile kenar/
// makas kısıtlarına da sahiptir (yüzey gerginliği).
//
// Kullanım alanları: jöle, şişirilebilir/yumuşak nesneler, basit organik
// gövde simülasyonu -- rijit gövdelerin veya ClothSystem'in karşılamadığı
// "sıkışınca hacmini koruyarak deforme olan cisim" ihtiyacı için.

#include "core/JoltAdapter.h"

#include <cloth/export.h>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/PhysicsSystem.h>

#include <vector>

namespace alazforge {

struct SoftBodyBlockConfig {
    int segmentsX = 3; // eksen başına vertex sayısı (>=2)
    int segmentsY = 3;
    int segmentsZ = 3;
    float sizeX = 1.0f; // m
    float sizeY = 1.0f;
    float sizeZ = 1.0f;
    float massPerVertex = 0.2f;       // kg
    float edgeCompliance = 1.0e-5f;   // yüzey kenar/makas esnekliği (0 = tamamen gergin)
    float volumeCompliance = 1.0e-6f; // hacim koruma esnekliği (0 = kesinlikle sabit hacim)
};

class CLOTH_API SoftBodyBlock {
public:
    explicit SoftBodyBlock(const SoftBodyBlockConfig& inConfig);
    ~SoftBodyBlock();

    SoftBodyBlock(const SoftBodyBlock&) = delete;
    SoftBodyBlock& operator=(const SoftBodyBlock&) = delete;

    // worldCenter: prizmanın geometrik merkezinin dünya pozisyonu.
    void Create(JPH::PhysicsSystem& inPhysics, const Vec3& worldCenter, JPH::ObjectLayer layer);

    // Dünya-uzayı vertex pozisyonları (x + segmentsX*(y + segmentsY*z) sırası)
    size_t GetVertexPositions(std::vector<Vec3>& out) const;

    // Şu anki (deforme olmuş) vertex konumlarından tüm tetrahedronların
    // toplam hacmi -- hacim korumasının gerçekten çalıştığını doğrulamak
    // için (test/tanılama amaçlı).
    float ComputeVolume() const;

    int VertexCount() const { return config.segmentsX * config.segmentsY * config.segmentsZ; }
    JPH::BodyID GetBodyID() const { return body; }

    void Destroy();

private:
    SoftBodyBlockConfig config;
    JPH::PhysicsSystem* physics = nullptr;
    JPH::BodyID body;
};

} // namespace alazforge
