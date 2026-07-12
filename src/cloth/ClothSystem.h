#pragma once
// Kumaş/bayrak simülasyonu: Jolt SoftBody üzerine dikdörtgen kumaş grid'i.
// Üst kenar sabitlenir (bayrak diregi), geri kalan serbest sarkar; ApplyWind
// ile WindSystem'den (veya sabit bir rüzgar vektöründen) beslenerek kumaş
// rüzgarda dalgalanır. Render tarafı GetVertexPositions ile her frame
// dünya-uzayı vertex pozisyonlarını alıp mesh'ini günceller.

#include "core/JoltAdapter.h"

#include <cloth/export.h>

#include <Jolt/Jolt.h>

#include <Jolt/Physics/PhysicsSystem.h>

#include <vector>

namespace alazforge {

struct ClothConfig {
    int widthSegments = 12;          // yatay vertex sayısı
    int heightSegments = 10;         // dikey vertex sayısı
    float width = 3.0f;              // m
    float height = 2.0f;             // m
    float massPerVertex = 0.05f;     // kg
    float compliance = 1.0e-4f;      // kenar esnekliği (0 = tamamen gergin)
    float windDragPerVertex = 0.02f; // ApplyWind kuvvet katsayısı
    bool pinTopEdge = true;          // üst sıra sabit (bayrak direği)
};

class CLOTH_API ClothSystem {
public:
    explicit ClothSystem(const ClothConfig& inConfig);
    ~ClothSystem();

    ClothSystem(const ClothSystem&) = delete;
    ClothSystem& operator=(const ClothSystem&) = delete;

    // Kumaşı dünyaya yerleştirir. topLeft: üst-sol köşenin dünya pozisyonu;
    // kumaş +x yönünde genişler, -y yönünde sarkar.
    void Create(JPH::PhysicsSystem& inPhysics, const Vec3& topLeft, JPH::ObjectLayer layer);

    // Rüzgar kuvvetini vertex hızlarına uygular (her frame, Step'ten önce).
    // wind: dünya uzayı rüzgar hızı (örn. WindSystem::GetWindAt sonucu).
    void ApplyWind(const Vec3& wind, float dt);

    // Dünya-uzayı vertex pozisyonları (satır-major: y*width + x)
    size_t GetVertexPositions(std::vector<Vec3>& out) const;

    int VertexCount() const { return config.widthSegments * config.heightSegments; }
    JPH::BodyID GetBodyID() const { return body; }

    void Destroy();

private:
    ClothConfig config;
    JPH::PhysicsSystem* physics = nullptr;
    JPH::BodyID body;
};

} // namespace alazforge
