#pragma once
// Yıkılabilir yapı veri tipleri (Faz B.5).

#include "core/AlazMath.h"
#include "material_db/MaterialDatabase.h"

#include <cstdint>
#include <vector>

namespace alazforge {

// Tek bir parça (ilk sürüm: kutu). supportNeighbors, bu parçanın yapısal
// olarak dayandığı komşu parça indekslerini listeler (örn. üst sıra bir
// duvar, altındaki parçalara dayanır).
struct PieceConfig {
    Vec3 localCenter;
    Vec3 halfExtents{0.5f, 0.5f, 0.5f};
    float healthMax = 100.0f;
    MaterialId material = kInvalidMaterial;
    std::vector<uint16_t> supportNeighbors;
};

struct StructureConfig {
    uint64_t structureId = 0;
    Vec3 worldOrigin;
    std::vector<PieceConfig> pieces;
};

struct PieceBrokenEvent {
    uint64_t structureId = 0;
    uint16_t pieceIndex = 0;
    Vec3 worldPosition;
    MaterialId material = kInvalidMaterial;
};

} // namespace alazforge
