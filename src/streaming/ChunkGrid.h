#pragma once
// Chunk koordinat sistemi: dünya koordinatı <-> chunk koordinatı dönüşümü.
// Harita [0, mapSize) x [0, mapSize) aralığında, origin (0,0) köşede.

#include <cmath>
#include <cstdint>
#include <functional>
#include <vector>

namespace alazforge {

struct ChunkCoord {
    int x = 0, z = 0;

    bool operator==(const ChunkCoord& o) const { return x == o.x && z == o.z; }
    bool operator!=(const ChunkCoord& o) const { return !(*this == o); }
};

struct ChunkCoordHash {
    size_t operator()(const ChunkCoord& c) const {
        // 2D koordinat için basit ama iyi dağılımlı karma
        return std::hash<uint64_t>{}((static_cast<uint64_t>(static_cast<uint32_t>(c.x)) << 32) |
                                     static_cast<uint64_t>(static_cast<uint32_t>(c.z)));
    }
};

struct ChunkGrid {
    int mapSize = 0;   // metre (kare harita kenarı)
    int chunkSize = 0; // metre (bir chunk kenarı)
    int chunksPerSide = 0;

    ChunkGrid(int inMapSize, int inChunkSize)
        : mapSize(inMapSize),
          chunkSize(inChunkSize),
          chunksPerSide((inMapSize + inChunkSize - 1) / inChunkSize) {}

    bool IsValid(const ChunkCoord& c) const {
        return c.x >= 0 && c.x < chunksPerSide && c.z >= 0 && c.z < chunksPerSide;
    }

    // Dünya koordinatını chunk koordinatına çevirir (harita sınırına kenetler)
    ChunkCoord GetChunkCoord(float worldX, float worldZ) const {
        int cx = static_cast<int>(std::floor(worldX / static_cast<float>(chunkSize)));
        int cz = static_cast<int>(std::floor(worldZ / static_cast<float>(chunkSize)));
        if (cx < 0) cx = 0;
        if (cz < 0) cz = 0;
        if (cx >= chunksPerSide) cx = chunksPerSide - 1;
        if (cz >= chunksPerSide) cz = chunksPerSide - 1;
        return ChunkCoord{cx, cz};
    }

    // Verilen dünya noktası etrafında radius içine giren tüm chunk'lar.
    // Chunk'ın merkeze en yakın noktası ile daire kesişimi test edilir.
    std::vector<ChunkCoord> GetChunksInRadius(float worldX, float worldZ, float radius) const {
        std::vector<ChunkCoord> result;
        const float cs = static_cast<float>(chunkSize);

        int minX = static_cast<int>(std::floor((worldX - radius) / cs));
        int maxX = static_cast<int>(std::floor((worldX + radius) / cs));
        int minZ = static_cast<int>(std::floor((worldZ - radius) / cs));
        int maxZ = static_cast<int>(std::floor((worldZ + radius) / cs));

        for (int cz = minZ; cz <= maxZ; ++cz) {
            for (int cx = minX; cx <= maxX; ++cx) {
                ChunkCoord c{cx, cz};
                if (!IsValid(c)) continue;

                // Dairenin chunk dikdörtgenine en yakın noktası
                const float cxLow = static_cast<float>(cx) * cs;
                const float cxHigh = static_cast<float>(cx + 1) * cs;
                const float czLow = static_cast<float>(cz) * cs;
                const float czHigh = static_cast<float>(cz + 1) * cs;
                float nearestX = worldX;
                if (nearestX < cxLow) nearestX = cxLow;
                if (nearestX > cxHigh) nearestX = cxHigh;
                float nearestZ = worldZ;
                if (nearestZ < czLow) nearestZ = czLow;
                if (nearestZ > czHigh) nearestZ = czHigh;

                float dx = nearestX - worldX;
                float dz = nearestZ - worldZ;
                if (dx * dx + dz * dz <= radius * radius) result.push_back(c);
            }
        }
        return result;
    }
};

} // namespace alazforge
