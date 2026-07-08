#pragma once
// Chunk tabanlı terrain deformasyon sistemi (Faz 3).
// ChunkStreamSystem<DeformationChunkData> üzerine kuruludur.
//
// Chunk içi hücreler sparse tutulur (hücre indeksi -> çökme derinliği).
// Aynı hücreye tekrar dokunmak mevcut değeri günceller, yeni kayıt eklemez;
// böylece tekerlek aynı izden geçtikçe veri boyutu şişmez.

#include "streaming/ChunkStreamSystem.h"

#include <algorithm>
#include <cstring>

namespace alazforge {

struct DeformationChunkData {
    // hücre indeksi (z * cellsPerSide + x) -> çökme derinliği (metre, pozitif aşağı)
    std::unordered_map<uint32_t, float> cells;

    void Serialize(std::vector<uint8_t>& out) const {
        const uint32_t count = static_cast<uint32_t>(cells.size());
        Append(out, &count, sizeof(count));
        for (const auto& [index, depth] : cells) {
            Append(out, &index, sizeof(index));
            Append(out, &depth, sizeof(depth));
        }
    }

    void Deserialize(const uint8_t* data, size_t size) {
        cells.clear();
        uint32_t count = 0;
        if (size < sizeof(count))
            return;
        std::memcpy(&count, data, sizeof(count));
        const uint8_t* p = data + sizeof(count);
        const size_t entrySize = sizeof(uint32_t) + sizeof(float);
        if (size < sizeof(count) + count * entrySize)
            return; // bozuk veri: eksik chunk yerine bos chunk
        cells.reserve(count);
        for (uint32_t i = 0; i < count; ++i) {
            uint32_t index;
            float depth;
            std::memcpy(&index, p, sizeof(index));
            std::memcpy(&depth, p + sizeof(index), sizeof(depth));
            cells[index] = depth;
            p += entrySize;
        }
    }

private:
    static void Append(std::vector<uint8_t>& out, const void* src, size_t n) {
        const uint8_t* b = static_cast<const uint8_t*>(src);
        out.insert(out.end(), b, b + n);
    }
};

class TerrainDeformSystem {
public:
    struct Config {
        int mapSize = 4096;          // metre
        int chunkSize = 64;          // metre (varsayılan: 64m chunk)
        float cellSize = 0.25f;      // metre (varsayılan: 25cm grid)
        float maxDepth = 1.0f;       // metre, hücre başına çökme sınırı
        float loadRadius = 200.0f;   // metre, streaming yükleme yarıçapı
        std::string savePath = "terrain_deform";
    };

    explicit TerrainDeformSystem(const Config& inConfig)
        : config(inConfig),
          cellsPerSide(static_cast<int>(inConfig.chunkSize / inConfig.cellSize)),
          grid(inConfig.mapSize, inConfig.chunkSize),
          stream(grid, inConfig.loadRadius, inConfig.savePath) {}

    void OnPlayerMove(float worldX, float worldZ) { stream.OnPlayerMove(worldX, worldZ); }
    void Flush() { stream.FlushDirtyChunks(); }

    int CellsPerSide() const { return cellsPerSide; }

    // Tek temas noktası: hücreye 'depth' kadar çökme ekler (maxDepth'e kenetli).
    void ApplyDeformation(float worldX, float worldZ, float depth) {
        DeformationChunkData& chunk = stream.GetChunkAt(worldX, worldZ);
        const uint32_t index = CellIndex(worldX, worldZ);
        float& cell = chunk.cells[index]; // yoksa 0 olarak eklenir, varsa güncellenir
        cell = std::min(cell + depth, config.maxDepth);
        stream.MarkDirty(grid.GetChunkCoord(worldX, worldZ));
    }

    // Dairesel temas (tekerlek/patlama): radius içindeki tüm hücrelere,
    // merkezden uzaklığa göre lineer azalan çökme uygular.
    // Chunk sınırlarını doğru şekilde aşar (her hücre kendi chunk'ına yazılır).
    void ApplyDeformationRadius(float worldX, float worldZ, float depth, float radius) {
        const float cs = config.cellSize;
        const float minX = worldX - radius, maxX = worldX + radius;
        const float minZ = worldZ - radius, maxZ = worldZ + radius;

        for (float z = SnapToCellCenter(minZ); z <= maxZ; z += cs) {
            for (float x = SnapToCellCenter(minX); x <= maxX; x += cs) {
                const float dx = x - worldX, dz = z - worldZ;
                const float distSq = dx * dx + dz * dz;
                if (distSq > radius * radius)
                    continue;
                const float falloff = 1.0f - std::sqrt(distSq) / radius;
                ApplyDeformation(x, z, depth * falloff);
            }
        }
    }

    // Noktadaki çökme derinliği (dokunulmamış hücre = 0)
    float GetDepthAt(float worldX, float worldZ) {
        DeformationChunkData& chunk = stream.GetChunkAt(worldX, worldZ);
        auto it = chunk.cells.find(CellIndex(worldX, worldZ));
        return it != chunk.cells.end() ? it->second : 0.0f;
    }

    // Chunk'taki dolu hücre sayısı (test/istatistik için)
    size_t TouchedCellCount(float worldX, float worldZ) {
        return stream.GetChunkAt(worldX, worldZ).cells.size();
    }

    ChunkStreamSystem<DeformationChunkData>& Stream() { return stream; }
    const ChunkGrid& Grid() const { return grid; }

private:
    Config config;
    int cellsPerSide;
    ChunkGrid grid;
    ChunkStreamSystem<DeformationChunkData> stream;

    // Dünya koordinatını chunk içi hücre indeksine çevirir
    uint32_t CellIndex(float worldX, float worldZ) const {
        const ChunkCoord c = grid.GetChunkCoord(worldX, worldZ);
        const float localX = worldX - static_cast<float>(c.x * config.chunkSize);
        const float localZ = worldZ - static_cast<float>(c.z * config.chunkSize);
        int cx = static_cast<int>(localX / config.cellSize);
        int cz = static_cast<int>(localZ / config.cellSize);
        cx = std::clamp(cx, 0, cellsPerSide - 1);
        cz = std::clamp(cz, 0, cellsPerSide - 1);
        return static_cast<uint32_t>(cz) * static_cast<uint32_t>(cellsPerSide) +
               static_cast<uint32_t>(cx);
    }

    float SnapToCellCenter(float world) const {
        const float cs = config.cellSize;
        return (std::floor(world / cs) + 0.5f) * cs;
    }
};

} // namespace alazforge
