#pragma once
// Chunk tabanlı terrain deformasyon sistemi (Faz 3).
// ChunkStreamSystem<DeformationChunkData> üzerine kuruludur.
//
// Chunk içi hücreler sparse tutulur (hücre indeksi -> çökme derinliği).
// Aynı hücreye tekrar dokunmak mevcut değeri günceller, yeni kayıt eklemez;
// böylece tekerlek aynı izden geçtikçe veri boyutu şişmez.

#include "streaming/ChunkStreamSystem.h"

#include <physics/export.h>

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
        if (size < sizeof(count)) return;
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

// DeformationChunkData tam tanımlı olduktan sonra: bu instantiation'ın kodu
// src/streaming/ChunkStreamSystem.cpp içinde bir kez derlenip DLL'e gömülür,
// bu extern bildirim tüketici çeviri birimlerinde yeniden üretilmesini engeller.
extern template class PHYSICS_API ChunkStreamSystem<DeformationChunkData>;

class PHYSICS_API TerrainDeformSystem {
public:
    struct Config {
        int mapSize = 4096;        // metre
        int chunkSize = 64;        // metre (varsayılan: 64m chunk)
        float cellSize = 0.25f;    // metre (varsayılan: 25cm grid)
        float maxDepth = 1.0f;     // metre, hücre başına çökme sınırı
        float loadRadius = 200.0f; // metre, streaming yükleme yarıçapı
        std::string savePath = "terrain_deform";
    };

    explicit TerrainDeformSystem(const Config& inConfig);

    void OnPlayerMove(float worldX, float worldZ);
    void Flush();

    int CellsPerSide() const;

    // Tek temas noktası: hücreye 'depth' kadar çökme ekler (maxDepth'e kenetli).
    void ApplyDeformation(float worldX, float worldZ, float depth);

    // Dairesel temas (tekerlek/patlama): radius içindeki tüm hücrelere,
    // merkezden uzaklığa göre lineer azalan çökme uygular.
    // Chunk sınırlarını doğru şekilde aşar (her hücre kendi chunk'ına yazılır).
    void ApplyDeformationRadius(float worldX, float worldZ, float depth, float radius);

    // Noktadaki çökme derinliği (dokunulmamış hücre = 0)
    float GetDepthAt(float worldX, float worldZ);

    // Chunk'taki dolu hücre sayısı (test/istatistik için)
    size_t TouchedCellCount(float worldX, float worldZ);

    ChunkStreamSystem<DeformationChunkData>& Stream();
    const ChunkGrid& Grid() const;

private:
    Config config;
    int cellsPerSide;
    ChunkGrid grid;
    ChunkStreamSystem<DeformationChunkData> stream;

    // Dünya koordinatını chunk içi hücre indeksine çevirir
    uint32_t CellIndex(float worldX, float worldZ) const;
    float SnapToCellCenter(float world) const;
};

} // namespace alazforge
