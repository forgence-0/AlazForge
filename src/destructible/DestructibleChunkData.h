#pragma once
// Yıkılabilir parça hasar durumunun sparse-chunk kalıcılığı (Faz B.5).
// TerrainDeformSystem/WreckPersistence ile AYNI ChunkStreamSystem<T>
// altyapısını kullanır — yalnızca dokunulmuş (hasar almış) parçaların
// bulunduğu chunk'lar diske yazılır.

#include "streaming/ChunkStreamSystem.h"

#include <destructible/export.h>

#include <cstring>

namespace alazforge {

struct PieceDamageRecord {
    uint64_t structureId = 0;
    uint16_t pieceIndex = 0;
    float health = 0.0f;
    bool broken = false;
};

struct DestructibleChunkData {
    std::vector<PieceDamageRecord> records;

    void Serialize(std::vector<uint8_t>& out) const {
        const uint32_t count = static_cast<uint32_t>(records.size());
        Append(out, &count, sizeof(count));
        for (const PieceDamageRecord& r : records) {
            Append(out, &r.structureId, sizeof(r.structureId));
            Append(out, &r.pieceIndex, sizeof(r.pieceIndex));
            Append(out, &r.health, sizeof(r.health));
            const uint8_t brokenByte = r.broken ? 1 : 0;
            Append(out, &brokenByte, sizeof(brokenByte));
        }
    }

    void Deserialize(const uint8_t* data, size_t size) {
        records.clear();
        uint32_t count = 0;
        if (size < sizeof(count)) return;
        std::memcpy(&count, data, sizeof(count));
        const uint8_t* p = data + sizeof(count);
        const size_t recordSize =
            sizeof(uint64_t) + sizeof(uint16_t) + sizeof(float) + sizeof(uint8_t);
        if (size < sizeof(count) + count * recordSize) return; // bozuk/eksik veri: boş chunk döndür
        records.resize(count);
        for (uint32_t i = 0; i < count; ++i) {
            PieceDamageRecord& r = records[i];
            Read(p, &r.structureId, sizeof(r.structureId));
            Read(p, &r.pieceIndex, sizeof(r.pieceIndex));
            Read(p, &r.health, sizeof(r.health));
            uint8_t brokenByte = 0;
            Read(p, &brokenByte, sizeof(brokenByte));
            r.broken = brokenByte != 0;
        }
    }

private:
    static void Append(std::vector<uint8_t>& out, const void* src, size_t n) {
        const uint8_t* b = static_cast<const uint8_t*>(src);
        out.insert(out.end(), b, b + n);
    }
    static void Read(const uint8_t*& p, void* dst, size_t n) {
        std::memcpy(dst, p, n);
        p += n;
    }
};

// DestructibleChunkData tam tanımlı olduktan sonra: kod destructible.dll içine
// gömülür (bkz. DestructibleChunkData.cpp), tüketici çeviri birimleri
// yeniden üretmez.
extern template class DESTRUCTIBLE_API ChunkStreamSystem<DestructibleChunkData>;

} // namespace alazforge
