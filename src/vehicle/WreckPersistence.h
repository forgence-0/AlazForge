#pragma once
// Araç enkaz/hasar kalıcılığı (Faz 6).
//
// Dünyada kalan hasarlı/terk edilmiş araçların pozisyon ve hasar durumunu
// saklar. Terrain deform ile AYNI ChunkStreamSystem<T> altyapısını kullanır —
// yalnızca chunk veri tipi farklıdır. Sparse: yalnızca enkaz bulunan chunk'lar
// diske yazılır.

#include "core/AlazMath.h"
#include "streaming/ChunkStreamSystem.h"

#include <cstring>

namespace alazforge {

// Tek bir enkaz kaydı. id ve vehicleType oyun tarafınca atanır (katalog id).
struct WreckRecord {
    uint64_t id = 0;
    uint32_t vehicleType = 0;
    Vec3 position;   // dünya pozisyonu
    Quat rotation;
    float damage = 0.0f; // 0 = sağlam, 1 = tamamen tahrip
    uint32_t flags = 0;  // bit maskesi (yanıyor/terk edilmiş vb. oyun tanımlı)
};

// Bir chunk içindeki tüm enkazlar. ChunkStreamSystem sözleşmesini uygular.
struct WreckChunkData {
    std::vector<WreckRecord> wrecks;

    void Serialize(std::vector<uint8_t>& out) const {
        const uint32_t count = static_cast<uint32_t>(wrecks.size());
        Append(out, &count, sizeof(count));
        // Alan alan yaz (POD dolgusuna güvenme — taşınabilir format)
        for (const WreckRecord& w : wrecks) {
            Append(out, &w.id, sizeof(w.id));
            Append(out, &w.vehicleType, sizeof(w.vehicleType));
            Append(out, &w.position, sizeof(w.position));
            Append(out, &w.rotation, sizeof(w.rotation));
            Append(out, &w.damage, sizeof(w.damage));
            Append(out, &w.flags, sizeof(w.flags));
        }
    }

    void Deserialize(const uint8_t* data, size_t size) {
        wrecks.clear();
        uint32_t count = 0;
        if (size < sizeof(count))
            return;
        std::memcpy(&count, data, sizeof(count));
        const uint8_t* p = data + sizeof(count);
        const size_t recordSize = sizeof(uint64_t) + sizeof(uint32_t) +
                                  sizeof(Vec3) + sizeof(Quat) + sizeof(float) +
                                  sizeof(uint32_t);
        if (size < sizeof(count) + count * recordSize)
            return; // bozuk/eksik veri: boş chunk döndür
        wrecks.resize(count);
        for (uint32_t i = 0; i < count; ++i) {
            WreckRecord& w = wrecks[i];
            Read(p, &w.id, sizeof(w.id));
            Read(p, &w.vehicleType, sizeof(w.vehicleType));
            Read(p, &w.position, sizeof(w.position));
            Read(p, &w.rotation, sizeof(w.rotation));
            Read(p, &w.damage, sizeof(w.damage));
            Read(p, &w.flags, sizeof(w.flags));
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

class WreckPersistenceSystem {
public:
    WreckPersistenceSystem(ChunkGrid inGrid, float loadRadius, const std::string& savePath)
        : grid(inGrid), stream(inGrid, loadRadius, savePath) {}

    void OnPlayerMove(float worldX, float worldZ) { stream.OnPlayerMove(worldX, worldZ); }
    void Flush() { stream.FlushDirtyChunks(); }

    // Enkazı, pozisyonunun düştüğü chunk'a ekler.
    void AddWreck(const WreckRecord& wreck) {
        WreckChunkData& chunk = stream.GetChunkAt(wreck.position.x, wreck.position.z);
        chunk.wrecks.push_back(wreck);
        stream.MarkDirty(grid.GetChunkCoord(wreck.position.x, wreck.position.z));
    }

    // id ile enkazı verilen konumdaki chunk'tan siler. Bulunduysa true.
    bool RemoveWreck(uint64_t id, float worldX, float worldZ) {
        WreckChunkData& chunk = stream.GetChunkAt(worldX, worldZ);
        auto& v = chunk.wrecks;
        for (size_t i = 0; i < v.size(); ++i) {
            if (v[i].id == id) {
                v[i] = v.back();
                v.pop_back();
                stream.MarkDirty(grid.GetChunkCoord(worldX, worldZ));
                return true;
            }
        }
        return false;
    }

    // Konumdaki chunk'ın tüm enkazları
    const std::vector<WreckRecord>& GetWrecksAt(float worldX, float worldZ) {
        return stream.GetChunkAt(worldX, worldZ).wrecks;
    }

    // Yüklü tüm enkazların bir mevcut kayıt sayısı (test/istatistik)
    size_t LoadedWreckCount(float worldX, float worldZ) {
        return stream.GetChunkAt(worldX, worldZ).wrecks.size();
    }

    ChunkStreamSystem<WreckChunkData>& Stream() { return stream; }
    const ChunkGrid& Grid() const { return grid; }

private:
    ChunkGrid grid;
    ChunkStreamSystem<WreckChunkData> stream;
};

} // namespace alazforge
