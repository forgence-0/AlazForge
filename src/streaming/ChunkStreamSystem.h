#pragma once
// Genel spatial streaming altyapısı. Terrain deform, balistik izleri ve
// araç enkazı gibi tüm chunk tabanlı sistemler bu template'i kullanır.
//
// ChunkDataType sözleşmesi:
//   void Serialize(std::vector<uint8_t>& out) const;   // sona ekleyerek yazar
//   void Deserialize(const uint8_t* data, size_t size);
//
// Sparse storage: yalnızca MarkDirty ile işaretlenmiş (dokunulmuş) chunk'lar
// diske yazılır. Hiç dokunulmamış chunk'ın dosyası oluşmaz, RAM'de de ancak
// yükleme yarıçapına girince default-constructed olarak var olur.
//
// Dosya formatı (.afc — AlazForge Chunk):
//   u32 magic 'AFC1' | u32 uncompressedSize | u32 compressedSize | LZ4 verisi

#include "streaming/ChunkGrid.h"

#include <lz4.h>

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace alazforge {

template <typename ChunkDataType>
class ChunkStreamSystem {
public:
    ChunkStreamSystem(ChunkGrid inGrid, float inLoadRadius, const std::string& inSavePath)
        : grid(inGrid), loadRadius(inLoadRadius), savePath(inSavePath) {
        std::filesystem::create_directories(savePath);
    }

    ~ChunkStreamSystem() { FlushDirtyChunks(); }

    // Oyuncu hareket ettikçe çağrılır: yarıçapa girenler yüklenir,
    // çıkanlar (kirliyse diske yazılıp) RAM'den atılır.
    void OnPlayerMove(float worldX, float worldZ) {
        std::vector<ChunkCoord> needed = grid.GetChunksInRadius(worldX, worldZ, loadRadius);

        std::unordered_set<ChunkCoord, ChunkCoordHash> neededSet(needed.begin(), needed.end());

        // Yarıçap dışına çıkanları boşalt
        std::vector<ChunkCoord> toUnload;
        for (const auto& [coord, data] : loadedChunks) {
            if (neededSet.find(coord) == neededSet.end())
                toUnload.push_back(coord);
        }
        for (const ChunkCoord& c : toUnload)
            UnloadChunk(c);

        // Yeni girenleri yükle
        for (const ChunkCoord& c : needed) {
            if (loadedChunks.find(c) == loadedChunks.end())
                LoadChunk(c);
        }
    }

    // Koordinattaki chunk'ı döndürür; yüklü değilse yükler/oluşturur.
    ChunkDataType& GetChunkAt(float worldX, float worldZ) {
        ChunkCoord c = grid.GetChunkCoord(worldX, worldZ);
        auto it = loadedChunks.find(c);
        if (it == loadedChunks.end()) {
            LoadChunk(c);
            it = loadedChunks.find(c);
        }
        return it->second;
    }

    void MarkDirty(const ChunkCoord& coord) { dirtyChunks.insert(coord); }

    // Kirli chunk'ları diske yazar (RAM'de tutmaya devam eder)
    void FlushDirtyChunks() {
        for (const ChunkCoord& c : dirtyChunks) {
            auto it = loadedChunks.find(c);
            if (it != loadedChunks.end())
                WriteChunkFile(c, it->second);
        }
        dirtyChunks.clear();
    }

    size_t LoadedChunkCount() const { return loadedChunks.size(); }
    bool IsLoaded(const ChunkCoord& c) const { return loadedChunks.count(c) != 0; }

    bool ChunkFileExists(const ChunkCoord& coord) const {
        return std::filesystem::exists(ChunkFilePath(coord));
    }

private:
    ChunkGrid grid;
    float loadRadius;
    std::string savePath;
    std::unordered_map<ChunkCoord, ChunkDataType, ChunkCoordHash> loadedChunks;
    std::unordered_set<ChunkCoord, ChunkCoordHash> dirtyChunks;

    static constexpr uint32_t kMagic = 0x31434641; // 'AFC1' little-endian

    std::string ChunkFilePath(const ChunkCoord& c) const {
        return savePath + "/chunk_" + std::to_string(c.x) + "_" + std::to_string(c.z) + ".afc";
    }

    void LoadChunk(const ChunkCoord& coord) {
        ChunkDataType data{};
        if (ChunkFileExists(coord))
            ReadChunkFile(coord, data);
        loadedChunks.emplace(coord, std::move(data));
    }

    void UnloadChunk(const ChunkCoord& coord) {
        auto it = loadedChunks.find(coord);
        if (it == loadedChunks.end())
            return;
        if (dirtyChunks.count(coord)) {
            WriteChunkFile(coord, it->second);
            dirtyChunks.erase(coord);
        }
        loadedChunks.erase(it);
    }

    void WriteChunkFile(const ChunkCoord& coord, const ChunkDataType& data) {
        std::vector<uint8_t> raw;
        data.Serialize(raw);

        const int maxDst = LZ4_compressBound(static_cast<int>(raw.size()));
        std::vector<uint8_t> compressed(static_cast<size_t>(maxDst));
        const int compressedSize = LZ4_compress_default(
            reinterpret_cast<const char*>(raw.data()),
            reinterpret_cast<char*>(compressed.data()),
            static_cast<int>(raw.size()), maxDst);
        if (compressedSize <= 0)
            throw std::runtime_error("AlazForge: LZ4 sikistirma basarisiz");

        FILE* f = std::fopen(ChunkFilePath(coord).c_str(), "wb");
        if (!f)
            throw std::runtime_error("AlazForge: chunk dosyasi yazilamiyor: " + ChunkFilePath(coord));

        const uint32_t magic = kMagic;
        const uint32_t rawSize = static_cast<uint32_t>(raw.size());
        const uint32_t compSize = static_cast<uint32_t>(compressedSize);
        std::fwrite(&magic, sizeof(magic), 1, f);
        std::fwrite(&rawSize, sizeof(rawSize), 1, f);
        std::fwrite(&compSize, sizeof(compSize), 1, f);
        std::fwrite(compressed.data(), 1, compSize, f);
        std::fclose(f);
    }

    void ReadChunkFile(const ChunkCoord& coord, ChunkDataType& outData) {
        const std::string path = ChunkFilePath(coord);
        FILE* f = std::fopen(path.c_str(), "rb");
        if (!f)
            throw std::runtime_error("AlazForge: chunk dosyasi acilamiyor: " + path);

        uint32_t magic = 0, rawSize = 0, compSize = 0;
        bool headerOk = std::fread(&magic, sizeof(magic), 1, f) == 1 &&
                        std::fread(&rawSize, sizeof(rawSize), 1, f) == 1 &&
                        std::fread(&compSize, sizeof(compSize), 1, f) == 1;
        if (!headerOk || magic != kMagic) {
            std::fclose(f);
            throw std::runtime_error("AlazForge: bozuk chunk dosyasi: " + path);
        }

        std::vector<uint8_t> compressed(compSize);
        const bool bodyOk = std::fread(compressed.data(), 1, compSize, f) == compSize;
        std::fclose(f);
        if (!bodyOk)
            throw std::runtime_error("AlazForge: eksik chunk verisi: " + path);

        std::vector<uint8_t> raw(rawSize);
        const int decoded = LZ4_decompress_safe(
            reinterpret_cast<const char*>(compressed.data()),
            reinterpret_cast<char*>(raw.data()),
            static_cast<int>(compSize), static_cast<int>(rawSize));
        if (decoded < 0 || static_cast<uint32_t>(decoded) != rawSize)
            throw std::runtime_error("AlazForge: LZ4 acma basarisiz: " + path);

        outData.Deserialize(raw.data(), raw.size());
    }
};

} // namespace alazforge
