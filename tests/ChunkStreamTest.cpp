// Faz 2 doğrulama testi: ChunkGrid koordinat matematiği + ChunkStreamSystem
// yükleme/boşaltma, sparse storage ve LZ4 disk round-trip kontrolleri.

#include "streaming/ChunkStreamSystem.h"

#include <cstdio>
#include <cstring>
#include <filesystem>

using namespace alazforge;

static int g_failCount = 0;

#define CHECK(cond, msg)                                        \
    do {                                                        \
        if (cond) {                                             \
            printf("  OK  : %s\n", msg);                        \
        } else {                                                \
            printf("  FAIL: %s (satir %d)\n", msg, __LINE__);   \
            ++g_failCount;                                      \
        }                                                       \
    } while (0)

// Brief'teki örnek: 64m chunk, 25cm grid -> 256x256 hücre
struct DeformationChunkData {
    std::vector<uint8_t> heightMap;

    void Serialize(std::vector<uint8_t>& out) const {
        uint32_t count = static_cast<uint32_t>(heightMap.size());
        const uint8_t* p = reinterpret_cast<const uint8_t*>(&count);
        out.insert(out.end(), p, p + sizeof(count));
        out.insert(out.end(), heightMap.begin(), heightMap.end());
    }

    void Deserialize(const uint8_t* data, size_t size) {
        uint32_t count = 0;
        if (size < sizeof(count))
            return;
        std::memcpy(&count, data, sizeof(count));
        heightMap.assign(data + sizeof(count), data + sizeof(count) + count);
    }
};

int main() {
    // ── ChunkGrid matematiği ────────────────────────────────────────
    printf("[ChunkGrid]\n");
    ChunkGrid grid(4096, 64); // 4km harita, 64m chunk -> 64x64 chunk

    CHECK(grid.chunksPerSide == 64, "chunksPerSide = 64");
    CHECK((grid.GetChunkCoord(0.0f, 0.0f) == ChunkCoord{0, 0}), "origin chunk (0,0)");
    CHECK((grid.GetChunkCoord(63.9f, 63.9f) == ChunkCoord{0, 0}), "63.9m hala chunk (0,0)");
    CHECK((grid.GetChunkCoord(64.0f, 128.0f) == ChunkCoord{1, 2}), "64m,128m -> chunk (1,2)");
    CHECK((grid.GetChunkCoord(-5.0f, 99999.0f) == ChunkCoord{0, 63}), "sinir disi kenetlenir");

    // Yarıçap: chunk (8,8) merkezinde (544,544), 100m yarıçap
    auto inRadius = grid.GetChunksInRadius(544.0f, 544.0f, 100.0f);
    bool centerIncluded = false;
    for (const auto& c : inRadius)
        if (c == ChunkCoord{8, 8}) centerIncluded = true;
    CHECK(centerIncluded, "merkez chunk yaricap listesinde");
    CHECK(inRadius.size() >= 9 && inRadius.size() <= 21,
          "100m yaricap makul chunk sayisi (9-21)");

    // ── ChunkStreamSystem ───────────────────────────────────────────
    printf("[ChunkStreamSystem]\n");
    const std::string saveDir =
        (std::filesystem::temp_directory_path() / "alazforge_chunk_test").string();
    std::filesystem::remove_all(saveDir);

    {
        ChunkStreamSystem<DeformationChunkData> stream(grid, 100.0f, saveDir);

        stream.OnPlayerMove(544.0f, 544.0f);
        CHECK(stream.LoadedChunkCount() >= 9, "OnPlayerMove chunk'lari yukledi");

        // Chunk (8,8)'e deformasyon yaz
        DeformationChunkData& chunk = stream.GetChunkAt(544.0f, 544.0f);
        chunk.heightMap.assign(256 * 256, 0);
        chunk.heightMap[1234] = 77;
        chunk.heightMap[65535] = 200;
        stream.MarkDirty(ChunkCoord{8, 8});

        // Sparse kontrol: flush'tan once hicbir dosya yok
        CHECK(!stream.ChunkFileExists(ChunkCoord{8, 8}), "flush oncesi dosya yok");

        stream.FlushDirtyChunks();
        CHECK(stream.ChunkFileExists(ChunkCoord{8, 8}), "flush sonrasi kirli chunk dosyasi var");
        CHECK(!stream.ChunkFileExists(ChunkCoord{8, 9}), "dokunulmamis chunk'in dosyasi YOK (sparse)");

        // Uzaklas: chunk (8,8) yaricap disina cikar, RAM'den atilir
        stream.OnPlayerMove(3000.0f, 3000.0f);
        CHECK(!stream.IsLoaded(ChunkCoord{8, 8}), "uzaklasinca chunk bosaltildi");

        // Geri don: diskteki veri geri yuklenmeli
        stream.OnPlayerMove(544.0f, 544.0f);
        DeformationChunkData& reloaded = stream.GetChunkAt(544.0f, 544.0f);
        CHECK(reloaded.heightMap.size() == 256 * 256, "reload: boyut korundu");
        CHECK(reloaded.heightMap[1234] == 77 && reloaded.heightMap[65535] == 200,
              "reload: deformasyon degerleri korundu (LZ4 round-trip)");
    }

    // Sıkıştırma etkinliği: 64KB'lik çoğu sıfır heightmap küçük dosya olmalı
    uintmax_t fileSize = std::filesystem::file_size(
        std::filesystem::path(saveDir) / "chunk_8_8.afc");
    printf("  bilgi: 65548 bayt ham veri -> %llu bayt dosya\n",
           static_cast<unsigned long long>(fileSize));
    CHECK(fileSize < 4096, "LZ4 sikistirma etkili (<4KB)");

    std::filesystem::remove_all(saveDir);

    if (g_failCount == 0) {
        printf("TEST BASARILI: streaming altyapisi dogru calisiyor.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
