// Faz 6 doğrulama testi: araç enkaz/hasar kalıcılığı.
// Aynı ChunkStreamSystem altyapısı üzerinde enkaz ekleme, sparse depolama,
// disk round-trip (pozisyon+hasar korunur), silme ve streaming kontrolü.

#include "vehicle/WreckPersistence.h"

#include <cstdio>
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

static WreckRecord MakeWreck(uint64_t id, float x, float z, float damage) {
    WreckRecord w;
    w.id = id;
    w.vehicleType = 42;
    w.position = Vec3{x, 0.0f, z};
    w.rotation = Quat{0, 0.7071f, 0, 0.7071f};
    w.damage = damage;
    w.flags = 0x1; // "yanıyor"
    return w;
}

int main() {
    const std::string saveDir =
        (std::filesystem::temp_directory_path() / "alazforge_wreck_test").string();
    std::filesystem::remove_all(saveDir);

    ChunkGrid grid(4096, 64); // 4km harita, 64m chunk

    printf("[Ekleme ve sparse depolama]\n");
    {
        WreckPersistenceSystem wrecks(grid, 200.0f, saveDir);
        wrecks.OnPlayerMove(500.0f, 500.0f);

        // Ayni chunk'a iki enkaz (chunk (7,7): 448..512)
        wrecks.AddWreck(MakeWreck(1, 500.0f, 500.0f, 0.8f));
        wrecks.AddWreck(MakeWreck(2, 510.0f, 505.0f, 0.3f));
        CHECK(wrecks.LoadedWreckCount(500.0f, 500.0f) == 2, "ayni chunk'a 2 enkaz eklendi");

        // Uzak chunk'a bir enkaz (yaricap icinde: ~120m)
        wrecks.AddWreck(MakeWreck(3, 600.0f, 500.0f, 1.0f));
        CHECK(wrecks.LoadedWreckCount(600.0f, 500.0f) == 1, "farkli chunk'a 1 enkaz eklendi");

        CHECK(!wrecks.Stream().ChunkFileExists(grid.GetChunkCoord(500.0f, 500.0f)),
              "flush oncesi dosya yok");

        wrecks.Flush();
        CHECK(wrecks.Stream().ChunkFileExists(grid.GetChunkCoord(500.0f, 500.0f)),
              "enkazli chunk diske yazildi");
        CHECK(!wrecks.Stream().ChunkFileExists(ChunkCoord{0, 0}),
              "enkazsiz chunk'in dosyasi YOK (sparse)");
    }

    printf("[Kalicilik - disk round-trip]\n");
    {
        WreckPersistenceSystem wrecks(grid, 200.0f, saveDir);
        wrecks.OnPlayerMove(500.0f, 500.0f);

        const auto& list = wrecks.GetWrecksAt(500.0f, 500.0f);
        CHECK(list.size() == 2, "disktan 2 enkaz geri yuklendi");

        // id=1 enkazini bul, alanlarini dogrula
        const WreckRecord* w1 = nullptr;
        for (const auto& w : list)
            if (w.id == 1) w1 = &w;
        CHECK(w1 != nullptr, "id=1 enkazi bulundu");
        if (w1) {
            CHECK(std::fabs(w1->damage - 0.8f) < 1e-6f, "hasar degeri korundu (0.8)");
            CHECK(std::fabs(w1->position.x - 500.0f) < 1e-4f &&
                  std::fabs(w1->position.z - 500.0f) < 1e-4f, "pozisyon korundu");
            CHECK(std::fabs(w1->rotation.y - 0.7071f) < 1e-4f, "rotasyon korundu");
            CHECK(w1->vehicleType == 42 && w1->flags == 0x1, "arac tipi + flag korundu");
        }
    }

    printf("[Silme]\n");
    {
        WreckPersistenceSystem wrecks(grid, 200.0f, saveDir);
        wrecks.OnPlayerMove(500.0f, 500.0f);

        CHECK(wrecks.RemoveWreck(1, 500.0f, 500.0f), "id=1 enkazi silindi");
        CHECK(!wrecks.RemoveWreck(999, 500.0f, 500.0f), "olmayan id silinemez");
        CHECK(wrecks.LoadedWreckCount(500.0f, 500.0f) == 1, "silme sonrasi 1 enkaz kaldi");
        wrecks.Flush();
    }

    printf("[Streaming - uzaklas/geri don]\n");
    {
        WreckPersistenceSystem wrecks(grid, 150.0f, saveDir);
        wrecks.OnPlayerMove(500.0f, 500.0f);
        CHECK(wrecks.LoadedWreckCount(500.0f, 500.0f) == 1, "baslangicta 1 enkaz yuklu");

        // Uzaklas: chunk bosaltilir
        wrecks.OnPlayerMove(3000.0f, 3000.0f);
        CHECK(!wrecks.Stream().IsLoaded(grid.GetChunkCoord(500.0f, 500.0f)),
              "uzaklasinca chunk bosaltildi");

        // Geri don: enkaz diskten geri gelmeli (silme kalici oldu)
        wrecks.OnPlayerMove(500.0f, 500.0f);
        CHECK(wrecks.LoadedWreckCount(500.0f, 500.0f) == 1,
              "geri donunce enkaz diskten yuklendi (silme kalici)");
    }

    std::filesystem::remove_all(saveDir);

    if (g_failCount == 0) {
        printf("TEST BASARILI: enkaz kaliciligi dogru calisiyor.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
