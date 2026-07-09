// Faz 3 doğrulama testi: terrain deformasyon modülü.
// Hücre güncelleme (şişme yok), maxDepth kenetleme, dairesel temas,
// chunk sınırı aşımı ve disk kalıcılığı kontrol edilir.

#include "terrain_deform/TerrainDeformSystem.h"

#include <cstdio>
#include <filesystem>

using namespace alazforge;

static int g_failCount = 0;

#define CHECK(cond, msg)                                      \
    do {                                                      \
        if (cond) {                                           \
            printf("  OK  : %s\n", msg);                      \
        } else {                                              \
            printf("  FAIL: %s (satir %d)\n", msg, __LINE__); \
            ++g_failCount;                                    \
        }                                                     \
    } while (0)

int main() {
    const std::string saveDir =
        (std::filesystem::temp_directory_path() / "alazforge_deform_test").string();
    std::filesystem::remove_all(saveDir);

    TerrainDeformSystem::Config cfg;
    cfg.mapSize = 4096;
    cfg.chunkSize = 64;   // varsayılan 64m chunk
    cfg.cellSize = 0.25f; // varsayılan 25cm grid
    cfg.maxDepth = 1.0f;
    cfg.loadRadius = 150.0f;
    cfg.savePath = saveDir;

    printf("[Parametreler]\n");
    {
        TerrainDeformSystem deform(cfg);
        CHECK(deform.CellsPerSide() == 256, "64m / 25cm = 256x256 hucre");

        // Parametrik olmali: farkli konfigurasyon farkli cozunurluk verir
        TerrainDeformSystem::Config cfg2 = cfg;
        cfg2.chunkSize = 32;
        cfg2.cellSize = 0.5f;
        cfg2.savePath = saveDir + "_alt";
        TerrainDeformSystem deform2(cfg2);
        CHECK(deform2.CellsPerSide() == 64, "32m / 50cm = 64x64 hucre (parametrik)");
        std::filesystem::remove_all(cfg2.savePath);
    }
    std::filesystem::remove_all(saveDir);

    printf("[Hucre guncelleme]\n");
    {
        TerrainDeformSystem deform(cfg);
        deform.OnPlayerMove(500.0f, 500.0f);

        // Tekerlek ayni noktaya 5 kez bastı
        for (int i = 0; i < 5; ++i)
            deform.ApplyDeformation(500.0f, 500.0f, 0.05f);

        CHECK(std::fabs(deform.GetDepthAt(500.0f, 500.0f) - 0.25f) < 1e-5f,
              "5 x 5cm temas = 25cm cokme (birikiyor)");
        CHECK(deform.TouchedCellCount(500.0f, 500.0f) == 1,
              "ayni hucreye 5 dokunus = 1 kayit (sisme yok)");

        // maxDepth kenetleme: cok sayida temas 1m'yi asamaz
        for (int i = 0; i < 100; ++i)
            deform.ApplyDeformation(500.0f, 500.0f, 0.05f);
        CHECK(std::fabs(deform.GetDepthAt(500.0f, 500.0f) - cfg.maxDepth) < 1e-5f,
              "cokme maxDepth'te (1m) kenetlendi");

        // Komsu hucre etkilenmedi (25cm grid -> 30cm otesi ayri hucre)
        CHECK(deform.GetDepthAt(500.6f, 500.0f) == 0.0f, "komsu hucre etkilenmedi");
    }
    std::filesystem::remove_all(saveDir);

    printf("[Dairesel temas]\n");
    {
        TerrainDeformSystem deform(cfg);
        deform.OnPlayerMove(500.0f, 500.0f);

        // 1m yaricapli temas (buyuk tekerlek/kucuk patlama)
        deform.ApplyDeformationRadius(500.0f, 500.0f, 0.2f, 1.0f);

        const float center = deform.GetDepthAt(500.0f, 500.0f);
        const float edge = deform.GetDepthAt(500.875f, 500.0f);
        CHECK(center > 0.15f, "merkez hucre derin cokme aldi");
        CHECK(edge > 0.0f && edge < center, "kenar hucre daha az cokme aldi (falloff)");
        CHECK(deform.GetDepthAt(502.0f, 500.0f) == 0.0f, "yaricap disi etkilenmedi");

        size_t touched = deform.TouchedCellCount(500.0f, 500.0f);
        CHECK(touched >= 40 && touched <= 60, "1m yaricap ~50 hucre dokundu (dairesel)");
    }
    std::filesystem::remove_all(saveDir);

    printf("[Chunk siniri]\n");
    {
        TerrainDeformSystem deform(cfg);
        deform.OnPlayerMove(128.0f, 128.0f);

        // Chunk siniri x=128'de: dairesel temas iki chunk'a da yazmali
        deform.ApplyDeformationRadius(128.0f, 100.0f, 0.1f, 1.0f);
        CHECK(deform.GetDepthAt(127.6f, 100.0f) > 0.0f, "sinirin sol chunk'ina yazildi");
        CHECK(deform.GetDepthAt(128.4f, 100.0f) > 0.0f, "sinirin sag chunk'ina yazildi");
    }
    std::filesystem::remove_all(saveDir);

    printf("[Kalicilik]\n");
    {
        {
            TerrainDeformSystem deform(cfg);
            deform.OnPlayerMove(500.0f, 500.0f);
            deform.ApplyDeformation(500.0f, 500.0f, 0.4f);
            deform.Flush();
        } // yikici: destructor da flush eder

        // Yeni sistem ayni klasorden okur — tekerlek izi kalici
        TerrainDeformSystem deform(cfg);
        deform.OnPlayerMove(500.0f, 500.0f);
        CHECK(std::fabs(deform.GetDepthAt(500.0f, 500.0f) - 0.4f) < 1e-5f,
              "cokme diske yazilip geri yuklendi");
        CHECK(!deform.Stream().ChunkFileExists(ChunkCoord{0, 0}),
              "dokunulmamis chunk'in dosyasi yok (sparse korunuyor)");
    }
    std::filesystem::remove_all(saveDir);

    if (g_failCount == 0) {
        printf("TEST BASARILI: terrain deformasyon modulu dogru calisiyor.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
