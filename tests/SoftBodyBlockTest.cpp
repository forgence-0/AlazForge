// SoftBodyBlock dogrulama testi: (1) tetrahedron ayristirmasinin gercekten
// hacim-tam oldugunu (baslangic ComputeVolume() ~ kutu hacmine esit)
// kanitlar, (2) yere dusup sikisirken hacmin buyuk olcude KORUNDUGUNU
// (salt yuzey agi/kumas gibi sifira cokmedigini) dogrular -- bu, hacim
// kisitinin gercekten calistiginin fiziksel kaniti.

#include "JoltTestWorld.h"
#include "cloth/SoftBodyBlock.h"

#include <cmath>
#include <cstdio>

using namespace alazforge;
using namespace alazforge_test;

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
    TestWorld world;
    printf("[SoftBodyBlock]\n");

    world.AddGround();

    SoftBodyBlockConfig cfg;
    cfg.segmentsX = cfg.segmentsY = cfg.segmentsZ = 3;
    cfg.sizeX = cfg.sizeY = cfg.sizeZ = 1.0f;
    cfg.massPerVertex = 0.15f;
    cfg.edgeCompliance = 1.0e-5f;
    cfg.volumeCompliance = 1.0e-6f;

    SoftBodyBlock block(cfg);
    block.Create(world.physics, Vec3{0, 3.0f, 0}, Layers::MOVING);

    CHECK(block.VertexCount() == 27, "3x3x3 = 27 vertex olustu");

    const float expectedVolume = cfg.sizeX * cfg.sizeY * cfg.sizeZ;
    const float restVolume = block.ComputeVolume();
    printf("  beklenen kutu hacmi=%.4f m^3, tetrahedron ayristirmasi hacmi=%.4f m^3\n",
           expectedVolume, restVolume);
    CHECK(std::abs(restVolume - expectedVolume) < 0.02f,
          "tetrahedron ayristirmasi TAM hacimli (bosluk/ortusme yok)");

    std::vector<Vec3> startPositions;
    block.GetVertexPositions(startPositions);
    CHECK(startPositions.size() == 27, "baslangic vertex pozisyonlari okunabiliyor");

    // Yere dusup sikissin
    for (int i = 0; i < 180; ++i)
        world.Step(1.0f / 60.0f);

    std::vector<Vec3> endPositions;
    block.GetVertexPositions(endPositions);

    bool anyVertexMoved = false;
    for (size_t i = 0; i < startPositions.size() && i < endPositions.size(); ++i) {
        const float dx = endPositions[i].x - startPositions[i].x;
        const float dy = endPositions[i].y - startPositions[i].y;
        const float dz = endPositions[i].z - startPositions[i].z;
        if (std::sqrt(dx * dx + dy * dy + dz * dz) > 0.05f) anyVertexMoved = true;
    }
    CHECK(anyVertexMoved, "180 adim sonra vertex'ler gercekten hareket etti (dustu)");

    const float finalVolume = block.ComputeVolume();
    const float ratio = finalVolume / restVolume;
    printf("  dustukten/sikistiktan sonra hacim=%.4f m^3 (oran=%.2f)\n", finalVolume, ratio);
    // Salt yuzey agi (hacim kisiti olmadan) yere carpinca hacmi sifira
    // yakin cokertir -- gercek hacim korumasi bu oranin makul bir aralikta
    // kalmasini saglamali (tamamen sabit degil, ama coku de degil).
    CHECK(ratio > 0.4f && ratio < 1.8f,
          "hacim koruma kisiti calisiyor: dusup sikistikten sonra hacim buyuk olcude korundu");

    printf("[SoftBodyBlock] %s (%d hata)\n", g_failCount == 0 ? "GECTI" : "BASARISIZ", g_failCount);
    return g_failCount == 0 ? 0 : 1;
}
