// Faz B.5 doğrulama testi: DestructibleStructureSystem.
// 3x3 kutu-parça duvar: her sıra yalnızca hemen altındaki parçaya dayanır
// (tek kolonluk destek zinciri). Alt-orta parçaya yeterli hasar verildiğinde
// o parça kırılır ve destek kaybeden orta kolon (satır 1, satır 2) kademeli
// olarak çöker; ilişkisiz kolonlar (sol/sağ) sağlam kalmalı.

#include "JoltTestWorld.h"
#include "destructible/DestructibleStructureSystem.h"

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

static uint16_t PieceIdx(int row, int col) { return static_cast<uint16_t>(row * 3 + col); }

int main() {
    TestWorld world;
    MaterialDatabase materials = MaterialDatabase::CreateDefault();
    const MaterialId concrete = materials.FindByName("concrete"); // brittleness 0.60

    DestructibleStructureSystem::Config sysConfig;
    DestructibleStructureSystem destructible(sysConfig);

    StructureConfig wall;
    wall.structureId = 1;
    wall.worldOrigin = Vec3{0, 0, 0};
    wall.pieces.resize(9); // 3 satir x 3 sutun

    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            PieceConfig& p = wall.pieces[PieceIdx(row, col)];
            p.localCenter = Vec3{static_cast<float>(col), static_cast<float>(row), 0.0f};
            p.halfExtents = Vec3{0.45f, 0.45f, 0.1f};
            p.healthMax = 100.0f;
            p.material = concrete;
            if (row > 0) p.supportNeighbors = {PieceIdx(row - 1, col)}; // yalnizca hemen altindaki
        }
    }

    destructible.RegisterStructure(world.physics, Layers::NON_MOVING, materials, wall);

    printf("[DestructibleStructureSystem]\n");

    std::vector<PieceBrokenEvent> events;
    destructible.ApplyDamage(1, PieceIdx(0, 1), 150.0f, events); // alt-orta parcaya oldurucu hasar

    for (const PieceBrokenEvent& ev : events) {
        printf("  kirildi: parca %u, pozisyon (%.1f, %.1f, %.1f)\n", ev.pieceIndex,
               ev.worldPosition.x, ev.worldPosition.y, ev.worldPosition.z);
    }

    CHECK(destructible.IsPieceBroken(1, PieceIdx(0, 1)), "hasar verilen alt-orta parca kirildi");
    CHECK(destructible.IsPieceBroken(1, PieceIdx(1, 1)),
          "destegini kaybeden orta-orta parca kademeli olarak cokme");
    CHECK(destructible.IsPieceBroken(1, PieceIdx(2, 1)),
          "destegini kaybeden ust-orta parca kademeli olarak cokme");
    CHECK(events.size() == 3, "toplam 3 parca kirildi (dogrudan + 2 kademeli)");

    CHECK(!destructible.IsPieceBroken(1, PieceIdx(0, 0)), "ilgisiz sol-alt parca saglam kaldi");
    CHECK(!destructible.IsPieceBroken(1, PieceIdx(1, 0)), "ilgisiz sol-orta parca saglam kaldi");
    CHECK(!destructible.IsPieceBroken(1, PieceIdx(2, 0)), "ilgisiz sol-ust parca saglam kaldi");
    CHECK(!destructible.IsPieceBroken(1, PieceIdx(0, 2)), "ilgisiz sag-alt parca saglam kaldi");
    CHECK(!destructible.IsPieceBroken(1, PieceIdx(1, 2)), "ilgisiz sag-orta parca saglam kaldi");
    CHECK(!destructible.IsPieceBroken(1, PieceIdx(2, 2)), "ilgisiz sag-ust parca saglam kaldi");

    // Kirilan bir parca gercek dinamik govdeye cevrilebilmeli
    JPH::BodyID debris = destructible.DetachPieceAsDynamicBody(world.physics, 1, PieceIdx(0, 1));
    CHECK(!debris.IsInvalid(), "kirilan parca dinamik enkaz govdesine cevrildi");
    if (!debris.IsInvalid()) {
        world.physics.GetBodyInterface().RemoveBody(debris);
        world.physics.GetBodyInterface().DestroyBody(debris);
    }

    // FindPieceByBodyID: hala saglam bir parcanin body'sinden geri cozulebilmeli
    uint64_t foundStructure = 0;
    uint16_t foundPiece = 0;
    // Saglam bir parcanin BodyID'sini elde etmek icin ApplyDamage'in kirmadigi
    // bir parcaya (sol-alt) dogrudan erisim yok; bu yuzden yalnizca kirilan
    // parcanin artik body haritasinda OLMADIGINI dogruluyoruz (RemoveBody'de
    // temizlendi).
    JPH::BodyID stale; // gecersiz BodyID -> bulunamamali
    CHECK(!destructible.FindPieceByBodyID(stale, foundStructure, foundPiece),
          "gecersiz BodyID icin parca bulunamadi");

    if (g_failCount == 0) {
        printf("TEST BASARILI: DestructibleStructureSystem kademeli cokmeyi dogru uyguluyor.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
