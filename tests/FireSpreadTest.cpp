// S3 doğrulama testi: FireSpreadSystem.
// Tutuşturulan bir düğüm, yarıçapı içindeki yanabilir komşuya eşik süresi
// dolunca sıçrar; yarıçap dışındaki veya yanmaz düğüm tutuşmaz; yanan düğüm
// süresi dolunca söner (Burnt, bir daha tutuşmaz). Determinizm: aynı girdi
// dizisiyle iki bağımsız sistem bit-birebir aynı sonucu verir.

#include "fire/FireSpreadSystem.h"

#include <cstdio>
#include <vector>

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
    printf("[FireSpreadSystem]\n");

    FireSpreadSystem fire;

    FireNodeConfig near;
    near.position = Vec3{2, 0, 0}; // yaricap icinde
    near.spreadRadius = 5.0f;
    near.igniteThresholdSec = 1.0f;
    near.burnDurationSec = 2.0f;
    const uint32_t nearId = fire.RegisterNode(near);

    FireNodeConfig far;
    far.position = Vec3{20, 0, 0}; // yaricap disinda
    far.spreadRadius = 5.0f;
    far.igniteThresholdSec = 1.0f;
    far.burnDurationSec = 2.0f;
    const uint32_t farId = fire.RegisterNode(far);

    FireNodeConfig nonFlammable;
    nonFlammable.position = Vec3{3, 0, 0}; // yaricap icinde ama yanmaz
    nonFlammable.spreadRadius = 5.0f;
    nonFlammable.igniteThresholdSec = 1.0f;
    nonFlammable.flammable = false;
    const uint32_t stoneId = fire.RegisterNode(nonFlammable);

    FireNodeConfig source;
    source.position = Vec3{0, 0, 0};
    source.spreadRadius = 5.0f;
    source.igniteThresholdSec = 1.0f;
    source.burnDurationSec = 3.0f;
    const uint32_t sourceId = fire.RegisterNode(source);

    fire.Ignite(sourceId);
    CHECK(fire.GetState(sourceId) == FireState::Burning, "kaynak dogrudan tutusturuldu");
    CHECK(fire.BurningCount() == 1, "yalnizca 1 dugum yaniyor (baslangic)");

    // igniteThresholdSec=1.0s: 0.5s sonra henuz tutusmamis olmali.
    std::vector<uint32_t> ignited, extinguished;
    for (int i = 0; i < 30; ++i)
        fire.Update(1.0f / 60.0f, &ignited, &extinguished); // 0.5s
    CHECK(fire.GetState(nearId) == FireState::Unburnt, "esik dolmadan yakin dugum henuz tutusmadi");

    for (int i = 0; i < 40; ++i)
        fire.Update(1.0f / 60.0f, &ignited, &extinguished); // +0.667s (toplam ~1.17s)
    CHECK(fire.GetState(nearId) == FireState::Burning,
          "esik dolunca yakin dugum tutustu (yayilim)");
    CHECK(fire.GetState(farId) == FireState::Unburnt, "yaricap disindaki dugum tutusmadi");
    CHECK(fire.GetState(stoneId) == FireState::Unburnt,
          "yanmaz dugum yaricap icinde olsa da tutusmadi");

    // Kaynak burnDurationSec=3s: toplam ~1.17s'de hala yaniyor olmali.
    CHECK(fire.GetState(sourceId) == FireState::Burning, "kaynak suresi dolmadan hala yaniyor");

    // Kaynagin sonmesini bekle (toplam 3s'yi asana kadar ilerlet).
    for (int i = 0; i < 180; ++i)
        fire.Update(1.0f / 60.0f, &ignited, &extinguished);
    CHECK(fire.GetState(sourceId) == FireState::Burnt, "kaynak suresi dolunca sondu");

    fire.Ignite(sourceId);
    CHECK(fire.GetState(sourceId) == FireState::Burnt,
          "sonmus dugum tekrar Ignite() ile yeniden tutusmuyor");

    // ── Determinizm: iki bagimsiz sistem ayni girdi dizisinde ayni sonucu vermeli ──
    printf("[FireSpreadSystem - determinizm]\n");
    {
        FireSpreadSystem sysA, sysB;
        for (FireSpreadSystem* s : {&sysA, &sysB}) {
            FireNodeConfig a;
            a.position = Vec3{0, 0, 0};
            a.igniteThresholdSec = 0.5f;
            a.spreadRadius = 4.0f;
            a.burnDurationSec = 1.5f;
            const uint32_t id = s->RegisterNode(a);
            s->Ignite(id);
            FireNodeConfig b = a;
            b.position = Vec3{2, 0, 0};
            s->RegisterNode(b);
        }
        for (int i = 0; i < 200; ++i) {
            sysA.Update(1.0f / 60.0f);
            sysB.Update(1.0f / 60.0f);
        }
        CHECK(sysA.GetState(1) == sysB.GetState(1) && sysA.GetState(2) == sysB.GetState(2),
              "ayni girdi dizisiyle iki bagimsiz sistem ayni durumda (determinizm)");
    }

    if (g_failCount == 0) {
        printf(
            "TEST BASARILI: FireSpreadSystem yayilim/sonme/determinizm dogru "
            "calisiyor.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
