// Faz B.2 doğrulama testi: SpreadAccumulator.
// Isınma monotonluğu, soğuma davranışı ve determinizm (RNG'siz, aynı çağrı
// dizisi aynı sonucu vermeli) doğrulanır. Jolt dünyasına ihtiyaç yok.

#include "weapons/SpreadAccumulator.h"

#include <cstdio>

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
    printf("[Isinma monotonlugu + kenetleme]\n");
    {
        SpreadAccumulator acc;
        float prev = acc.CurrentHalfAngleRad();
        bool monotonic = true;
        for (int i = 0; i < 50; ++i) {
            acc.RegisterShot();
            const float cur = acc.CurrentHalfAngleRad();
            if (cur < prev) monotonic = false;
            prev = cur;
        }
        CHECK(monotonic, "ates ettikce yayilma acisi hic azalmadi");

        SpreadConfig cfg;
        CHECK(acc.CurrentHeat() >= cfg.maxHeat - 1e-6f, "isi maxHeat'te kenetlendi");
        CHECK(std::fabs(acc.CurrentHalfAngleRad() - cfg.maxHalfAngleRad) < 1e-5f,
              "tam isinmis konide maxHalfAngleRad'a ulasti");
    }

    printf("[Soguma]\n");
    {
        SpreadAccumulator acc;
        for (int i = 0; i < 20; ++i)
            acc.RegisterShot();
        const float hot = acc.CurrentHalfAngleRad();

        acc.Update(10.0f); // uzun sure ates yok

        const float cooled = acc.CurrentHalfAngleRad();
        SpreadConfig cfg;
        CHECK(cooled < hot, "zamanla sogudu");
        CHECK(std::fabs(cooled - cfg.baseHalfAngleRad) < 1e-4f,
              "yeterince uzun sogumada taban aciya dondu");
    }

    printf("[Determinizm]\n");
    {
        SpreadAccumulator a, b;
        bool identical = true;
        for (int i = 0; i < 30; ++i) {
            a.RegisterShot();
            b.RegisterShot();
            a.Update(0.05f);
            b.Update(0.05f);
            if (a.CurrentHalfAngleRad() != b.CurrentHalfAngleRad()) identical = false;
        }
        CHECK(identical,
              "iki bagimsiz accumulator ayni cagri dizisinde bit-birebir ayni sonucu verdi");
    }

    printf("[ApplyConeJitter saflik]\n");
    {
        const Vec3 dir{0, 0, 1};
        const Vec3 j1 = ApplyConeJitter(dir, 0.05f, 0.3f, 0.7f);
        const Vec3 j2 = ApplyConeJitter(dir, 0.05f, 0.3f, 0.7f);
        CHECK(j1.x == j2.x && j1.y == j2.y && j1.z == j2.z,
              "ayni girdiyle ApplyConeJitter ayni sonucu verdi (RNG icermiyor)");
        const float len = std::sqrt(j1.x * j1.x + j1.y * j1.y + j1.z * j1.z);
        CHECK(std::fabs(len - 1.0f) < 1e-4f, "jitter uygulanmis yon birim vektor");
    }

    if (g_failCount == 0) {
        printf("TEST BASARILI: SpreadAccumulator deterministik ve tutarli calisiyor.\n");
        return 0;
    }
    printf("TEST BASARISIZ: %d kontrol gecmedi!\n", g_failCount);
    return 1;
}
