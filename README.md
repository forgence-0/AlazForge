# AlazForge

AlazEngine için bağımsız fizik motoru katmanı. Jolt Physics üzerine inşa
edilir, kendi mimarisiyle genişletilir.

**Kapalı kaynak — tüm hakları saklıdır.** Bkz. [COPYING](COPYING).

## Kütüphaneler

Derleme çıktısı 8 ayrı paylaşımlı kütüphanedir. `weapons`/`buoyancy`/
`destructible`/`ragdoll`/`context`/`sportsball` yalnızca `physics`'e
bağımlıdır, birbirlerine değil; `physics` da yalnızca `jolt`'a bağımlıdır:

```
jolt.dll  <-  physics.dll  <-  weapons.dll
                            <-  buoyancy.dll
                            <-  destructible.dll
                            <-  ragdoll.dll
                            <-  context.dll
                            <-  sportsball.dll
```

Her hedef `add_library(... SHARED ...)` ile tanımlı; Windows/MSVC'de bu,
CMake tarafından hem `.dll`'i hem de onunla eşleşen import `.lib`'ini otomatik
üretir (ekstra bir CMake ayarı gerekmez) — Linux'ta tek çıktı `.so`'dur:

| Kütüphane | Windows | Linux | İçerik |
|---|---|---|---|
| **jolt** | `jolt.dll` + `jolt.lib` | `libjolt.so` | Jolt Physics'in kendisi (submodule, MIT) — ayrı DLL olarak derlenir |
| **physics** | `physics.dll` + `physics.lib` | `libphysics.so` | Ana fizik: adapter, spatial streaming, terrain deform, balistik, malzeme db, araç fiziği + enkaz kalıcılığı |
| **weapons** | `weapons.dll` + `weapons.lib` | `libweapons.so` | Geri tepme, sekme/dağılım (spread), türet mount fiziği |
| **buoyancy** | `buoyancy.dll` + `buoyancy.lib` | `libbuoyancy.so` | Su hacimleri ve yüzerlik (`JPH::Body::ApplyBuoyancyImpulse` üzerine) |
| **destructible** | `destructible.dll` + `destructible.lib` | `libdestructible.so` | Ayrık parça grafiği tabanlı yıkılabilir yapılar (kademeli çökme) |
| **ragdoll** | `ragdoll.dll` + `ragdoll.lib` | `libragdoll.so` | Jolt'un ragdoll sistemini saran iskelet + örnek sınıfları |
| **context** | `context.dll` + `context.lib` | `libcontext.so` | `AlazForgeContext` — Jolt yaşam döngüsü + `physics` alt sistemlerini saran facade (ECS bridge placeholder) |
| **sportsball** | `sportsball.dll` + `sportsball.lib` | `libsportsball.so` | Topla oynanan sporlar (futbol, basketbol, hentbol, Amerikan futbolu vb.) için genel top fiziği: sürükleme (drag) + Magnus/spin etkisi |

> `.lib` dosyaları Windows'ta consumer'ın (AlazEngine) derleme zamanında DLL'e
> bağlanması için kullanılan import kütüphaneleridir — gerçek kod içermezler,
> yükleme zamanında asıl `.dll` gerekir. Tüm kütüphaneler (Jolt'un `jolt.dll`
> olarak SHARED derlenmesi dahil) gerçek submodule checkout'uyla lokal olarak
> derlenip 14/14 testle doğrulandı.

## Modüller

| Modül | Kütüphane | Açıklama |
|---|---|---|
| `src/core` | physics | Jolt ↔ AlazEngine Vec3/Quat/Transform adapter katmanı |
| `src/streaming` | physics | Genel `ChunkStreamSystem<T>` — spatial streaming altyapısı (LZ4) |
| `src/terrain_deform` | physics | Chunk tabanlı terrain deformasyon sistemi |
| `src/ballistics` | physics | Raycasting tabanlı mermi/balistik sistemi |
| `src/material_db` | physics | Malzeme özellikleri: sertlik, penetrasyon, kırılma |
| `src/vehicle` | physics | Araç fiziği (tekerlekli + paletli) ve enkaz kalıcılığı |
| `src/weapons` | weapons | Geri tepme, sekme/dağılım (spread), türet mount fiziği |
| `src/buoyancy` | buoyancy | Su hacimleri ve yüzerlik (`JPH::Body::ApplyBuoyancyImpulse` üzerine) |
| `src/destructible` | destructible | Ayrık parça grafiği tabanlı yıkılabilir yapılar (kademeli çökme) |
| `src/ragdoll` | ragdoll | Jolt'un ragdoll sistemini saran iskelet + örnek sınıfları |
| `src/context` | context | `AlazForgeContext` — Jolt yaşam döngüsü + tüm alt sistemleri saran facade (ECS bridge placeholder) |
| `src/sportsball` | sportsball | Topla oynanan sporlar için genel top fiziği (sürükleme + Magnus/spin) |

Detaylı plan ve faz sıralaması: [docs/AlazForge_ClaudeCode_Brief.md](docs/AlazForge_ClaudeCode_Brief.md)

## Durum

### Temel fazlar (1-6)

Altı fazın tamamı çalışır durumda ve testleri geçiyor:

1. **Temel kurulum** — Jolt entegrasyonu, adapter katmanı, düşen küp testi
2. **Spatial streaming** — `ChunkStreamSystem<T>`, sparse LZ4 chunk depolama
3. **Terrain deformasyon** — grid tabanlı çökme (64m chunk / 25cm hücre)
4. **Balistik + malzeme DB** — mermi düşüşü, penetrasyon, sekme
5. **Araç fiziği** — çok akslı tekerlekli + paletli (skid-steer) ağır vasıta
6. **Enkaz kalıcılığı** — aynı streaming altyapısıyla hasarlı araç saklama

### Faz A — Altyapı

- Proje, header-only `INTERFACE` kütüphanesinden 8 ayrı paylaşımlı kütüphaneye
  dönüştürüldü — `jolt`, `physics` ve 6 yardımcı DLL (`weapons`, `buoyancy`,
  `destructible`, `ragdoll`, `context`, `sportsball`); bkz.
  [Kütüphaneler](#kütüphaneler). Her biri kendi `GenerateExportHeader`
  makrosuyla (`PHYSICS_API`, `WEAPONS_API`, `BUOYANCY_API`,
  `DESTRUCTIBLE_API`, `RAGDOLL_API`, `CONTEXT_API`, `SPORTSBALL_API`)
  public sınıflarını dışa açıyor.
- `ChunkStreamSystem` destructor'ındaki exception-safety hatası (disk
  hatasında olası `std::terminate()`) düzeltildi.
- `.clang-format` / `.clang-tidy` eklendi, tüm kod tabanı reformat edildi.
- GitHub Actions CI (`.github/workflows/ci.yml`): build+test, format kontrolü
  (bloklayıcı) ve clang-tidy (bilgilendirici) job'ları.

### Faz B — Fizik motoru genişletmesi

- **Silah fiziği** (`src/weapons`) — momentum korunumlu geri tepme impulse'u,
  motorlu `HingeConstraint` çiftiyle türet mount (yaw/pitch), deterministik
  sekme/dağılım (spread) modeli (RNG'siz — jitter çağırana bırakılır).
- **Yüzerlik** (`src/buoyancy`) — su hacimleri + Jolt'un
  `Body::ApplyBuoyancyImpulse` fonksiyonu üzerine ince bir kayıt/sorgu katmanı.
- **Yıkılabilir yapılar** (`src/destructible`) — ayrık parça grafiği tabanlı,
  `MaterialProperties::brittleness`'e göre kademeli çöken yapılar; terrain
  deform ile aynı sparse-chunk kalıcılık altyapısını kullanır.
- **Ragdoll** (`src/ragdoll`) — Jolt'un hazır ragdoll sistemini saran,
  ölüm anında son poz/hızla aktive edilen tek seferlik fizik örneği.
- **AlazForgeContext** (`src/context`) — Jolt yaşam döngüsünü (Factory,
  job sistemi, `PhysicsSystem::Init`) ve tüm alt sistemleri saran facade;
  gelecekteki AlazEngine ECS entegrasyonu için bilinçli olarak minimal
  tutulan bir başlangıç noktası.
- **Spor topu fiziği** (`src/sportsball`) — futbol, basketbol, hentbol,
  Amerikan futbolu gibi topla oynanan sporlar için genel top fiziği:
  aerodinamik sürükleme (drag) + Magnus etkisi (spin'e bağlı kaldırma —
  frikik/faul atışı eğrisi, backspin şut, spiral pas). Sporlara özel hazır
  preset yok — kütle/yarıçap/sürtünme/sekme/drag gibi tüm parametreler
  `BallConfig` ile oyunu yapan tarafından belirlenir. Amerikan futbolu topu
  gibi küresel olmayan toplar `elongation` alanıyla (prolate spheroid,
  `JPH::ScaledShape` üzerine) desteklenir.

## Test

14 test, `ctest`'e kayıtlı (`build/tests/alazforge_test_*`):

| Test | Kapsam |
|---|---|
| `falling_cube` | Temel rigid body — yerçekimiyle düşen küp |
| `chunk_stream` | `ChunkStreamSystem<T>` streaming + sparse LZ4 kalıcılık |
| `terrain_deform` | Grid tabanlı terrain çökmesi |
| `ballistics` | Mermi düşüşü, penetrasyon, saplanma, sekme |
| `vehicle` | Tekerlekli + paletli araç fiziği |
| `wreck_persistence` | Araç enkazı kalıcılığı |
| `recoil_impulse` | Geri tepme momentum korunumu |
| `spread_accumulator` | Sekme/dağılım ısınma-soğuma + determinizm |
| `turret_mount` | Motorlu türet yaw/pitch + limit kenetlenmesi |
| `buoyancy` | Yüksek/düşük buoyancy'nin yüzme/batma farkı |
| `destructible_structure` | Kademeli çökme (brittleness eşiği) |
| `ragdoll` | İki kemikli ragdoll zincirinin tutarlılığı |
| `alazforge_context` | Jolt yaşam döngüsü + aktif gövde snapshot'ı |
| `sportsball` | Top fiziği: Magnus etkisiyle yanal sapma, sürüklemeyle yavaşlama, elips top sanity-check |

## Build

AlazForge 8 paylaşımlı kütüphane üretir (`jolt`, `physics`, `weapons`,
`buoyancy`, `destructible`, `ragdoll`, `context`, `sportsball`). Bkz.
[Kütüphaneler](#kütüphaneler).

```
git clone --recursive <repo-url>
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

Windows'ta her hedef için `build/*.dll` + `build/*.lib` (import kütüphanesi),
Linux'ta `build/lib*.so` üretilir — bkz. [Kütüphaneler](#kütüphaneler) için
tam liste.

CI, her push/PR'da submodule'ları taze checkout edip build+test+format kontrolü
yapar (bkz. `.github/workflows/ci.yml`). Kod stili `.clang-format` ile,
statik analiz `.clang-tidy` ile denetlenir.

### Derlenen kütüphaneleri indirme

İki yol var:

1. **Her push'ta (geçici)**: CI, derlenen `.so` dosyalarını (testler geçmese
   bile — `build-and-test` job'unun "Upload built libraries" adımı) GitHub
   Actions'ın **Actions** sekmesinde ilgili run'ın altında
   `alazforge-libraries-<commit-sha>` adıyla indirilebilir bir `.zip` olarak
   yayınlar. Varsayılan olarak 90 gün sonra silinir.

2. **Versiyonlu sürümlerde (kalıcı)**: bir `v*` tag'i push edildiğinde
   (örn. `git tag v0.2.0 && git push origin v0.2.0`), release workflow'u
   (`.github/workflows/release.yml`) projeyi derleyip testleri koşar ve tüm
   `.so` dosyalarını (jolt dahil) **Releases** sekmesinde kalıcı bir GitHub
   Release olarak yayınlar — bu dosyalar silinmez.

## Credits

- [Jolt Physics](https://github.com/jrouwe/JoltPhysics) — MIT License,
  Copyright (c) Jorrit Rouwe. Tam lisans metni: `external/JoltPhysics/LICENSE`
- [LZ4](https://github.com/lz4/lz4) — BSD 2-Clause License,
  Copyright (c) Yann Collet. Tam lisans metni: `external/lz4/lib/LICENSE`
