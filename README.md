# AlazForge

AlazEngine için bağımsız fizik motoru katmanı. Jolt Physics üzerine inşa
edilir, kendi mimarisiyle genişletilir.

**Kapalı kaynak — tüm hakları saklıdır.** Bkz. [COPYING](COPYING).

## Kütüphaneler

Derleme çıktısı iki ayrı paylaşımlı kütüphanedir — `physics_ext`, `physics`'e
bağımlıdır, tersi değil:

| Kütüphane | Dosya | İçerik |
|---|---|---|
| **physics** | `physics.dll` / `libphysics.so` | Ana fizik: adapter, spatial streaming, terrain deform, balistik, malzeme db, araç fiziği + enkaz kalıcılığı |
| **physics_ext** | `physics_ext.dll` / `libphysics_ext.so` | Yardımcı/genişletme: silah fiziği, yüzerlik, yıkılabilir yapılar, ragdoll, ECS bridge facade'ı |

## Modüller

| Modül | Kütüphane | Açıklama |
|---|---|---|
| `src/core` | physics | Jolt ↔ AlazEngine Vec3/Quat/Transform adapter katmanı |
| `src/streaming` | physics | Genel `ChunkStreamSystem<T>` — spatial streaming altyapısı (LZ4) |
| `src/terrain_deform` | physics | Chunk tabanlı terrain deformasyon sistemi |
| `src/ballistics` | physics | Raycasting tabanlı mermi/balistik sistemi |
| `src/material_db` | physics | Malzeme özellikleri: sertlik, penetrasyon, kırılma |
| `src/vehicle` | physics | Araç fiziği (tekerlekli + paletli) ve enkaz kalıcılığı |
| `src/weapons` | physics_ext | Geri tepme, sekme/dağılım (spread), türet mount fiziği |
| `src/buoyancy` | physics_ext | Su hacimleri ve yüzerlik (`JPH::Body::ApplyBuoyancyImpulse` üzerine) |
| `src/destructible` | physics_ext | Ayrık parça grafiği tabanlı yıkılabilir yapılar (kademeli çökme) |
| `src/ragdoll` | physics_ext | Jolt'un ragdoll sistemini saran iskelet + örnek sınıfları |
| `src/context` | physics_ext | `AlazForgeContext` — Jolt yaşam döngüsü + tüm alt sistemleri saran facade (ECS bridge placeholder) |

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

- Proje, header-only `INTERFACE` kütüphanesinden iki gerçek paylaşımlı
  kütüphaneye dönüştürüldü: **physics** (`PHYSICS_API`, ana fizik) ve
  **physics_ext** (`PHYSICS_EXT_API`, yardımcı/genişletme — bkz.
  [Kütüphaneler](#kütüphaneler)); her ikisi de `GenerateExportHeader` ile
  üretilen makrolarla public sınıflarını dışa açıyor.
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

> Bu geliştirme ortamında Jolt submodule checkout edilmediği için `TurretMount`,
> `BuoyancySystem` ve ragdoll modülleri gerçek Jolt header'larına karşı derlenip
> doğrulanamadı — ilgili dosyalarda üst bilgi yorumlarıyla işaretlendi.
> Submodule mevcut olduğunda (CI'da) ilk derlemede teyit/düzeltme gerekebilir.

## Test

13 test, `ctest`'e kayıtlı (`build/tests/alazforge_test_*`):

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

## Build

AlazForge iki paylaşımlı kütüphane üretir: `physics` (ana fizik) ve
`physics_ext` (yardımcı/genişletme, `physics`'e bağımlı). Bkz.
[Kütüphaneler](#kütüphaneler).

```
git clone --recursive <repo-url>
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

Windows'ta `build/physics.dll` ve `build/physics_ext.dll`; Linux'ta
`build/libphysics.so` ve `build/libphysics_ext.so` üretilir.

CI, her push/PR'da submodule'ları taze checkout edip build+test+format kontrolü
yapar (bkz. `.github/workflows/ci.yml`). Kod stili `.clang-format` ile,
statik analiz `.clang-tidy` ile denetlenir.

## Credits

- [Jolt Physics](https://github.com/jrouwe/JoltPhysics) — MIT License,
  Copyright (c) Jorrit Rouwe. Tam lisans metni: `external/JoltPhysics/LICENSE`
- [LZ4](https://github.com/lz4/lz4) — BSD 2-Clause License,
  Copyright (c) Yann Collet. Tam lisans metni: `external/lz4/lib/LICENSE`
