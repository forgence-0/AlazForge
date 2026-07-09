# AlazForge

AlazEngine için bağımsız fizik motoru katmanı. Jolt Physics üzerine inşa
edilir, kendi mimarisiyle genişletilir.

**Kapalı kaynak — tüm hakları saklıdır.** Bkz. [COPYING](COPYING).

## Modüller

| Modül | Açıklama |
|---|---|
| `src/core` | Jolt ↔ AlazEngine Vec3/Quat/Transform adapter katmanı |
| `src/streaming` | Genel `ChunkStreamSystem<T>` — spatial streaming altyapısı (LZ4) |
| `src/terrain_deform` | Chunk tabanlı terrain deformasyon sistemi |
| `src/ballistics` | Raycasting tabanlı mermi/balistik sistemi |
| `src/material_db` | Malzeme özellikleri: sertlik, penetrasyon, kırılma |
| `src/vehicle` | Araç fiziği (tekerlekli + paletli) ve enkaz kalıcılığı |
| `src/weapons` | Geri tepme, sekme/dağılım (spread), türet mount fiziği |
| `src/buoyancy` | Su hacimleri ve yüzerlik (`JPH::Body::ApplyBuoyancyImpulse` üzerine) |
| `src/destructible` | Ayrık parça grafiği tabanlı yıkılabilir yapılar (kademeli çökme) |
| `src/ragdoll` | Jolt'un ragdoll sistemini saran iskelet + örnek sınıfları |
| `src/context` | `AlazForgeContext` — Jolt yaşam döngüsü + tüm alt sistemleri saran facade (ECS bridge placeholder) |

Detaylı plan ve faz sıralaması: [docs/AlazForge_ClaudeCode_Brief.md](docs/AlazForge_ClaudeCode_Brief.md)

## Durum

Temel altı faz + genişletme (Faz B) çalışır durumda ve testleri geçiyor (`ctest`):

1. **Temel kurulum** — Jolt entegrasyonu, adapter katmanı, düşen küp testi
2. **Spatial streaming** — `ChunkStreamSystem<T>`, sparse LZ4 chunk depolama
3. **Terrain deformasyon** — grid tabanlı çökme (64m chunk / 25cm hücre)
4. **Balistik + malzeme DB** — mermi düşüşü, penetrasyon, sekme
5. **Araç fiziği** — çok akslı tekerlekli + paletli (skid-steer) ağır vasıta
6. **Enkaz kalıcılığı** — aynı streaming altyapısıyla hasarlı araç saklama
7. **Silah fiziği** — geri tepme impulse'u, motorlu türet mount, deterministik sekme/dağılım
8. **Yüzerlik** — su hacimleri, Jolt buoyancy impulse entegrasyonu
9. **Yıkılabilir yapılar** — malzeme kırılganlığına göre kademeli çökme
10. **Ragdoll** — ölüm anında son poz/hızla aktive edilen fizik iskeleti
11. **AlazForgeContext** — tüm alt sistemleri saran, gelecekteki AlazEngine ECS entegrasyonu için facade

> Faz B'deki `TurretMount`, `BuoyancySystem` ve ragdoll modülleri, bu geliştirme
> ortamında Jolt submodule checkout edilmediği için gerçek Jolt header'larına
> karşı derlenip doğrulanamadı — ilgili dosyalarda üst bilgi yorumlarıyla
> işaretlendi. Submodule mevcut olduğunda (CI'da) ilk derlemede teyit/düzeltme
> gerekebilir.

## Build

AlazForge, `alazforge` adında paylaşımlı bir kütüphane (`libalazforge.so` /
`AlazForge.dll`) olarak derlenir.

```
git clone --recursive <repo-url>
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
```

CI, her push/PR'da submodule'ları taze checkout edip build+test+format kontrolü
yapar (bkz. `.github/workflows/ci.yml`). Kod stili `.clang-format` ile,
statik analiz `.clang-tidy` ile denetlenir.

## Credits

- [Jolt Physics](https://github.com/jrouwe/JoltPhysics) — MIT License,
  Copyright (c) Jorrit Rouwe. Tam lisans metni: `external/JoltPhysics/LICENSE`
- [LZ4](https://github.com/lz4/lz4) — BSD 2-Clause License,
  Copyright (c) Yann Collet. Tam lisans metni: `external/lz4/lib/LICENSE`
