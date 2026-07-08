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
| `src/vehicle` | Araç fiziği, ağır vasıta genişletmeleri |

Detaylı plan ve faz sıralaması: [docs/AlazForge_ClaudeCode_Brief.md](docs/AlazForge_ClaudeCode_Brief.md)

## Build

```
git clone --recursive <repo-url>
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
build/tests/alazforge_test_falling_cube
```

## Credits

- [Jolt Physics](https://github.com/jrouwe/JoltPhysics) — MIT License,
  Copyright (c) Jorrit Rouwe. Tam lisans metni: `external/JoltPhysics/LICENSE`
