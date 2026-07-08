# AlazForge — Fizik Motoru Projesi Başlangıç Planı

## Proje Tanımı

AlazForge, AlazEngine (ana oyun motoru) için ayrı bir repo olarak geliştirilen,
bağımsız bir fizik motoru katmanıdır. Jolt Physics üzerine inşa edilir, kendi
mimarisiyle genişletilir. Kapalı kaynak, tüm haklar saklıdır.

## Lisans Durumu

- **AlazForge'un kendi kodu:** Proprietary / All Rights Reserved. Repo **private**
  tutulacak. `LICENSE` dosyası yerine `COPYING` dosyasına şu not eklenecek:
  ```
  Copyright (c) 2026 [isim/stüdyo]
  All Rights Reserved.
  Bu depodaki kaynak kodun kopyalanması, değiştirilmesi veya dağıtılması
  izin alınmadan yasaktır.
  ```
- **Jolt Physics (MIT):** Kaynağı gizli kalabilir, ama README/credits içinde
  Jolt'un MIT lisans metnine atıf yapılmalı.

## Proje Klasör Yapısı

```
AlazForge/
├── external/
│   └── JoltPhysics/              (git submodule, MIT lisans)
├── src/
│   ├── core/                     (Jolt <-> AlazEngine Vec3/Quat/Transform adapter)
│   ├── vehicle/                  (araç fiziği, ağır vasıta genişletmeleri)
│   ├── ballistics/                (mermi/balistik sistemi, malzeme penetrasyonu)
│   ├── terrain_deform/          (chunk tabanlı deformasyon sistemi)
│   ├── streaming/                 (genel ChunkStreamSystem — spatial streaming altyapısı)
│   └── material_db/               (malzeme özellikleri: sertlik, penetrasyon, kırılma)
├── tests/
├── COPYING
├── README.md
└── CMakeLists.txt
```

## Faz Sıralaması (öncelik sırasıyla)

### Faz 1 — Temel Kurulum
- Jolt Physics'i git submodule olarak ekle
- CMake yapılandırması (Jolt'u build sistemine bağla)
- `core/` içinde AlazEngine'in kendi Vec3/Quat/Transform tipleri ile Jolt'un
  tipleri arasında adapter katmanı yaz (iki yönlü dönüşüm fonksiyonları)
- Basit bir test sahnesi: yerçekimi altında düşen bir küp (rigid body doğrulama)

### Faz 2 — Genel Spatial Streaming Altyapısı
- `streaming/ChunkStreamSystem<T>` template sınıfını yaz (bkz. aşağıdaki interface)
- Chunk koordinat hesaplama (`GetChunkCoord`), yükleme/boşaltma mantığı
- Disk I/O: sparse storage (sadece dokunulmuş chunk'lar dosya olarak var olur)
- Sıkıştırma (zlib veya LZ4) entegrasyonu

### Faz 3 — Terrain Deformasyon Modülü
- `terrain_deform/` — ChunkStreamSystem üzerine kurulu, grid tabanlı deformasyon
- Chunk boyutu ve çözünürlük parametrik olmalı (varsayılan: 64m chunk, 25cm grid)
- Tekerlek/temas noktası → hücre güncelleme mantığı
- Aynı hücreye tekrar dokunma = değer güncellenir, yeni hücre eklenmez (boyut şişmesin)

### Faz 4 — Balistik Modülü
- `ballistics/` — raycasting tabanlı mermi sistemi
- `material_db/` ile entegre: malzemeye göre penetrasyon/ricochet/decal davranışı
- Mermi düşüşü (bullet drop), menzil kaybı hesaplama

### Faz 5 — Araç Fiziği (Vehicle)
- Jolt'un `Physics/Vehicle` sistemi temel alınır
- Ağır vasıta için çok akslı süspansiyon/fren/traksiyon parametreleri genişletilir

### Faz 6 — Araç Enkaz/Hasar Kalıcılığı
- Terrain deform ile aynı ChunkStreamSystem altyapısını kullanarak, dünyada
  kalan hasarlı/terk edilmiş araçların pozisyon ve hasar durumunu sakla

## Genel Streaming Interface (referans taslak)

```cpp
struct ChunkCoord {
    int x, z;
    bool operator==(const ChunkCoord&) const;
};

struct ChunkGrid {
    int mapSize;
    int chunkSize;
    int chunksPerSide;

    ChunkGrid(int mapSize, int chunkSize);
    ChunkCoord GetChunkCoord(float worldX, float worldZ) const;
    std::vector<ChunkCoord> GetChunksInRadius(float worldX, float worldZ, float radius) const;
};

template<typename ChunkDataType>
class ChunkStreamSystem {
public:
    ChunkStreamSystem(ChunkGrid grid, float loadRadius, const std::string& savePath);

    void OnPlayerMove(float worldX, float worldZ);
    ChunkDataType& GetChunkAt(float worldX, float worldZ);
    void MarkDirty(const ChunkCoord& coord);
    void FlushDirtyChunks(); // diske yaz

private:
    ChunkGrid grid;
    float loadRadius;
    std::string savePath;
    std::unordered_map<ChunkCoord, ChunkDataType> loadedChunks;
    std::unordered_set<ChunkCoord> dirtyChunks;

    void LoadChunk(const ChunkCoord& coord);
    void UnloadChunk(const ChunkCoord& coord); // önce diske yaz, sonra RAM'den at
    bool ChunkFileExists(const ChunkCoord& coord) const;
};

// Kullanım örneği:
struct DeformationChunkData {
    std::vector<uint8_t> heightMap; // 256x256, 25cm çözünürlük, 64m chunk için
};
```

## Notlar / Kısıtlar

- Hedef donanım: AMD Ryzen 5 8400F, 32GB RAM, AMD RX570 (CUDA yok, DirectML/CPU
  fallback düşünülmeli — GPU-accelerated fizik özellikleri bu donanımda sınırlı)
- Sparse storage zorunlu: haritanın dokunulmamış kısımları asla dosya/RAM
  kaplamamalı
- Her chunk sistemi (terrain, ballistics, wreck) aynı `ChunkStreamSystem<T>`
  altyapısını kullanmalı — kod tekrarından kaçın
- İlerleyen fazlarda AlazEngine ana projesiyle entegrasyon için AlazForge'un
  public API'si stabil ve minimal tutulmalı (ECS bridge kolay olsun diye)

## İlk Talimat (Claude Code'a verilecek)

> AlazForge adında yeni bir C++ projesi kur. Yukarıdaki klasör yapısını oluştur,
> Jolt Physics'i submodule olarak ekle, CMakeLists.txt'yi Jolt'u build edecek
> şekilde yapılandır. Faz 1'i tamamla: temel adapter katmanı ve basit bir rigid
> body test sahnesi (yerçekimi altında düşen küp) çalışır halde olsun.
