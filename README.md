# AlazForge

AlazEngine için bağımsız fizik motoru katmanı. Jolt Physics üzerine inşa
edilir, kendi mimarisiyle genişletilir.

**Kapalı kaynak — tüm hakları saklıdır.** Bkz. [COPYING](COPYING).

## Kütüphaneler

Derleme çıktısı 16 ayrı paylaşımlı kütüphanedir. `weapons`/`buoyancy`/
`destructible`/`ragdoll`/`context`/`sportsball`/`character`/`rope`/`debugdraw`/
`zones`/`cloth`/`aero`/`audio`/`lod` yalnızca `physics`'e bağımlıdır, birbirlerine değil; `physics` da yalnızca
`jolt`'a bağımlıdır:

```
jolt.dll  <-  physics.dll  <-  weapons.dll
                            <-  buoyancy.dll
                            <-  destructible.dll
                            <-  ragdoll.dll
                            <-  context.dll
                            <-  sportsball.dll
                            <-  character.dll
                            <-  rope.dll
                            <-  debugdraw.dll
                            <-  zones.dll
                            <-  cloth.dll
                            <-  aero.dll
                            <-  audio.dll
                            <-  lod.dll
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
| **character** | `character.dll` + `character.lib` | `libcharacter.so` | Oyuncu/NPC karakter kontrolcüsü (`JPH::CharacterVirtual` üzerine): yürüme/koşma/zıplama, merdiven, eğim, hareketli platform |
| **rope** | `rope.dll` + `rope.lib` | `librope.so` | Segment zinciri tabanlı halat/kablo fiziği (vinç, çekme halatı) — uçlar gövdelere sabitlenebilir |
| **debugdraw** | `debugdraw.dll` + `debugdraw.lib` | `libdebugdraw.so` | Jolt DebugRenderer → AlazEngine çizim callback köprüsü (shape/constraint görselleştirme) |
| **zones** | `zones.dll` + `zones.lib` | `libzones.so` | Sürtünme bölgeleri: buz/çamur/yağ zemin koşulları (gövde friction'ı bölgeye göre değişir) |
| **cloth** | `cloth.dll` + `cloth.lib` | `libcloth.so` | Jolt SoftBody kumaş/bayrak simülasyonu (rüzgarda dalgalanır) |
| **aero** | `aero.dll` + `aero.lib` | `libaero.so` | Paraşüt + planör/wingsuit aerodinamiği (taşıma + sürükleme) |
| **audio** | `audio.dll` + `audio.lib` | `libaudio.so` | `JPH::ContactListener` tabanlı çarpışma-olayı köprüsü: şiddet + malzeme etiketli olay kuyruğu, ses tetikleme için |
| **lod** | `lod.dll` + `lod.lib` | `liblod.so` | Mesafeye göre genel amaçlı gövde uyutma/uyandırma (LOD) — uzak dinamik gövdeleri histerezisli zorla uyutur |

> `.lib` dosyaları Windows'ta consumer'ın (AlazEngine) derleme zamanında DLL'e
> bağlanması için kullanılan import kütüphaneleridir — gerçek kod içermezler,
> yükleme zamanında asıl `.dll` gerekir. Tüm kütüphaneler (Jolt'un `jolt.dll`
> olarak SHARED derlenmesi dahil) gerçek submodule checkout'uyla lokal olarak
> derlenip 24/24 testle doğrulandı.

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
| `src/character` | character | Karakter kontrolcüsü (`CharacterVirtual`): yürüme/zıplama/merdiven/eğim |
| `src/rope` | rope | Halat/kablo fiziği (kapsül segment zinciri + top eklemler) |
| `src/debugdraw` | debugdraw | Debug görselleştirme köprüsü (çizgi/üçgen callback'leri) |
| `src/wind` | physics | Küresel rüzgar sistemi (deterministik türbülans — balistik/top/halat/kumaş besler) |
| `src/zones` | zones | Sürtünme bölgeleri (buz/çamur — girince friction değişir, çıkınca döner) |
| `src/cloth` | cloth | Kumaş/bayrak (Jolt SoftBody grid + rüzgar sürüklemesi) |
| `src/aero` | aero | Paraşüt/planör aerodinamiği |
| `src/audio` | audio | Çarpışma-olayı → ses tetikleme köprüsü (`ContactListener` tabanlı) |
| `src/lod` | lod | Mesafeye göre gövde uyutma/uyandırma (genel amaçlı LOD) |

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
- **Karakter kontrolcüsü** (`src/character`) — `JPH::CharacterVirtual`
  üzerine yürüme/koşma/zıplama, merdiven çıkma + zemine yapışma
  (`ExtendedUpdate`), dik yamaç engeli, hareketli platform desteği.
- **Patlama sistemi** (`src/weapons/ExplosionSystem`) — yarıçap içindeki
  gövdelere mesafeyle azalan radyal impulse; etkilenen gövde listesi döner
  (hasar hesabı çağırana bırakılır — DLL bağımsızlık kuralı korunur).
- **Halat fiziği** (`src/rope`) — kapsül segment zinciri + top eklemler;
  uçlar dünyadaki gövdelere sabitlenebilir (vinç/çekme halatı).
- **Su durumu sorgusu** (`BuoyancySystem::QueryWaterState`) — karakter gibi
  rigid-body olmayan aktörler için nokta bazlı batıklık + akıntı bilgisi.
- **Dünya kaydet/yükle** (`AlazForgeContext::Save/RestoreWorldState`) —
  Jolt `StateRecorder` ile tüm fizik dünyasının binary snapshot'ı; aynı
  durumdan aynı adımlar bit-birebir aynı sonucu verir (determinizm testi
  ile doğrulanmış — multiplayer lockstep'in ön koşulu).
- **Terrain gerçek çarpışması** (`TerrainDeformSystem::UpdateChunkCollision`)
  — deformasyon verisi `JPH::HeightFieldShape` gövdesine çevrilir; kraterler
  gerçekten çarpışılabilir (araç çukura düşer).
- **Debug görselleştirme** (`src/debugdraw`) — Jolt DebugRenderer'ı
  AlazEngine'in çizim katmanına çizgi/üçgen callback'leriyle aktaran köprü.

### Gerçekçilik katmanı

- **Rüzgar sistemi** (`src/wind`) — yön+şiddet+deterministik türbülans (RNG'siz
  sinüs toplamı); balistik, spor topu, halat, kumaş ve paraşüt aynı rüzgardan
  beslenir. Balistikte sürükleme havaya-bağıl hıza uygulanır: yan rüzgar
  mermiyi ölçülebilir şekilde sürükler.
- **Malzeme sürtünme hattı** — `MaterialProperties`'e friction/restitution;
  `ApplyToBody` ile buz kayar, kauçuk zıplar. Varsayılan sete `ice` ve
  `rubber` eklendi.
- **Patlama siperi** — merkezden hedefe görüş hattı raycast'i: duvar arkası
  korunur; falloff varsayılan olarak ters-kare benzeri.
- **Dalgalı su** — `WaterVolume` dalga parametreleri + `SurfaceHeightAt`;
  yüzen cisimler ve yüzen karakter dalgayla iner-kalkar.
- **Sürtünme bölgeleri** (`src/zones`) — buz/çamur bölgeleri; Jolt'un
  tekerlek CombineFriction'ı zemin friction'ını çarptığından araçları da
  etkiler.
- **Kumaş** (`src/cloth`) — SoftBody bayrak, rüzgarda dalgalanır.
- **Lastik tutunması** — `AxleConfig.tireGripLongitudinal/Lateral` slip
  eğrisi çarpanları (ıslak/karlı zemin, drift).
- **Çömelme + yüzme** — `CharacterController::SetCrouch` (alçak tavan
  altında kalkamaz), `SetWaterState` ile yüzme modu (yüzeye kaldırma +
  akıntı sürüklenmesi).
- **Fırlatma yörüngesi** (`src/weapons/TrajectoryPredictor`) — el bombası
  nişan çizgisi: deterministik yörünge + ilk çarpma noktası (Explosion'a
  doğrudan verilebilir).
- **Enkaz** — `DetachBrokenPieces`: kırılan destructible parçaları tek
  çağrıyla gerçek dinamik gövdelere dönüşür.
- **Paraşüt/planör** (`src/aero`) — kuadratik sürükleme (bağlantı noktası
  üstte: kendini dikleştirir) + basitleştirilmiş kanat taşıması; rüzgarla
  birleşir.

### S1 — Entegrasyon + performans

- **Ses tetikleyicileri** (`src/audio/CollisionEventSystem`) — tek
  process-genelinde `JPH::ContactListener` iki gövdenin çarpma hızını ve
  malzeme ID'lerini olay kuyruğuna yazar; eşik altı temaslar filtrelenir,
  gerçek ses çalma AlazEngine tarafında kalır (DLL bağımsızlık kuralı).
- **Araç hasar modeli** (`VehicleSystem::ApplyImpactDamage`) — çarpışma
  hızı bir eşiği geçince motor torku doğrusal olarak azalır, araç yeterince
  hasar görünce sürülemez hale gelir. Jolt kısıtı: tekerlek `WheelSettings`
  kurulumdan sonra değiştirilemediği için direksiyon/fren hasarı
  modellenemiyor — yalnızca motor torku ölçeklenir.
- **LOD** (`src/lod/LODSystem`) — referans noktasından (oyuncu) uzak
  gövdeler histerezisli olarak zorla uyutulur/uyandırılır; Jolt'un kendi
  uyku mekanizmasından bağımsız, mesafeye dayalı CPU tasarrufu.

## Test

24 test, `ctest`'e kayıtlı (`build/tests/alazforge_test_*`):

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
| `character_controller` | Yürüme hızı, dik duvarı geçememe, zıplayıp inme |
| `explosion` | Patlama falloff sırası, yarıçap dışı, statik gövde davranışı |
| `rope` | Asılı halatın sarkması, zincir bütünlüğü + su durumu sorgusu |
| `world_state` | Dünya kaydet/geri yükle + bit-birebir determinizm |
| `terrain_collision` | Kraterin gerçek çarpışılabilirliği + debug çizim köprüsü |
| `realism` | Rüzgar determinizmi, balistik rüzgar sürüklenmesi, patlama siperi, malzeme sürtünmesi |
| `realism2` | Dalgalı su yüzeyi, sürtünme bölgeleri, kumaş+rüzgar, lastik tutunması |
| `realism3` | Çömelme/alçak tavan, yüzme+akıntı, fırlatma yörüngesi, enkaz, paraşüt/planör |
| `s1_collision_damage` | Çarpışma olayı eşik/malzeme etiketleme + araç hasar modeli eşiği |
| `lod_system` | Mesafeye göre zorla uyutma/uyandırma + histerezis |

## Build

AlazForge 16 paylaşımlı kütüphane üretir (`jolt`, `physics`, `weapons`,
`buoyancy`, `destructible`, `ragdoll`, `context`, `sportsball`, `character`,
`rope`, `debugdraw`, `zones`, `cloth`, `aero`, `audio`, `lod`). Bkz.
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
