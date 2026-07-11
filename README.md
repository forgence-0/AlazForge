# AlazForge

AlazEngine için bağımsız fizik motoru katmanı. Jolt Physics üzerine inşa
edilir, kendi mimarisiyle genişletilir.

**Kapalı kaynak — tüm hakları saklıdır.** Bkz. [COPYING](COPYING).

## Kütüphaneler

Derleme çıktısı 16 ayrı paylaşımlı kütüphanedir. `weapons`/`buoyancy`/
`destructible`/`ragdoll`/`sportsball`/`character`/`rope`/`debugdraw`/
`zones`/`cloth`/`aero`/`audio`/`lod` yalnızca `physics`'e bağımlıdır, birbirlerine
değil; `physics` da yalnızca `jolt`'a bağımlıdır. **`context` bu kuralın
bilinçli tek istisnasıdır**: AlazEngine'in tek giriş noktası olması için
`zones`/`buoyancy`/`lod`/`audio`/`debugdraw`/`destructible`'ı da sarmalar
(aşağıda [AlazForgeContext](#alazforgecontext--entegrasyon-noktası) bölümüne
bakın):

```
jolt.dll  <-  physics.dll  <-  weapons.dll
                            <-  buoyancy.dll ────────┐
                            <-  destructible.dll ────┤
                            <-  ragdoll.dll          │
                            <-  sportsball.dll       │
                            <-  character.dll        │
                            <-  rope.dll             │
                            <-  debugdraw.dll ───────┤
                            <-  zones.dll ───────────┤
                            <-  cloth.dll            │
                            <-  aero.dll             │
                            <-  audio.dll ───────────┤
                            <-  lod.dll ─────────────┤
                                                      ▼
                                                context.dll (hepsini sarmalar)
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
| **context** | `context.dll` + `context.lib` | `libcontext.so` | `AlazForgeContext` — AlazEngine entegrasyonu için TEK giriş noktası: Jolt yaşam döngüsü + tüm world-scoped alt sistemleri (rüzgar/bölge/yüzerlik/LOD/ses-olayı/debug-çizim, isteğe bağlı terrain/destructible) tek bir `Step()` arkasında toplar |
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
> derlenip 26/26 testle doğrulandı.

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
| `src/context` | context | `AlazForgeContext` — AlazEngine entegrasyonu için tek giriş noktası facade'ı |
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
  job sistemi, `PhysicsSystem::Init`) VE tüm world-scoped alt sistemleri
  saran tek giriş noktası facade'ı; bkz.
  [AlazForgeContext — entegrasyon noktası](#alazforgecontext--entegrasyon-noktası).
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

### R4 — Derinlik geçişi (araç/balistik/yıkım/termal-sıvı)

- **Araç hava direnci** (`VehicleSystem::ApplyAeroDrag`) — gövde hızına
  karşıt kuadratik hava sürtünmesi (F = 0.5·ρ·Cd·A·v²); `dragCoefficient
  <= 0` ile devre dışı (varsayılan, geriye uyumlu).
- **Balistik spin drift** (`BulletParams::spinDriftAccel`) — yiv
  kararlılığından kaynaklanan yanal sürüklenmenin basitleştirilmiş,
  deterministik modeli (sabit yanal ivme, ateş yönüne dik sağ eksende).
  Gerçek gyroskopik "yaw of repose" fiziği modellenmez.
- **Su derinliğine bağlı direnç** (`WaterVolume::depthDragMultiplierMax`)
  — derinlik arttıkça linear/angular drag doğrusal olarak artar (basınç/
  viskozite yaklaşıklığı); yüzeyde çarpan=1, `depthSaturationM`'de
  maksimuma ulaşır. `depthDragMultiplierMax <= 1` ile devre dışı
  (varsayılan, geriye uyumlu — eski davranış değişmez).
- **Halat kopması** (`RopeSystem::maxTensionN` + `Update`) — segmentler
  arası top eklemin taşıdığı gerginlik (`JPH::PointConstraint::
  GetTotalLambdaPosition() / dt`) eşiği aşınca eklem fizikten kaldırılır,
  halat gerçekten ikiye ayrılır. `maxTensionN <= 0` ile kopmaz (varsayılan).

## AlazForgeContext — entegrasyon noktası

`AlazForgeContext`, AlazEngine'in bu repoyu kullanmaya başlaması için TEK
giriş noktasıdır — demo/prototip değil, 26 testle doğrulanmış gerçek bir
entegrasyon yüzeyi. Amaç: AlazEngine bir `AlazForgeContext` `new` edip her
frame `Step()` çağırsın; 16 ayrı DLL'i elle birbirine bağlamak zorunda
kalmasın.

**Context her zaman kurar (disk I/O yok, ekstra yapılandırma gerekmez):**
Jolt yaşam döngüsü (Factory/job sistemi/`PhysicsSystem::Init`),
`MaterialDatabase`, `BallisticsSystem`, `WindSystem`, `FrictionZoneSystem`,
`BuoyancySystem`, `LODSystem`, `CollisionEventSystem` (ses olayları —
`physics.SetContactListener` OTOMATIK bağlanır, elle `Attach()` çağırmaya
gerek yok) ve `DebugDrawBridge`. `Step(dt, collisionSteps, lodReferencePoint)`
tek çağrıda rüzgar zamanını ilerletir, sürtünme bölgelerini + yüzerliği
fiziği ilerletmeden ÖNCE, LOD'u ilerlettikten SONRA doğru sırada günceller.

**İsteğe bağlı (disk'e dizin yazan, bu yüzden yalnızca istenirse kurulan):**
`EnableTerrain(cfg)` ve `EnableDestructible(cfg)` — ilk çağrıda örneği
oluşturur, sonraki çağrılar aynı örneği döner (idempotent); `HasTerrain()`/
`HasDestructible()` ile sorgulanabilir.

**Context'e BİLİNÇLİ OLARAK dahil edilmeyenler** — varlık başına
örneklenen sistemler (tek bir "dünya" sahibi olamazlar): `VehicleSystem`,
`CharacterController`, `RopeSystem`, `RagdollInstance`, `ClothSystem`,
`AeroSystem`, `TurretMount`, `RecoilImpulse`, `SpreadAccumulator`,
`ExplosionSystem`, `TrajectoryPredictor`. Bunlar oyun tarafında varlık
başına `context.Physics()`'ten alınan `JPH::PhysicsSystem&` ile kurulur.

AlazEngine'in gerçek ECS'i bu repodan görünmediği için entity id eşlemesi
kasıtlı olarak Context'te tutulmuyor — oyun tarafı `BodyID -> entity`
tablosunu `SnapshotActiveBodies()` üzerinden kendi kurar. AlazEngine'in
ECS'i erişilebilir olduğunda Context, doğrudan component senkronizasyonu
yapan ince bir köprüye dönüştürülebilir; o zamana kadar da tek başına
çalışan gerçek bir entegrasyon noktasıdır.

### Sertleştirme — bağımsız kod incelemesinde bulunanlar

Harici bir kod incelemesinde bulunan 5 gerçek sorun, hepsi doğrulanıp
düzeltildi (yeni testlerle kanıtlandı, sadece kod yorumu değiştirilmedi):

- **Katman haritalama sınır-dışı yazma** — `BPLayerInterfaceImpl`
  `Config::movingLayer`/`nonMovingLayer`'ı 2 elemanlı sabit bir diziye
  doğrudan indeksliyordu; AlazEngine kendi katman numaralandırmasında 2/3
  gibi bir değer verseydi bellek sınırı dışına yazardı. Diziyi kaldırıp
  karşılaştırmaya geçirildi (diğer filtre sınıflarıyla tutarlı hale
  getirildi).
- **Yıkım kalıcılığı tek yönlüydü** — `BreakPiece` kırılan parçaları
  diske yazıyordu ama `RegisterStructure` bu kayıtları hiç geri okumuyordu;
  bir yeniden başlatma/reload sonrası tüm kırık yapılar "iyileşmiş"
  görünürdü. `RegisterStructure` artık her parça için ilgili chunk'ta
  kayıtlı bir kırık-parça girdisi olup olmadığını kontrol ediyor, varsa
  gövde hiç oluşturmadan parçayı kırık başlatıyor.
- **Girdi doğrulama boşlukları** — boş aks listesiyle paletli araç
  kurulumu `.front()` ile çökerdi (artık sessizce reddediliyor, `Kind()`
  `None` kalıyor); sıfır yön vektörüyle ateşleme `Normalized()`'i NaN'a
  götürürdü (artık geçersiz atış olarak hemen duruyor); yarıçap=0 ile
  terrain/destructible/patlama falloff hesapları `0/0` NaN üretiyordu
  (üçü de sıfır yarıçapı güvenli şekilde ele alacak şekilde düzeltildi).

Test sayısı 26'da sabit kaldı — yeni assertion'lar mevcut testlere
eklendi, ayrı dosya açılmadı. Hepsi gerçek regresyon senaryosu kurup
doğrulandı (örn. yıkım kalıcılığı için iki ayrı `DestructibleStructureSystem`
örneği: biri kırıp diske yazıyor, diğeri aynı `savePath`'ten yeniden
kurulup kırık durumun geri geldiğini kanıtlıyor).

## Performans — sistem bazlı ölçüm

`tests/BenchmarkMain.cpp` (`build/tests/alazforge_bench`, `ctest`'e kayıtlı
DEĞİL — CI'da ayrı, bilgilendirici bir adım olarak çalışır) her fizik alt
sistemini KENDİ gerçekçi senaryosunda, kendi `AlazForgeContext`'inde ayrı
ayrı ölçer.

> **Bu sayılar CI'nin jenerik runner CPU'sunda ölçüldü — hedef oyun
> donanımınızdaki MUTLAK FPS'i temsil etmez.** Değerleri kendi
> makinenizde `./build/tests/alazforge_bench` çalıştırarak yeniden üretin.
> Buradaki asıl değer, sistemler ARASI GÖRECELİ maliyet karşılaştırmasıdır:
> hangi sistem hangi varlık sayısında ne kadar pahalı, hangisi ucuz.

| Kütüphane | Senaryo | Varlık | Ort. süre | 60 FPS bütçesindeki payı* |
|---|---|---:|---:|---:|
| `physics` | 1200 dinamik kutu yığını (genel rijit gövde, taban çizgisi) | 1200 | 1.02 ms | %6 |
| `physics` | 20 tekerlekli araç (tam gaz, `Step` dahil) | 20 | 0.26 ms | %1.5 |
| `physics` | 500 mermi atışı (`BallisticsSystem::Fire`, toplu) | 500 | 0.64 ms (0.0013 ms/mermi) | %4 |
| `character` | 100 karakter yürüme (`Update` + `Step`) | 100 | 0.11 ms | %0.7 |
| `cloth` | 20 kumaş parçası (12×10 = 2400 vertex, `ApplyWind`+`Step`) | 2400 vertex | 0.44 ms | %2.6 |
| `rope` | 30 halat (16 segment = 480 gövde, `Step`) | 480 gövde | 2.40 ms | **%14** |
| `ragdoll` | 50 ragdoll (3 kemik = 150 gövde, `Step`) | 150 gövde | 0.36 ms | %2.2 |
| `buoyancy` | 300 yüzen kutu (dalgalı yüzey, `Step`) | 300 | 0.33 ms | %2 |
| `zones` | 500 izlenen gövde (bölge giriş/çıkış sorgusu, `Step`) | 500 | 0.32 ms | %1.9 |
| `destructible` | 900 parçalı yapı, steady-state (`Step`) | 900 parça | 0.001 ms | ~%0 |
| `destructible` | Geniş yarıçaplı hasar + kademeli çökme (BFS, toplu) | 900 parça | 0.005 ms/çağrı | ~%0 |
| `weapons` | 500 gövdeye patlama (`ExplosionSystem::Detonate`, toplu) | 500 | 0.11 ms | %0.6 |
| `lod` | 2000 uzak gövde, LOD **YOK** (`Step`) | 2000 | 1.04 ms | %6.3 |
| `lod` | 2000 uzak gövde, LOD **VAR** (`Step`) | 2000 | 0.11 ms | %0.7 |

*60 FPS'de bir kareye ayrılan bütçe 16.6 ms'dir; oran doğrusal ölçekleme
varsayımıyla hesaplanmıştır — gerçek maliyet broad-phase çift sayısı gibi
etkenlerle varlık sayısıyla doğrusal büyümeyebilir, bu yüzden kaba bir
fikir verir, kesin üst sınır değildir.

### Hepsi birlikte — kombine sahne (tek gerçek ölçüm)

Yukarıdaki satırlar İZOLE ölçümler — her sistem kendi başına, boş bir
sahnede. Gerçek bir oyun sahnesinde hepsi AYNI ANDA çalışır, bu yüzden
`BenchCombinedAll` tek bir `AlazForgeContext`'te hepsini birden kurup
TEK bir `Step()`'in gerçek maliyetini ölçer (izole sürelerin toplamı
DEĞİL — paylaşılan broad-phase/job-sistemi overhead'i bir kez sayılır):

| Sahne içeriği | Toplam gövde | Ort. süre/kare | Ort. FPS | En kötü kare** | En kötü FPS |
|---|---:|---:|---:|---:|---:|
| 20 araç + 100 karakter + 20 kumaş + 30 halat (480 gövde) + 50 ragdoll (150 gövde) + 300 yüzen kutu + 500 bölge-izlenen gövde + 900 parçalı yıkılabilir yapı + 2000 uzak LOD-izlenen gövde + her 30 karede patlama+20 mermi | ~2820 | 2.47 ms | **~405 FPS** | 8.67 ms | **~115 FPS** |

**En kötü kare, patlama + 20 mermi ateşlemenin aynı anda gerçekleştiği
karelerdir (30 karede bir) — sürekli çatışma olan en yoğun anı temsil eder.

**Sonuç: bu ölçekte (yukarıdaki TÜM sistemler aynı anda, gerçekçi sayılarla
aktif) fizik motoru 60 FPS hedefinin oldukça altında bir maliyete sahip —
CI'nin jenerik/paylaşımlı CPU'sunda bile ortalama karede bütçenin sadece
~%25'ini kullanıyor, en yoğun anda bile bütçenin altında kalıyor.** Gerçek
oyun donanımında (özellikle CI'dan daha güçlü bir CPU'da) bu sayı daha da
yüksek çıkar. Tekrarlamak gerekirse: bu TEK bir jenerik CPU'nun sonucu,
render/AI/oyun mantığı gibi diğer sistemlerin payını içermez — sadece
fizik motorunun kendi maliyetidir.

### Sistem bazlı değerlendirme — hangisi hangi ölçekte/parçada uygun

- **`physics` (rijit gövde + araç + balistik)** — en olgun, en ucuz katman.
  1200 kutu yığını bile bütçenin ~%6'sını kullanıyor; büyük açık dünya
  sahnelerinde binlerce statik/uykuda gövde sorun değil (Jolt'un kendi
  uyku mekanizması + `lod.dll` ile daha da ucuzlaşır). Araç fiziği ucuz
  (araç başına ~0.013 ms) — 20-40 araçlı bir konvoy/trafik sahnesi rahat.
  Balistik mermi başına ~0.0013 ms — saniyede binlerce mermi (otomatik
  silah sürüleri, shotgun saçması) sorun yaratmaz.
- **`character`** — en ucuz sistemlerden biri (karakter başına ~0.0011 ms).
  100 NPC/kalabalık sahnesi bütçenin %1'inden az; yüzlerce NPC'li kalabalık
  sahneleri (pazar yeri, savaş alanı) rahatlıkla desteklenir.
- **`rope` — dikkat edilmesi gereken en pahalı sistem.** 480 gövdelik
  (30 halat × 16 segment) senaryo bütçenin **%14**'ünü tek başına yiyor —
  segment başına maliyet rijit gövdeden ~3× daha yüksek (her segment bir
  top-eklem constraint'i çözüyor). **Öneri**: sahne başına halat sayısını
  sınırlı tutun (vinç halatı, çekme halatı gibi az sayıda "hero" nesne için
  uygun), çok sayıda dekoratif halat/kablo gerekiyorsa `segmentCount`'u
  düşürün (8-10 segment çoğu görsel amaç için yeterli) veya uzak halatları
  elle basitleştirin (LOD sistemi şu an segment-bazlı değil, gövde-bazlı
  uyutma yapar — halat tamamen uykuya geçene kadar tüm segmentler aktif
  kalır).
- **`cloth`** — orta maliyetli ama vertex başına ucuz (2400 vertex için
  0.44 ms, ~%2.6). Bayrak, çadır bezi, perde gibi az sayıda (sahne başına
  5-20) "hero" kumaş nesnesi için uygun; yüzlerce küçük kumaş parçasından
  oluşan bir sahne (örn. yaprak/konfeti simülasyonu) için tasarlanmadı.
- **`ragdoll`** — ucuz (150 gövde için 0.36 ms). Aynı anda 10-20 aktif
  ragdoll (çok kişili bir çatışma sahnesinin ölüm anları) rahat; ragdoll'lar
  kalıcı değil (tek seferlik "ölüm anı" fiziği), bu yüzden aynı anda çok
  fazla birikmesi beklenmez — birikirse `LOD`/despawn ile temizlenmeli.
- **`buoyancy` / `zones`** — ikisi de hafif (300-500 izlenen gövde için
  ~%2). Açık dünya su/buz/çamur sahnelerinde yüzlerce gövdeyi aynı anda
  izlemek sorun değil.
- **`destructible`** — ölçülemeyecek kadar ucuz (900 parça steady-state
  ~0.001 ms, kademeli çökme dahi ~0.005 ms/çağrı) çünkü parçalar STATİK
  gövdeler — Jolt'un broad phase'i onları neredeyse bedava tutuyor.
  Gerçek maliyet, kırılan parçaları `DetachPieceAsDynamicBody` ile dinamik
  enkaza çevirdiğinizde ortaya çıkar (o zaman `physics` rijit-gövde
  maliyetine döner) — bu yüzden aynı anda çok fazla enkaz parçasını
  dinamikleştirmekten kaçının (birkaç saniye sonra despawn/statikleştirin).
- **`weapons` (patlama)** — ucuz (500 gövdeye tek patlama 0.11 ms).
  Saniyede birkaç patlama (roket/el bombası/topçu) sorun değil; saniyede
  onlarca eşzamanlı patlama (halı bombardımanı) test edilmedi.
- **`lod` — maliyet değil, TASARRUF sistemi.** 2000 uzak gövdede LOD
  kapalıyken 1.04 ms/adım, açıkken 0.11 ms/adım — **~9× hızlanma**. Büyük
  açık dünya sahnelerinde (oyuncudan uzak yüzlerce/binlerce fizik nesnesi
  olan bir harita) `Step()`'e her frame oyuncu pozisyonunu vermek neredeyse
  bedavaya CPU tasarrufu sağlar; tek maliyeti `LODSystem::Update`'in kendi
  döngü taraması (izlenen gövde sayısıyla doğrusal, çok ucuz).
- **`audio` (çarpışma-ses olayları)** ve **`aero`/`sportsball` (paraşüt/
  top fiziği)** ayrı ölçülmedi çünkü maliyetleri kendi başlarına anlamlı
  değil: `audio` bir `ContactListener` callback'i (Jolt zaten her temas
  için tetikliyor, ekstra maliyeti O(1) bir kayıt işlemi); `aero`/
  `sportsball` tek bir gövdeye her frame uygulanan bir kuvvet hesabı
  (birkaç float çarpımı) — asıl maliyet o gövdenin kendisi zaten `physics`
  rijit-gövde satırında ölçülüyor.

## Test

26 test, `ctest`'e kayıtlı (`build/tests/alazforge_test_*`):

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
| `context_integration` | Context'in rüzgar/bölge/yüzerlik/LOD/ses-olayı/terrain/destructible alt sistemlerini tek `Step()` arkasında birleşik yönettiği |
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
| `realism4` | Araç hava direnci, balistik spin drift + determinizm, su derinlik direnci, halat kopması |

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
