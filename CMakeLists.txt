cmake_minimum_required(VERSION 3.20)
project(AlazForge LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Tüm SHARED hedefler statik olarak derlenen lz4 nesne dosyalarını içine
# alabilir; bu yüzden PIC, alt dizinler eklenmeden ÖNCE global olarak açılmalı.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ── Jolt Physics (external/JoltPhysics, MIT) — kendi DLL'i (jolt.dll) ──
# BUILD_SHARED_LIBS ON iken Jolt'un kendi CMakeLists'i "Jolt" hedefini
# SHARED olarak kurar (JPH_SHARED_LIBRARY/JPH_EXPORT Jolt tarafında
# otomatik devreye girer). NOT: bu ortamda submodule checkout edilmediği
# için Jolt'un SHARED build desteği burada derlenip doğrulanamadı; ilk
# derlemede teyit edilmeli.
set(BUILD_SHARED_LIBS ON)

set(TARGET_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(TARGET_HELLO_WORLD OFF CACHE BOOL "" FORCE)
set(TARGET_PERFORMANCE_TEST OFF CACHE BOOL "" FORCE)
set(TARGET_SAMPLES OFF CACHE BOOL "" FORCE)
set(TARGET_VIEWER OFF CACHE BOOL "" FORCE)
set(ENABLE_ALL_WARNINGS OFF CACHE BOOL "" FORCE)
# AlazEngine dinamik CRT kullanıyor; Jolt da uyumlu olsun
set(USE_STATIC_MSVC_RUNTIME_LIBRARY OFF CACHE BOOL "" FORCE)
add_subdirectory(external/JoltPhysics/Build)
set_target_properties(Jolt PROPERTIES OUTPUT_NAME jolt)

# ── LZ4 (external/lz4, BSD-2) — tek çeviri birimi, her zaman statik ────
add_library(lz4_static STATIC external/lz4/lib/lz4.c)
target_include_directories(lz4_static PUBLIC external/lz4/lib)

include(GenerateExportHeader)

# ── physics: ana fizik kütüphanesi (physics.dll / libphysics.so) ─────
# Temel 6 faz: adapter, spatial streaming, terrain deform, balistik,
# malzeme db, araç fiziği + enkaz kalıcılığı.
add_library(physics SHARED
    src/material_db/MaterialDatabase.cpp
    src/streaming/ChunkStreamSystem.cpp
    src/terrain_deform/TerrainDeformSystem.cpp
    src/vehicle/WreckPersistence.cpp
    src/vehicle/VehicleSystem.cpp
    src/vehicle/TrackBeltSystem.cpp
    src/ballistics/BallisticsSystem.cpp
    src/wind/WindSystem.cpp
)

target_include_directories(physics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

generate_export_header(physics
    EXPORT_MACRO_NAME PHYSICS_API
    EXPORT_FILE_NAME ${CMAKE_CURRENT_BINARY_DIR}/generated_include/physics/export.h
)
target_include_directories(physics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated_include>
)

# Linux'ta Windows dllexport/dllimport semantiğini taklit eder: hiçbir sembol
# *_API ile işaretlenmeden dışa açılmaz. Eksik export'ları CI'da yakalar.
set_target_properties(physics PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
)

target_link_libraries(physics
    # JPH:: tipleri kendi public header'larında (VehicleSystem.h,
    # BallisticsSystem.h, ...) göründüğü için Jolt PUBLIC olmalı. LZ4 de
    # PUBLIC olmalı: ChunkStreamSystem.h (<lz4.h> include eder) physics'in
    # public bir header'ı ve destructible gibi ayrı hedefler physics'i
    # yalnızca PUBLIC link ederek bu header'a transitively erişiyor --
    # lz4_static PRIVATE kalirsa include yolu o hedeflere yayilmiyor
    # (yalnizca sistemde libz4-dev kurulu oldugu icin CI'da tesadufen
    # calisiyordu, bu ortamda gercek eksiklik yakalandi).
    PUBLIC Jolt lz4_static
)

# ── Yardımcı/genişletme DLL'leri (Faz B) — her biri kendi başına, hepsi
# yalnızca physics'e bağımlı (birbirlerine değil) ───────────────────────
function(alazforge_add_helper_library name export_macro)
    add_library(${name} SHARED ${ARGN})
    generate_export_header(${name}
        EXPORT_MACRO_NAME ${export_macro}
        EXPORT_FILE_NAME ${CMAKE_CURRENT_BINARY_DIR}/generated_include/${name}/export.h
    )
    target_include_directories(${name} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated_include>
    )
    set_target_properties(${name} PROPERTIES
        CXX_VISIBILITY_PRESET hidden
        VISIBILITY_INLINES_HIDDEN ON
    )
    target_link_libraries(${name} PUBLIC physics)
endfunction()

# weapons.dll — geri tepme, sekme/dağılım, türet mount, patlama
alazforge_add_helper_library(weapons WEAPONS_API
    src/weapons/TurretMount.cpp
    src/weapons/ExplosionSystem.cpp
    src/weapons/TrajectoryPredictor.cpp
)

# character.dll — CharacterVirtual tabanlı oyuncu/NPC kontrolcüsü
alazforge_add_helper_library(character CHARACTER_API src/character/CharacterController.cpp)

# rope.dll — segment zinciri tabanlı halat/kablo fiziği
alazforge_add_helper_library(rope ROPE_API src/rope/RopeSystem.cpp)

# buoyancy.dll — su hacimleri, yüzerlik
alazforge_add_helper_library(buoyancy BUOYANCY_API src/buoyancy/BuoyancySystem.cpp)

# destructible.dll — yıkılabilir yapılar
alazforge_add_helper_library(destructible DESTRUCTIBLE_API
    src/destructible/DestructibleChunkData.cpp
    src/destructible/DestructibleStructureSystem.cpp
)

# ragdoll.dll — ragdoll iskelet + örnek
alazforge_add_helper_library(ragdoll RAGDOLL_API
    src/ragdoll/RagdollSkeleton.cpp
    src/ragdoll/RagdollInstance.cpp
)

# audio.dll — çarpışma olaylarını (şiddet+malzeme) ses tetiklemesi için
# olay listesine çeviren köprü (ContactListener tabanlı)
alazforge_add_helper_library(audio AUDIO_API src/audio/CollisionEventSystem.cpp)

# lod.dll — mesafeye göre gövde uyutma/uyandırma (genel amaçlı LOD)
alazforge_add_helper_library(lod LOD_API src/lod/LODSystem.cpp)

# aero.dll — paraşüt/planör aerodinamiği (taşıma + sürükleme)
alazforge_add_helper_library(aero AERO_API src/aero/AeroSystem.cpp)

# zones.dll — sürtünme bölgeleri (buz/çamur/yağ zemin koşulları)
alazforge_add_helper_library(zones ZONES_API src/zones/FrictionZoneSystem.cpp)

# magnetism.dll — manyetik/çekim alanları (izlenen gövdelere radyal kuvvet)
alazforge_add_helper_library(magnetism MAGNETISM_API src/magnetism/MagneticFieldSystem.cpp)

# fire.dll — ateş/yanma yayılım simülasyonu (Jolt'a bağımlı değil, saf graf)
alazforge_add_helper_library(fire FIRE_API src/fire/FireSpreadSystem.cpp)

# cloth.dll — Jolt SoftBody tabanlı kumaş/bayrak simülasyonu
alazforge_add_helper_library(cloth CLOTH_API src/cloth/ClothSystem.cpp)

# debugdraw.dll — Jolt DebugRenderer -> AlazEngine çizim callback köprüsü
alazforge_add_helper_library(debugdraw DEBUGDRAW_API src/debugdraw/DebugDrawBridge.cpp)
# Jolt varsayılan olarak RTTI'siz derlenir (CPP_RTTI_ENABLED OFF), bu yüzden
# jolt.dll typeinfo sembolleri içermez. DebugRendererSimple'dan türediğimiz
# için debugdraw da RTTI'siz derlenmeli — aksi halde türetilmiş sınıfın
# typeinfo'su base'inkine referans verir ve "undefined reference to typeinfo
# for JPH::DebugRendererSimple" link hatası oluşur (gerçek derlemede yakalandı).
if(MSVC)
    target_compile_options(debugdraw PRIVATE /GR-)
else()
    target_compile_options(debugdraw PRIVATE -fno-rtti)
endif()

# sportsball.dll — topla oynanan sporlar icin genel top fizigi (surukleme +
# Magnus/spin etkisi); futbol/basketbol/hentbol/amerikan futbolu gibi
# sporlara ozel hazir preset yok, her top BallConfig ile kurulur.
alazforge_add_helper_library(sportsball SPORTSBALL_API
    src/sportsball/BallPhysicsSystem.cpp
)

# context.dll — AlazForgeContext: AlazEngine entegrasyonu icin TEK giris
# noktasi (facade). Diger yardimcilarin aksine (yalnizca physics'e bagimli
# olma kurali), context BILINCLI OLARAK istisna -- amaci butun world-scoped
# alt sistemleri (zones/buoyancy/lod/audio/debugdraw/destructible) tek bir
# Step() arkasinda toplamak, bu yuzden hepsine bagimli olmasi gerekiyor.
alazforge_add_helper_library(context CONTEXT_API src/context/AlazForgeContext.cpp)
target_link_libraries(context PUBLIC zones buoyancy lod audio debugdraw destructible)

# ── Testler ─────────────────────────────────────────────────────────
enable_testing()
add_subdirectory(tests)
