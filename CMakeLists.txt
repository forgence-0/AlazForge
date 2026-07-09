cmake_minimum_required(VERSION 3.20)
project(AlazForge LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# alazforge SHARED hedefi, statik olarak derlenen Jolt/lz4 nesne dosyalarını
# içine alacak; bu yüzden PIC, alt dizinler eklenmeden ÖNCE global olarak açılmalı.
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ── Jolt Physics (external/JoltPhysics, MIT) ────────────────────────
set(TARGET_UNIT_TESTS OFF CACHE BOOL "" FORCE)
set(TARGET_HELLO_WORLD OFF CACHE BOOL "" FORCE)
set(TARGET_PERFORMANCE_TEST OFF CACHE BOOL "" FORCE)
set(TARGET_SAMPLES OFF CACHE BOOL "" FORCE)
set(TARGET_VIEWER OFF CACHE BOOL "" FORCE)
set(ENABLE_ALL_WARNINGS OFF CACHE BOOL "" FORCE)
# AlazEngine dinamik CRT kullanıyor; Jolt da uyumlu olsun
set(USE_STATIC_MSVC_RUNTIME_LIBRARY OFF CACHE BOOL "" FORCE)
add_subdirectory(external/JoltPhysics/Build)

# ── LZ4 (external/lz4, BSD-2) — tek çeviri birimi yeterli ──────────
add_library(lz4_static STATIC external/lz4/lib/lz4.c)
target_include_directories(lz4_static PUBLIC external/lz4/lib)

# ── alazforge: paylaşımlı kütüphane (DLL/so) ─────────────────────────
include(GenerateExportHeader)

add_library(alazforge SHARED
    src/material_db/MaterialDatabase.cpp
    src/streaming/ChunkStreamSystem.cpp
    src/terrain_deform/TerrainDeformSystem.cpp
    src/vehicle/WreckPersistence.cpp
    src/vehicle/VehicleSystem.cpp
    src/ballistics/BallisticsSystem.cpp
)

target_include_directories(alazforge PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
)

generate_export_header(alazforge
    EXPORT_MACRO_NAME ALAZFORGE_API
    EXPORT_FILE_NAME ${CMAKE_CURRENT_BINARY_DIR}/generated_include/alazforge/export.h
)
target_include_directories(alazforge PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/generated_include>
)

# Linux'ta Windows dllexport/dllimport semantiğini taklit eder: hiçbir sembol
# ALAZFORGE_API ile işaretlenmeden dışa açılmaz. Eksik export'ları CI'da yakalar.
set_target_properties(alazforge PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN ON
)

target_link_libraries(alazforge
    # JPH:: tipleri AlazForge'un kendi public header'larında (VehicleSystem.h,
    # BallisticsSystem.h, ...) göründüğü için Jolt PUBLIC olmalı.
    PUBLIC Jolt
    # LZ4 hiçbir public header'da görünmüyor, yalnızca dahili kullanım.
    PRIVATE lz4_static
)

# ── Testler ─────────────────────────────────────────────────────────
enable_testing()
add_subdirectory(tests)
