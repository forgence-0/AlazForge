cmake_minimum_required(VERSION 3.20)
project(AlazForge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

# ── alazforge_core: adapter katmanı (header-only) ───────────────────
add_library(alazforge_core INTERFACE)
target_include_directories(alazforge_core INTERFACE src)
target_link_libraries(alazforge_core INTERFACE Jolt)

# ── Testler ─────────────────────────────────────────────────────────
enable_testing()
add_subdirectory(tests)
