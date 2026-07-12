#pragma once
// Karakter kontrolcüsü: Jolt'un CharacterVirtual'ını saran yürüme/zıplama/
// merdiven/eğim sistemi. CharacterVirtual "sanal" bir karakterdir — dünyada
// gerçek bir rigid body olarak yaşamaz, her Update'te shape-cast ile çarpışma
// çözümü yapar; bu, oyuncu kontrolü için rigid-body karakterlerden çok daha
// kararlı davranış verir (Jolt'un önerdiği yaklaşım).
//
// Kullanım (her frame):
//   controller.SetMoveInput(yatayYon, kosuyorMu);
//   if (ziplaBasildi) controller.Jump();
//   controller.Update(dt);
//   Transform t = controller.GetWorldTransform(); // render'a ver
//
// Merdiven çıkma (WalkStairs) ve zemine yapışma (StickToFloor) Jolt'un
// ExtendedUpdate'i üzerinden otomatik uygulanır.

#include "core/JoltAdapter.h"

#include <character/export.h>

#include <Jolt/Jolt.h>

#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/PhysicsSystem.h>

#include <memory>

namespace alazforge {

struct CharacterControllerConfig {
    float radius = 0.35f;      // kapsül yarıçapı (m)
    float height = 1.8f;       // toplam boy (m), kapsül = height - 2*radius silindir kısmı
    float mass = 80.0f;        // itilen dinamik gövdelere uygulanan etki için
    float maxSlopeDeg = 50.0f; // bu eğimden dik yamaçlar "zemin" sayılmaz
    float walkSpeed = 4.0f;    // m/s
    float runSpeed = 8.0f;     // m/s
    float jumpSpeed = 6.0f;    // zıplama anındaki dikey hız (m/s)
    float gravity = -9.81f;    // m/s^2 (y ekseni)
    float airControl = 0.25f;  // havadayken yatay girdinin etki oranı [0..1]
    float stepUpHeight = 0.4f; // tek adımda çıkılabilen merdiven yüksekliği (m)
};

class CHARACTER_API CharacterController {
public:
    explicit CharacterController(const CharacterControllerConfig& inConfig);
    ~CharacterController();

    CharacterController(const CharacterController&) = delete;
    CharacterController& operator=(const CharacterController&) = delete;

    // Karakteri dünyaya yerleştirir. layer: karakterin çarpışma katmanı
    // (tipik olarak MOVING).
    void Spawn(JPH::PhysicsSystem& inPhysics, const Vec3& worldPosition, JPH::ObjectLayer layer);

    // Yatay hareket girdisi (dünya uzayında, y bileşeni yok sayılır).
    // Normalize edilmemiş olabilir; büyüklük 1'e kırpılır.
    void SetMoveInput(const Vec3& direction, bool running = false);

    // Zeminde ise bir sonraki Update'te zıplar (havada çağrılırsa yok sayılır).
    void Jump();

    // Fizik adımıyla birlikte her frame çağrılır (yerçekimi + hareket +
    // merdiven + zemine yapışma çözümü burada yapılır).
    void Update(float dt);

    bool IsOnGround() const;
    Vec3 GetVelocity() const;
    Transform GetWorldTransform() const;
    void Teleport(const Vec3& worldPosition);

    void Destroy();

private:
    CharacterControllerConfig config;
    JPH::PhysicsSystem* physics = nullptr;
    JPH::ObjectLayer layer = 0;
    JPH::Ref<JPH::CharacterVirtual> character;
    std::unique_ptr<JPH::TempAllocatorImpl> tempAllocator;

    Vec3 moveInput{0, 0, 0};
    bool runInput = false;
    bool jumpRequested = false;
};

} // namespace alazforge
