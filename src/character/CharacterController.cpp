#include "character/CharacterController.h"

#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>

#include <algorithm>

namespace alazforge {

CharacterController::CharacterController(const CharacterControllerConfig& inConfig)
    : config(inConfig) {}

CharacterController::~CharacterController() { Destroy(); }

void CharacterController::Spawn(JPH::PhysicsSystem& inPhysics, const Vec3& worldPosition,
                                JPH::ObjectLayer inLayer) {
    Destroy();
    physics = &inPhysics;
    layer = inLayer;

    // CharacterVirtual::Update her cagrida gecici bellek kullanir; kucuk,
    // kontrolcuye ozel bir allocator yeterli.
    tempAllocator = std::make_unique<JPH::TempAllocatorImpl>(1024 * 1024);

    // Kapsul, karakterin AYAK noktasi orijinde olacak sekilde yukari
    // kaydirilir (Jolt'un kendi karakter ornekleriyle ayni kurulum) --
    // boylece GetWorldTransform().position dogrudan ayak/zemin noktasidir.
    const float cylinderHalfHeight = std::max(0.05f, 0.5f * config.height - config.radius);
    JPH::RotatedTranslatedShapeSettings shapeSettings(
        JPH::Vec3(0, cylinderHalfHeight + config.radius, 0), JPH::Quat::sIdentity(),
        new JPH::CapsuleShape(cylinderHalfHeight, config.radius));
    shapeSettings.SetEmbedded();

    JPH::CharacterVirtualSettings settings;
    settings.SetEmbedded();
    settings.mShape = shapeSettings.Create().Get();
    settings.mMass = config.mass;
    settings.mMaxSlopeAngle = JPH::DegreesToRadians(config.maxSlopeDeg);
    // Ayak kuresinin merkezinin altindaki temaslar "destekleyici" sayilir --
    // karakter kapsul kenarina surtunen duvarlara asilip kalmaz.
    settings.mSupportingVolume = JPH::Plane(JPH::Vec3::sAxisY(), -config.radius);

    character = new JPH::CharacterVirtual(&settings, ToJoltR(worldPosition), JPH::Quat::sIdentity(),
                                          physics);
}

void CharacterController::SetMoveInput(const Vec3& direction, bool running) {
    moveInput = Vec3{direction.x, 0.0f, direction.z};
    const float len = std::sqrt(moveInput.x * moveInput.x + moveInput.z * moveInput.z);
    if (len > 1.0f) {
        moveInput.x /= len;
        moveInput.z /= len;
    }
    runInput = running;
}

void CharacterController::Jump() {
    if (character && character->GetGroundState() == JPH::CharacterBase::EGroundState::OnGround)
        jumpRequested = true;
}

void CharacterController::Update(float dt) {
    if (!character || !physics) return;

    const float speed = runInput ? config.runSpeed : config.walkSpeed;
    JPH::Vec3 desiredHorizontal(moveInput.x * speed, 0, moveInput.z * speed);
    // Dik yamaclara dogru itmeyi kes (yamacta zıplayarak tirmanmayi onler)
    desiredHorizontal = character->CancelVelocityTowardsSteepSlopes(desiredHorizontal);

    const JPH::Vec3 gravity(0, config.gravity, 0);
    JPH::Vec3 newVelocity;
    if (character->GetGroundState() == JPH::CharacterBase::EGroundState::OnGround) {
        // Zeminde: zemin hizini devral (hareketli platform destegi) + girdi
        newVelocity = character->GetGroundVelocity() + desiredHorizontal;
        if (jumpRequested) newVelocity += JPH::Vec3(0, config.jumpSpeed, 0);
    } else {
        // Havada: dusey hiz korunur, yatay girdi kismi etki eder
        const JPH::Vec3 current = character->GetLinearVelocity();
        newVelocity = JPH::Vec3(current.GetX(), current.GetY(), current.GetZ());
        newVelocity += desiredHorizontal * config.airControl * dt;
    }
    newVelocity += gravity * dt;
    jumpRequested = false;

    character->SetLinearVelocity(newVelocity);

    JPH::CharacterVirtual::ExtendedUpdateSettings updateSettings;
    updateSettings.mWalkStairsStepUp = JPH::Vec3(0, config.stepUpHeight, 0);
    character->ExtendedUpdate(dt, gravity, updateSettings,
                              physics->GetDefaultBroadPhaseLayerFilter(layer),
                              physics->GetDefaultLayerFilter(layer), JPH::BodyFilter{},
                              JPH::ShapeFilter{}, *tempAllocator);
}

bool CharacterController::IsOnGround() const {
    return character && character->GetGroundState() == JPH::CharacterBase::EGroundState::OnGround;
}

Vec3 CharacterController::GetVelocity() const {
    return character ? FromJolt(character->GetLinearVelocity()) : Vec3{0, 0, 0};
}

Transform CharacterController::GetWorldTransform() const {
    Transform t;
    if (character) {
        t.position = FromJolt(JPH::Vec3(character->GetPosition()));
        t.rotation = FromJolt(character->GetRotation());
    }
    return t;
}

void CharacterController::Teleport(const Vec3& worldPosition) {
    if (!character || !physics) return;
    character->SetPosition(ToJoltR(worldPosition));
    character->RefreshContacts(physics->GetDefaultBroadPhaseLayerFilter(layer),
                               physics->GetDefaultLayerFilter(layer), JPH::BodyFilter{},
                               JPH::ShapeFilter{}, *tempAllocator);
}

void CharacterController::Destroy() {
    character = nullptr;
    tempAllocator.reset();
    physics = nullptr;
}

} // namespace alazforge
