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

    JPH::CharacterVirtualSettings settings;
    settings.SetEmbedded();
    settings.mShape = MakeCapsule(config.height);
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

    const float speed =
        crouching ? config.crouchSpeed : (runInput ? config.runSpeed : config.walkSpeed);
    JPH::Vec3 desiredHorizontal(moveInput.x * speed, 0, moveInput.z * speed);
    // Dik yamaclara dogru itmeyi kes (yamacta zıplayarak tirmanmayi onler)
    desiredHorizontal = character->CancelVelocityTowardsSteepSlopes(desiredHorizontal);

    const JPH::Vec3 gravity(0, config.gravity, 0);

    // Yuzme modu: ayak, su yuzeyinin swimSubmergeDepth kadar altindaysa
    const float feetY = static_cast<float>(character->GetPosition().GetY());
    swimming = waterValid && (waterSurfaceY - feetY) > config.swimSubmergeDepth;
    if (swimming) {
        // Yercekimi yok: yuzme girdisi + yuzeye kaldirma + akinti
        JPH::Vec3 swimVel(moveInput.x * config.swimSpeed, 0, moveInput.z * config.swimSpeed);
        const float depthBelowSurface = waterSurfaceY - feetY;
        // Yuzeye yakinken kaldirma azalir (yuzeyde asagi-yukari titremesin)
        const float rise =
            config.swimRiseSpeed * std::min(1.0f, depthBelowSurface / config.swimSubmergeDepth);
        swimVel += JPH::Vec3(waterFlow.x, rise + waterFlow.y, waterFlow.z);
        jumpRequested = false;
        character->SetLinearVelocity(swimVel);

        JPH::CharacterVirtual::ExtendedUpdateSettings swimSettings;
        swimSettings.mWalkStairsStepUp = JPH::Vec3::sZero();     // suda merdiven yok
        swimSettings.mStickToFloorStepDown = JPH::Vec3::sZero(); // zemine yapisma yok
        character->ExtendedUpdate(dt, JPH::Vec3::sZero(), swimSettings,
                                  physics->GetDefaultBroadPhaseLayerFilter(layer),
                                  physics->GetDefaultLayerFilter(layer), JPH::BodyFilter{},
                                  JPH::ShapeFilter{}, *tempAllocator);
        return;
    }

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

JPH::RefConst<JPH::Shape> CharacterController::MakeCapsule(float totalHeight) const {
    // Kapsul, karakterin AYAK noktasi orijinde olacak sekilde yukari
    // kaydirilir (Jolt'un kendi karakter ornekleriyle ayni kurulum) --
    // boylece GetWorldTransform().position dogrudan ayak/zemin noktasidir.
    const float cylinderHalfHeight = std::max(0.05f, 0.5f * totalHeight - config.radius);
    JPH::RotatedTranslatedShapeSettings shapeSettings(
        JPH::Vec3(0, cylinderHalfHeight + config.radius, 0), JPH::Quat::sIdentity(),
        new JPH::CapsuleShape(cylinderHalfHeight, config.radius));
    shapeSettings.SetEmbedded();
    return shapeSettings.Create().Get();
}

bool CharacterController::SetCrouch(bool crouch) {
    if (!character || !physics || crouch == crouching) return crouch == crouching;
    const float targetHeight = crouch ? config.crouchHeight : config.height;
    // SetShape penetrasyon kontrolu yapar: ayaga kalkarken tavan varsa
    // false doner ve sekil degismez (comelmede kaliriz).
    const bool ok = character->SetShape(MakeCapsule(targetHeight), 0.01f,
                                        physics->GetDefaultBroadPhaseLayerFilter(layer),
                                        physics->GetDefaultLayerFilter(layer), JPH::BodyFilter{},
                                        JPH::ShapeFilter{}, *tempAllocator);
    if (ok) crouching = crouch;
    return ok;
}

void CharacterController::SetWaterState(bool inWater, float surfaceY, const Vec3& flowVelocity) {
    waterValid = inWater;
    waterSurfaceY = surfaceY;
    waterFlow = flowVelocity;
}

void CharacterController::Destroy() {
    character = nullptr;
    tempAllocator.reset();
    physics = nullptr;
}

} // namespace alazforge
