#include "vehicle/TrackBeltSystem.h"

#include "core/JoltAdapter.h"

#include <Jolt/Math/Vec3.h>

#include <algorithm>
#include <cmath>

namespace alazforge {

namespace {
constexpr float kTwoPi = 6.28318530717958647692f;
constexpr float kPi = 3.14159265358979323846f;

float NormalizeAngle(float a) {
    while (a > kPi)
        a -= kTwoPi;
    while (a < -kPi)
        a += kTwoPi;
    return a;
}
} // namespace

TrackBeltSystem::TrackBeltSystem(const TrackBeltConfig& inConfig) : config(inConfig) {
    const size_t n = config.wheelLocalPositions.size();
    if (n < 2) return;

    // Düzlem tabanı: planeNormal'e dik iki eksen. World +z'yi (ileri)
    // düzleme izdüşürüp normalize ediyoruz; dejenere olursa (planeNormal
    // zaten +z'ye çok yakınsa) world +y'ye düşüyoruz.
    const JPH::Vec3 normal = ToJolt(config.planeNormal).NormalizedOr(JPH::Vec3::sAxisX());
    JPH::Vec3 refAxis = JPH::Vec3::sAxisZ();
    if (std::fabs(refAxis.Dot(normal)) > 0.99f) refAxis = JPH::Vec3::sAxisY();
    const JPH::Vec3 u = (refAxis - normal * refAxis.Dot(normal)).NormalizedOr(JPH::Vec3::sAxisZ());
    const JPH::Vec3 v = normal.Cross(u).Normalized();
    planeU = FromJolt(u);
    planeV = FromJolt(v);

    // Merkezleri 2B düzlem koordinatına indirger.
    std::vector<float> cx(n), cy(n);
    for (size_t i = 0; i < n; ++i) {
        const JPH::Vec3 p = ToJolt(config.wheelLocalPositions[i]);
        cx[i] = p.Dot(u);
        cy[i] = p.Dot(v);
    }

    // Shoelace formülüyle sarma yönü (CCW mi CW mi); N=2'de alan tam sıfır
    // olur (dejenere) -- bu durumda yön keyfi ama tutarlı seçilir (iki
    // makaralı kayış için hangi tarafın "dış" sayıldığı görsel olarak
    // fark etmez, simetriktir).
    float area = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n;
        area += cx[i] * cy[j] - cx[j] * cy[i];
    }
    const bool ccw = area >= 0.0f;

    const float r = config.wheelRadius;
    std::vector<float> tanInAngle(n), tanOutAngle(n);
    std::vector<float> edgeLen(n);

    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n;
        float dx = cx[j] - cx[i], dy = cy[j] - cy[i];
        const float len = std::sqrt(dx * dx + dy * dy);
        edgeLen[i] = len;
        if (len < 1.0e-6f) {
            tanOutAngle[i] = 0.0f;
            tanInAngle[j] = 0.0f;
            continue;
        }
        dx /= len;
        dy /= len;
        // Disa donuk normal: CCW ise saat yonunde (dy,-dx), CW ise ters.
        const float ox = ccw ? dy : -dy;
        const float oy = ccw ? -dx : dx;
        tanOutAngle[i] = std::atan2(oy, ox);
        tanInAngle[j] = tanOutAngle[i]; // paralel oteleme: giris/cikis tegeti ayni aci
    }

    segments.clear();
    totalLength = 0.0f;
    for (size_t i = 0; i < n; ++i) {
        // Makara i'nin etrafindaki yay: giris tegetinden cikis tegetine.
        float delta = NormalizeAngle(tanOutAngle[i] - tanInAngle[i]);
        if (ccw && delta < 0.0f) delta += kTwoPi;
        if (!ccw && delta > 0.0f) delta -= kTwoPi;

        PathSegment arc;
        arc.isArc = true;
        arc.centerX = cx[i];
        arc.centerY = cy[i];
        arc.radius = r;
        arc.startAngle = tanInAngle[i];
        arc.endAngle = tanInAngle[i] + delta;
        arc.length = std::fabs(delta) * r;
        segments.push_back(arc);
        totalLength += arc.length;

        const size_t j = (i + 1) % n;
        if (edgeLen[i] < 1.0e-6f) continue;
        const float ox = std::cos(tanOutAngle[i]), oy = std::sin(tanOutAngle[i]);
        PathSegment line;
        line.isArc = false;
        line.startX = cx[i] + ox * r;
        line.startY = cy[i] + oy * r;
        line.endX = cx[j] + ox * r;
        line.endY = cy[j] + oy * r;
        line.length = edgeLen[i];
        segments.push_back(line);
        totalLength += line.length;
    }
}

int TrackBeltSystem::LinkCount() const {
    if (config.linkLength <= 0.0f || totalLength <= 0.0f) return 0;
    return static_cast<int>(totalLength / config.linkLength);
}

Vec3 TrackBeltSystem::ToLocal(float x, float y) const {
    return Vec3{planeU.x * x + planeV.x * y, planeU.y * x + planeV.y * y,
                planeU.z * x + planeV.z * y};
}

void TrackBeltSystem::ComputeLinkTransforms(float beltOffset,
                                            std::vector<Transform>& outLinks) const {
    outLinks.clear();
    const int count = LinkCount();
    if (count <= 0 || segments.empty()) return;
    outLinks.reserve(static_cast<size_t>(count));

    float offset = std::fmod(beltOffset, totalLength);
    if (offset < 0.0f) offset += totalLength;

    for (int link = 0; link < count; ++link) {
        float s = std::fmod(offset + static_cast<float>(link) * config.linkLength, totalLength);
        if (s < 0.0f) s += totalLength;

        // s'nin duştuğu segmenti bul.
        float acc = 0.0f;
        float px = 0, py = 0, tx = 1, ty = 0;
        for (const PathSegment& seg : segments) {
            if (s <= acc + seg.length || &seg == &segments.back()) {
                const float local = std::max(0.0f, std::min(s - acc, seg.length));
                if (seg.isArc) {
                    const float t = seg.length > 1.0e-6f ? local / seg.length : 0.0f;
                    const float ang = seg.startAngle + (seg.endAngle - seg.startAngle) * t;
                    px = seg.centerX + std::cos(ang) * seg.radius;
                    py = seg.centerY + std::sin(ang) * seg.radius;
                    // Teğet yön: yarıçap vektörüne dik, sarma yönünde.
                    const float sign = (seg.endAngle >= seg.startAngle) ? 1.0f : -1.0f;
                    tx = -std::sin(ang) * sign;
                    ty = std::cos(ang) * sign;
                } else {
                    const float t = seg.length > 1.0e-6f ? local / seg.length : 0.0f;
                    px = seg.startX + (seg.endX - seg.startX) * t;
                    py = seg.startY + (seg.endY - seg.startY) * t;
                    const float dx = seg.endX - seg.startX, dy = seg.endY - seg.startY;
                    const float dl = std::sqrt(dx * dx + dy * dy);
                    tx = dl > 1.0e-6f ? dx / dl : 1.0f;
                    ty = dl > 1.0e-6f ? dy / dl : 0.0f;
                }
                break;
            }
            acc += seg.length;
        }

        Transform t;
        t.position = ToLocal(px, py);
        // Rotasyon: yerel +z = kayış ilerleme (teğet) yönü, yerel +y =
        // düzlem normaline mümkün olduğunca yakın (link "yuvarlanmaz",
        // hep düzlem içinde durur). Ortonormal üçlüden (right,up,forward)
        // dönüş matrisi kurulup kuaterniyona çevrilir.
        const Vec3 fwd = ToLocal(tx, ty);
        const JPH::Vec3 jFwd = ToJolt(fwd).NormalizedOr(JPH::Vec3::sAxisZ());
        const JPH::Vec3 jNormal = ToJolt(config.planeNormal).NormalizedOr(JPH::Vec3::sAxisX());
        JPH::Vec3 jRight = jNormal.Cross(jFwd).NormalizedOr(JPH::Vec3::sAxisX());
        const JPH::Vec3 jUp = jFwd.Cross(jRight).Normalized();
        jRight = jUp.Cross(jFwd).Normalized();
        const JPH::Mat44 rot(JPH::Vec4(jRight, 0), JPH::Vec4(jUp, 0), JPH::Vec4(jFwd, 0),
                             JPH::Vec4(0, 0, 0, 1));
        t.rotation = FromJolt(rot.GetQuaternion());
        outLinks.push_back(t);
    }
}

} // namespace alazforge
