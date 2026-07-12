#include "destructible/VoronoiFracture.h"

#include "core/JoltAdapter.h"

#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>

#include <algorithm>
#include <cmath>

namespace alazforge {

namespace {

// Dışbükey çokyüzlü kırpma sırasında bir yüzü (poligonu) temsil eder.
struct Face {
    std::vector<JPH::Vec3> poly;
};

std::vector<Face> MakeBoxFaces(JPH::Vec3 mn, JPH::Vec3 mx) {
    std::vector<Face> faces(6);
    faces[0].poly = {JPH::Vec3(mn.GetX(), mn.GetY(), mn.GetZ()),
                     JPH::Vec3(mn.GetX(), mx.GetY(), mn.GetZ()),
                     JPH::Vec3(mn.GetX(), mx.GetY(), mx.GetZ()),
                     JPH::Vec3(mn.GetX(), mn.GetY(), mx.GetZ())}; // -X
    faces[1].poly = {JPH::Vec3(mx.GetX(), mn.GetY(), mn.GetZ()),
                     JPH::Vec3(mx.GetX(), mn.GetY(), mx.GetZ()),
                     JPH::Vec3(mx.GetX(), mx.GetY(), mx.GetZ()),
                     JPH::Vec3(mx.GetX(), mx.GetY(), mn.GetZ())}; // +X
    faces[2].poly = {JPH::Vec3(mn.GetX(), mn.GetY(), mn.GetZ()),
                     JPH::Vec3(mn.GetX(), mn.GetY(), mx.GetZ()),
                     JPH::Vec3(mx.GetX(), mn.GetY(), mx.GetZ()),
                     JPH::Vec3(mx.GetX(), mn.GetY(), mn.GetZ())}; // -Y
    faces[3].poly = {JPH::Vec3(mn.GetX(), mx.GetY(), mn.GetZ()),
                     JPH::Vec3(mx.GetX(), mx.GetY(), mn.GetZ()),
                     JPH::Vec3(mx.GetX(), mx.GetY(), mx.GetZ()),
                     JPH::Vec3(mn.GetX(), mx.GetY(), mx.GetZ())}; // +Y
    faces[4].poly = {JPH::Vec3(mn.GetX(), mn.GetY(), mn.GetZ()),
                     JPH::Vec3(mx.GetX(), mn.GetY(), mn.GetZ()),
                     JPH::Vec3(mx.GetX(), mx.GetY(), mn.GetZ()),
                     JPH::Vec3(mn.GetX(), mx.GetY(), mn.GetZ())}; // -Z
    faces[5].poly = {JPH::Vec3(mn.GetX(), mn.GetY(), mx.GetZ()),
                     JPH::Vec3(mn.GetX(), mx.GetY(), mx.GetZ()),
                     JPH::Vec3(mx.GetX(), mx.GetY(), mx.GetZ()),
                     JPH::Vec3(mx.GetX(), mn.GetY(), mx.GetZ())}; // +Z
    return faces;
}

// Bir yüzü n.p <= d yarı-uzayına kırpar (Sutherland-Hodgman). Düzlem
// üzerinde oluşan yeni köşe noktalarını newPointsOut'a ekler -- bunlar
// birden fazla yüzün kırpılmasından toplanıp yeni bir "kapak" yüzü
// oluşturmak için kullanılır (aşağıdaki ClipPolytope).
Face ClipFace(const Face& face, JPH::Vec3 n, float d, std::vector<JPH::Vec3>& newPointsOut) {
    Face out;
    const size_t count = face.poly.size();
    if (count == 0) return out;
    for (size_t i = 0; i < count; ++i) {
        const JPH::Vec3 cur = face.poly[i];
        const JPH::Vec3 nxt = face.poly[(i + 1) % count];
        const float dCur = n.Dot(cur) - d;
        const float dNxt = n.Dot(nxt) - d;
        const bool curIn = dCur <= 1.0e-6f;
        const bool nxtIn = dNxt <= 1.0e-6f;
        if (curIn) out.poly.push_back(cur);
        if (curIn != nxtIn) {
            const float t = dCur / (dCur - dNxt);
            const JPH::Vec3 ip = cur + (nxt - cur) * t;
            out.poly.push_back(ip);
            newPointsOut.push_back(ip);
        }
    }
    return out;
}

// Tüm çokyüzlüyü n.p <= d yarı-uzayına kırpar; kırpma düzleminde ortaya
// çıkan yeni yüzü (varsa) açıya göre sıralayıp ekler.
std::vector<Face> ClipPolytope(const std::vector<Face>& faces, JPH::Vec3 n, float d) {
    std::vector<Face> result;
    result.reserve(faces.size() + 1);
    std::vector<JPH::Vec3> capPoints;
    for (const Face& f : faces) {
        Face clipped = ClipFace(f, n, d, capPoints);
        if (clipped.poly.size() >= 3) result.push_back(std::move(clipped));
    }
    if (result.empty()) return result; // hucre bu duzlemin gerisinde tamamen yok oldu

    if (capPoints.size() >= 3) {
        JPH::Vec3 centroid = JPH::Vec3::sZero();
        for (const JPH::Vec3& p : capPoints)
            centroid += p;
        centroid /= static_cast<float>(capPoints.size());

        // n'e dik iki baz vektor (kapak duzlemi icinde acisal siralama icin).
        // n'ye EN AZ hizali eksen secilir (0.99 esikli onceki yaklasim,
        // n neredeyse esik degerine denk geldiginde cross product'in
        // neredeyse sifir uzunlukta olmasina -- Normalized()'da NaN/Inf'e --
        // izin verebiliyordu; bu Windows'ta Jolt'un kayan nokta istisna
        // tuzagi tarafindan yakalanip gercek bir crash'e donustu).
        // En az hizali eksen secilince cross product uzunlugu her zaman
        // en az sin(54.7 derece) ~ 0.816 olur, sifira asla yaklasmaz.
        const float ax = std::abs(n.GetX()), ay = std::abs(n.GetY()), az = std::abs(n.GetZ());
        JPH::Vec3 up;
        if (ax <= ay && ax <= az)
            up = JPH::Vec3::sAxisX();
        else if (ay <= ax && ay <= az)
            up = JPH::Vec3::sAxisY();
        else
            up = JPH::Vec3::sAxisZ();
        const JPH::Vec3 tangent = up.Cross(n).Normalized();
        const JPH::Vec3 bitangent = n.Cross(tangent);

        std::sort(capPoints.begin(), capPoints.end(), [&](const JPH::Vec3& a, const JPH::Vec3& b) {
            const JPH::Vec3 da = a - centroid;
            const JPH::Vec3 db = b - centroid;
            const float angA = std::atan2(da.Dot(bitangent), da.Dot(tangent));
            const float angB = std::atan2(db.Dot(bitangent), db.Dot(tangent));
            return angA < angB;
        });

        Face cap;
        cap.poly = std::move(capPoints);
        result.push_back(std::move(cap));
    }
    return result;
}

} // namespace

std::vector<VoronoiCell> ComputeVoronoiCells(const Vec3& boxMin, const Vec3& boxMax,
                                             const std::vector<Vec3>& seedsIn) {
    std::vector<VoronoiCell> cells;
    cells.reserve(seedsIn.size());

    std::vector<JPH::Vec3> seeds;
    seeds.reserve(seedsIn.size());
    for (const Vec3& s : seedsIn)
        seeds.push_back(ToJolt(s));

    const JPH::Vec3 mn = ToJolt(boxMin);
    const JPH::Vec3 mx = ToJolt(boxMax);

    for (size_t si = 0; si < seeds.size(); ++si) {
        std::vector<Face> faces = MakeBoxFaces(mn, mx);

        for (size_t sj = 0; sj < seeds.size() && !faces.empty(); ++sj) {
            if (sj == si) continue;
            const JPH::Vec3 diff = seeds[sj] - seeds[si];
            const float len = diff.Length();
            if (len < 1.0e-6f) continue;
            const JPH::Vec3 n = diff / len;
            const JPH::Vec3 mid = (seeds[si] + seeds[sj]) * 0.5f;
            const float d = n.Dot(mid);
            faces = ClipPolytope(faces, n, d);
        }

        if (faces.empty()) continue; // (nadiren) bu tohum icin hicbir bolge kalmadi

        VoronoiCell cell;
        JPH::Vec3 centroid = JPH::Vec3::sZero();
        size_t total = 0;
        for (const Face& f : faces) {
            for (const JPH::Vec3& v : f.poly) {
                cell.vertices.push_back(FromJolt(v));
                centroid += v;
                ++total;
            }
        }
        if (total > 0) centroid /= static_cast<float>(total);
        cell.centroid = FromJolt(centroid);
        cells.push_back(std::move(cell));
    }

    return cells;
}

std::vector<JPH::BodyID> FractureVoronoi(JPH::PhysicsSystem& physics, JPH::ObjectLayer layer,
                                         const Vec3& boxMin, const Vec3& boxMax,
                                         const std::vector<Vec3>& seeds, const Vec3& impactPoint,
                                         const VoronoiFractureConfig& config) {
    std::vector<JPH::BodyID> bodies;
    const std::vector<VoronoiCell> cells = ComputeVoronoiCells(boxMin, boxMax, seeds);
    bodies.reserve(cells.size());

    JPH::BodyInterface& bi = physics.GetBodyInterface();
    const JPH::Vec3 impact = ToJolt(impactPoint);

    for (const VoronoiCell& cell : cells) {
        if (cell.vertices.size() < 4) continue; // gecerli bir 3B govde icin en az 4 nokta gerekir

        const JPH::Vec3 centroid = ToJolt(cell.centroid);
        std::vector<JPH::Vec3> localPoints;
        localPoints.reserve(cell.vertices.size());
        for (const Vec3& v : cell.vertices) {
            const JPH::Vec3 p = ToJolt(v) - centroid;
            // Cok yakin/cakisik noktalari birlestir -- Jolt'un convex hull
            // olusturucusu neredeyse-dejenere nokta bulutlarinda kararsiz
            // olabiliyor (Windows'ta gercek bir crash'e yol acti).
            bool duplicate = false;
            for (const JPH::Vec3& q : localPoints) {
                if ((p - q).LengthSq() < 1.0e-8f) {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate) localPoints.push_back(p);
        }
        if (localPoints.size() < 4) continue;

        // Dejenere (duzlemsel/cizgisel/nokta-benzeri) hucreleri atla:
        // bounding box'in her ekseninde asgari bir genislik olmali.
        JPH::Vec3 bmin = localPoints[0], bmax = localPoints[0];
        for (const JPH::Vec3& p : localPoints) {
            bmin = JPH::Vec3::sMin(bmin, p);
            bmax = JPH::Vec3::sMax(bmax, p);
        }
        const JPH::Vec3 extent = bmax - bmin;
        if (extent.GetX() < 1.0e-4f || extent.GetY() < 1.0e-4f || extent.GetZ() < 1.0e-4f) continue;

        JPH::ConvexHullShapeSettings shapeSettings(localPoints.data(),
                                                   static_cast<int>(localPoints.size()));
        shapeSettings.SetDensity(config.density);
        shapeSettings.SetEmbedded();
        const JPH::ShapeSettings::ShapeResult shapeResult = shapeSettings.Create();
        if (shapeResult.HasError()) continue; // dejenere (duzlemsel/cok kucuk) hucre -- atla

        JPH::BodyCreationSettings bodySettings(shapeResult.Get(), JPH::RVec3(centroid),
                                               JPH::Quat::sIdentity(), JPH::EMotionType::Dynamic,
                                               layer);
        bodySettings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
        bodySettings.mUserData = config.material;

        const JPH::BodyID id = bi.CreateAndAddBody(bodySettings, JPH::EActivation::Activate);
        if (id.IsInvalid()) continue;

        JPH::Vec3 outward = centroid - impact;
        const float dist = outward.Length();
        outward = dist > 1.0e-4f ? outward / dist : JPH::Vec3::sAxisY();
        const float falloff = 1.0f / (1.0f + dist);
        bi.AddImpulse(id, outward * (config.impactImpulseN * falloff));

        bodies.push_back(id);
    }

    return bodies;
}

} // namespace alazforge
