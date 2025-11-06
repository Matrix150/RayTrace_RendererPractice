#include "lights.h"

// Point Light
Color PointLight::Illuminate(ShadeInfo const& sInfo, Vec3f& L) const
{
    float dist = (position - sInfo.P()).Length();
    if (dist <= 0.0f)
    {
        L = Vec3f(0.0f, 0.0f, 0.0f);
        return Color(0.0f, 0.0f, 0.0f);
    }
    L = (position - sInfo.P()).GetNormalized();

    Vec3f N = (std::fabs(L.z) < 0.999f) ? Vec3f(0.0f, 0.0f, 1.0f) : Vec3f(0.0f, 1.0f, 0.0f);
    Vec3f T = (N ^ L).GetNormalized();
    Vec3f B = (L ^ T).GetNormalized();

    auto RaySphereIntersect = [](const Vec3f& P, const Vec3f& dRay, const Vec3f& center, float r)->float
        {
            // a = 1.0f
            float b = 2.0f * ((P - center) % dRay);
            float c = ((P - center) % (P - center)) - (r * r);
            float disc = b * b - 4.0f * c;
            if (disc < 0.0f)
                return -1.0f;

            float sqrtDisc = std::sqrt(disc);
            float q = (b > 0) ? -0.5f * (b + sqrtDisc) : -0.5f * (b - sqrtDisc);
            float t0 = q;
            float t1 = c / q;
            if (t0 > t1)
                std::swap(t0, t1);
            return (t0 > 0.0f) ? t0 : ((t1 > 0.0f) ? t1 : -1.0f);
        };

    const float r = size;
    const bool nearTouch = ((dist * dist) <= ((r * r) * 1.0001f));

    constexpr int kBaseSample = 8;
    constexpr int kMaxSample = 128;
    constexpr int HaltonCutoff = 7;
    int sampleTarget = kBaseSample;
    int currentSample = 0;
    int shadowCount = 0;

    const float shiftX = sInfo.RandomFloat();
    const float shiftY = sInfo.RandomFloat();
    auto frac = [](float x) {return x - std::floor(x); };

    auto isOccluded = [&](const Vec3f& worldPoint)->bool
        {
            float tMax = (worldPoint - sInfo.P()).Length();
            if (tMax <= 0.0f)
                return false;
            Vec3f dir = (worldPoint - sInfo.P()).GetNormalized();

            const float eps = std::max(1e-3f * tMax, 1e-4f);
            tMax = std::max(0.0f, tMax - eps);

            return sInfo.TraceShadowRay(Ray(sInfo.P(), dir), tMax) == 0.0f;
        };

    while (currentSample < sampleTarget)
    {
        int index = currentSample + HaltonCutoff;
        float u = frac(Halton(index + 1, 2) + shiftX);
        float v = frac(Halton(index + 1, 3) + shiftY);

        Vec3f samplePos;

        if (!nearTouch)
        {
            float rho = (dist * r) / std::sqrt(std::max((dist * dist) - (r * r), 1e-12f));
            float rr = rho * std::sqrt(u);
            float phi = 2.0f * Pi<float>() * v;
            Vec3f Splane = position + T * (rr * std::cos(phi)) + B * (rr * std::sin(phi));

            Vec3f dRay = (Splane - sInfo.P()).GetNormalized();
            float tHit = RaySphereIntersect(sInfo.P(), dRay, position, r);
            if (tHit > 0.0f)
                samplePos = sInfo.P() + dRay * tHit;
            else
                samplePos = position;
        }
        else
        {
            float u1 = u, u2 = v;
            float z = 1.0f - 2.0f * u1;
            float t = std::sqrt(std::max(0.0f, (1.0f - z * z)));
            float ph = 2.0f * Pi<float>() * u2;
            Vec3f onUnit = Vec3f(t * std::cos(ph), t * std::sin(ph), z);
            samplePos = position + onUnit * r;
        }

        if (isOccluded(samplePos))
            ++shadowCount;
        ++currentSample;

        if (sampleTarget == kBaseSample && currentSample >= kBaseSample)
        {
            if (shadowCount != 0 && shadowCount != currentSample)
                sampleTarget = kMaxSample;
        }
    }

    float visibility = 1.0f - (float)shadowCount / (float)currentSample;
    return visibility * Radiance(sInfo);
}

bool PointLight::IntersectRay(Ray const& ray, HitInfo& hInfo, int hitSide) const
{
    // Sphere center & radius
    const Vec3f center = position;
    const float r = size;

    const float a = ray.dir % ray.dir;
    const float b = 2.0f * ((ray.p - center) % ray.dir);
    const float c = ((ray.p - center) % (ray.p - center)) - (r * r);

    float disc = b * b - 4.0f * a * c;
    if (disc < 0.0f)
        return false;

    float sqrtDisc = std::sqrt(disc);
    float q = (b > 0) ? -0.5f * (b + sqrtDisc) : -0.5f * (b - sqrtDisc);
    float t0 = q / a;
    float t1 = c / q;
    if (t0 > t1)
        std::swap(t0, t1);

    float t = (t0 > 0.0f) ? t0 : ((t1 > 0.0f) ? t1 : -1.0f);
    if (t <= 0 || t >= hInfo.z)
        return false;

    const Vec3f p = ray.p + ray.dir * t;
    Vec3f N = p - center;
    const bool front = ((N % ray.dir) < 0.0f);
    N.Normalize();

    if ((hitSide == HIT_FRONT && !front) || (hitSide == HIT_BACK && front))
        return false;

    hInfo.p = p;
    hInfo.z = t;
    hInfo.N = N;
    hInfo.GN = N;
    hInfo.front = front;

    hInfo.uvw.Set(0.0f, 0.0f, 0.0f);

    return true;
}
