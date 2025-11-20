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

    const float r = size;
    if (dist <= r)
        return intensity;

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

    constexpr int kBaseSample = 8;
    constexpr int kMaxSample = 64;
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

    float sinThetaMax = r / dist;
    sinThetaMax = std::min(std::max(sinThetaMax, 0.0f), 1.0f);
    float cosThetaMax = std::sqrt(std::max(0.0f, 1.0f - sinThetaMax * sinThetaMax));

    while (currentSample < sampleTarget)
    {
        int index = currentSample + HaltonCutoff;
        float u = frac(Halton(index + 1, 2) + shiftX);
        float v = frac(Halton(index + 1, 3) + shiftY);

        float cosTheta = 1.0f - u * (1.0f - cosThetaMax);
        float sinTheta = std::sqrt(std::max(0.0f, 1.0f - cosTheta * cosTheta));
        float phi = 2.0f * Pi<float>() * v;

        Vec3f dRay = T * (sinTheta * std::cos(phi)) + B * (sinTheta * std::sin(phi)) + L * cosTheta;
        float tHit = RaySphereIntersect(sInfo.P(), dRay, position, r);
        Vec3f samplePos;
        if (tHit > 0.0f)
            samplePos = sInfo.P() + dRay * tHit;
        else
            samplePos = position;

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
    Color Lout = visibility * intensity;

    if (attenuation > 0.0f)
    {
        float d = dist * attenuation;
        float invSq = 1.0f / std::max(d * d, 1e-6f);
        Lout *= invSq;
    }
    return Lout;
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