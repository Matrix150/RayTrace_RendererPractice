#include "objects.h"

// Solve quadratic equation
static inline bool SolveQuadratic(float a, float b, float c, float& t0, float& t1)
{
	float disc = b * b - 4 * a * c;
	if (disc < 0)
		return false;	// no solution (no intersection)

	float sqrtDisc = std::sqrt(disc);
	/*
	t0 = (-b - sqrtDisc) / (2.0f * a);
	t1 = (-b + sqrtDisc) / (2.0f * a);	// normal method
	*/
	// Kahan's method for quadratic roots
	float q = (b > 0) ? -0.5f * (b + sqrtDisc) : -0.5f * (b - sqrtDisc);
	t0 = q / a;
	t1 = c / q;
	if (t0 > t1)
		std::swap(t0, t1);	// t0 is front hit, t1 is back hit
	return true;
}


// Return HitInfo when ray intersect sphere with best z-buffer
bool Sphere::IntersectRay(Ray const& ray, HitInfo& hInfo, int hitSide) const
{
	// Unit ball (origin & radius = 1)
	const float a = ray.dir % ray.dir;
	const float b = 2.0f * (ray.p % ray.dir);
	const float c = (ray.p % ray.p) - 1.0f;

	float t0, t1;	// solution of the quadratic equation
	if (!SolveQuadratic(a, b, c, t0, t1))
		return false;	// no intersection
	float t = (t0 > 0.0f) ? t0 : ((t1 > 0.0f) ? t1 : -1.0f);
	if (t <= 0 || t >= hInfo.z)
		return false;	// sphere behind the camera or hitpoint is not the closest

	Vec3f p = ray.p + ray.dir * t;
	Vec3f N = p.GetNormalized();
	const bool front = ((N % ray.dir) < 0.0f);

	if ((hitSide == HIT_FRONT && !front) || (hitSide == HIT_BACK && front))
		return false;	// Check whether hit the allowed side

	// Write in HitInfo
	hInfo.Init();
	hInfo.p = p;
	hInfo.z = t;
	hInfo.N = N;
	hInfo.front = front;

	return true;
}