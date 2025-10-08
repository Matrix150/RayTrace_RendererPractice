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
	Vec3f N = p;
	const bool front = ((N % ray.dir) < 0.0f);
	N.Normalize();

	if ((hitSide == HIT_FRONT && !front) || (hitSide == HIT_BACK && front))
		return false;	// Check whether hit the allowed side

	// Write in HitInfo
	//hInfo.Init();
	hInfo.p = p;
	hInfo.z = t;
	hInfo.N = N;
	hInfo.GN = N;
	hInfo.front = front;

	return true;
}


bool Plane::IntersectRay(Ray const& ray, HitInfo& hInfo, int hitSide) const
{
	// Unit plane (z = 0, x,y in [-1,1], normal = (0,0,1))
	const float eps = 1e-8f;
	const Vec3f N(0, 0, 1);
	const float denom = ray.dir % N;
	if (std::abs(denom) < eps)
		return false;		// ray parallel with plane

	const float t = -(ray.p % N) / denom;
	if (t <= 0 || t >= hInfo.z)
		return false;

	const Vec3f p = ray.p + ray.dir * t;
	if (p.x < -1.0f || p.x > 1.0f || p.y < -1.0f || p.y > 1.0f)
		return false;

	const bool front = (N % ray.dir) < 0.0f;
	if ((hitSide == HIT_FRONT && !front) || (hitSide == HIT_BACK && front))
		return false;

	hInfo.p = p;
	hInfo.z = t;
	hInfo.N = N;
	hInfo.GN = N;
	hInfo.front = front;

	return true;
}


bool TriObj::IntersectRay(Ray const& ray, HitInfo& hInfo, int hitSide) const
{
	bool hit = false;
	HitInfo hBest = hInfo;
	const int faceNum = NF();

	for (int i = 0; i < faceNum; ++i)
	{
		HitInfo hCurrent = hBest;
		if (IntersectTriangle(ray, hCurrent, hitSide, (unsigned)i))
			if (hCurrent.z < hBest.z)
			{
				hBest = hCurrent;
				hit = true;
			}
	}

	if (hit)
	{
		hInfo = hBest;
		return true;
	}

	return false;
}


static inline bool MollerTrumbore(Ray const& ray, const Vec3f& v0, const Vec3f& v1, const Vec3f& v2, float& t, float& u, float& v)
{
	const float eps = 1e-8f;
	const Vec3f e1 = v1 - v0;
	const Vec3f e2 = v2 - v0;

	const Vec3f s1 = ray.dir ^ e2;		// cross product of ray direction and edge2
	const float det = e1 % s1;

	if (std::abs(det) < eps)
		return false;		// ray parallel with triangle

	const float invDet = 1.0f / det;
	const Vec3f s = ray.p - v0;		// vector from v0 ro ray origin
	u = (s % s1) * invDet;
	if (u < 0.0f || u > 1.0f)
		return false;

	const Vec3f s2 = s ^ e1;
	v = (ray.dir % s2) * invDet;
	if (v < 0.0f || (u + v) > 1.0f)
		return false;

	t = (e2 % s2) * invDet;
	return (t > 0.0f);
}


bool TriObj::IntersectTriangle(Ray const& ray, HitInfo& hInfo, int hitSide, unsigned int faceID) const
{
	const TriFace& f = F((int)faceID);
	const Vec3f v0 = this->V(f.v[0]);
	const Vec3f v1 = this->V(f.v[1]);
	const Vec3f v2 = this->V(f.v[2]);

	float t, u, v;
	if (!MollerTrumbore(ray, v0, v1, v2, t, u, v))
		return false;
	if (t >= hInfo.z)
		return false;

	Vec3f GN = (v1 - v0) ^ (v2 - v0);		// e1 ^ e2
	const bool front = (GN % ray.dir) < 0.0f;
	if ((hitSide == HIT_FRONT && !front) || (hitSide == HIT_BACK && front))
		return false;

	Vec3f N = GN;		// flat normal
	if (HasNormals())
	{
		Vec3f bc(1.0f - u - v, u, v);		// barycentric coordinate
		N = GetNormal((int)faceID, bc);
	}

	GN.Normalize();
	N.Normalize();
	const Vec3f p = ray.p + ray.dir * t;

	hInfo.p = p;
	hInfo.z = t;
	hInfo.N = N;
	hInfo.GN = GN;
	hInfo.front = front;

	return true;
}