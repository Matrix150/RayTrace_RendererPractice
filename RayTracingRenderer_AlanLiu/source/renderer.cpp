#include <thread>
#include <algorithm>
#include "renderer.h"
#include "lights.h"
#include "rng.h"


// Transform from Color(0, 1) to Color24 (0, 255)
inline unsigned char ToByte(float x, bool applyGamma)
{
	if (x < 0.0f)
		x = 0.0f;
	else if (x > 1.0f)
		x = 1.0f;
	// Gamma corrent
	if (applyGamma)
	{
		const float invGamma = 1.0f / 2.2f;
		x = std::pow(x, invGamma);
	}
	x *= 255.0f;
	if (x < 0.0f) 
		x = 0.0f;
	else if (x > 255.0f) 
		x = 255.0f;
	return static_cast<unsigned char>(x + 0.5f);
}


// Check Whether ray hits node or not
static bool IntersectNodeRecursive(const Node& node, const Ray& ray, HitInfo& hInfo, int hitSide)
{
	bool hit = false;
	// Transform ray from wolrd to node object
	Ray rayLocal = node.ToNodeCoords(ray);
	// Get object in currenet node
	const Object* obj = node.GetNodeObj();
	HitInfo hCurrent;
	hCurrent.Init();
	if (obj && obj->IntersectRay(rayLocal, hCurrent, hitSide))
	{
		if (hCurrent.z < hInfo.z)
		{
			hCurrent.node = &node;
			hInfo = hCurrent;
			hit = true;
		}
	}

	// Recursive child node
	for (int i = 0; i < node.GetNumChild(); ++i)
	{
		if (IntersectNodeRecursive(*node.GetChild(i), rayLocal, hCurrent, hitSide))
		{
			if (hCurrent.z < hInfo.z)
			{
				hInfo = hCurrent;
				hit = true;
			}
		}
	}

	if (hit)
		node.FromNodeCoords(hInfo);

	return hit;
}


// Check whether need to cast or not
static bool CastShadowRecursive(const Node& node, const Ray& ray, float t_max, int hitSide, float eps)
{
	Ray rayLocal = node.ToNodeCoords(ray);
	const Object* obj = node.GetNodeObj();
	HitInfo hCurrent;
	hCurrent.Init();

	if (obj && obj->IntersectRay(rayLocal, hCurrent, hitSide))
	{
		if (hCurrent.z > eps && hCurrent.z < (t_max - eps))
			return true;
	}

	for (int i = 0; i < node.GetNumChild(); ++i)
	{
		if (CastShadowRecursive(*node.GetChild(i), rayLocal, t_max, hitSide, eps))
			return true;
	}

	return false;
}



//-------------------------------------------------------------------------------
// Renderer

// Construct camera basis vectors 
static inline void ConstructCameraBasis(const Camera& camera, Vec3f& camRight, Vec3f& camUp, Vec3f& camDir)
{
	camDir = camera.dir.GetNormalized();		// Viewing direction (forward)
	camRight = camDir ^ camera.up;
	camRight.Normalize();		//Right
	camUp = camera.up.GetNormalized();		// Up
}

//
static inline void ConcentricSampleDisk(float u1, float u2, float& dx, float& dy)
{
	// [0, 1) -> [-1, 1)
	float sx = 2.0f * u1 - 1.0f;
	float sy = 2.0f * u2 - 1.0f;

	if (sx == 0.0f && sy == 0.0f)
	{
		dx = dy = 0.0f;
		return;
	}

	float r, theta;
	if (std::abs(sx) > std::abs(sy))
	{
		r = sx;
		theta = float(Pi<float>() / 4.0f) * (sy / sx);
	}
	else
	{
		r = sy;
		theta = float(Pi<float>() / 2.0f) - float(Pi<float>() / 4.0f) * (sx / sy);
	}

	dx = r * std::cos(theta);
	dy = r * std::sin(theta);
}



//class Myrenderer;

// ShadeInfo class
class MyShadeInfo : public ShadeInfo
{
public:
	MyShadeInfo(const Renderer* renderer, const std::vector<Light*>& lights, const TexturedColor& environment, RNG& rng, int maxBounce) : ShadeInfo(lights, environment, rng), rendererPtr(renderer), maxSpecularBounce(maxBounce) {}

	float TraceShadowRay(const Ray& ray, float t_max = BIGFLOAT) const override;

	bool CanBounce() const override
	{
		return CurrentBounce() < maxSpecularBounce;
	}

	Color TraceSecondaryRay(const Ray& ray, float& dist, bool reflection = true) const override;

	void IncrementBounce() 
	{ 
		++bounce; 
	}

private:
	const Renderer* rendererPtr = nullptr;
	int maxSpecularBounce = 5;		// Default bounces
};



float MyShadeInfo::TraceShadowRay(const Ray& ray, float t_max) const
{
	constexpr float kEps = 1e-4f;
	Ray r = ray;
	Vec3f dir = r.dir.GetNormalized();
	r.p += dir * kEps;

	return rendererPtr->TraceShadowRay(r, t_max, HIT_FRONT_AND_BACK) ? 0.0f : 1.0f;		// 0.0f if hit, 1.0f if not hit
}

Color MyShadeInfo::TraceSecondaryRay(const Ray& ray, float& dist, bool reflection) const
{
	dist = BIGFLOAT;

	constexpr float kEps = 1e-4f;
	Vec3f dir = ray.dir.GetNormalized();
	Ray raySencondary(ray.p + dir * kEps, dir);

	// Check if can bounce
	if (!CanBounce())
		return Color(0.0f, 0.0f, 0.0f);

	HitInfo hInfo;
	hInfo.Init();

	// Secondary ray not hit
	if (!rendererPtr->TraceRay(raySencondary, hInfo, HIT_FRONT_AND_BACK))
		return env.EvalEnvironment(dir);

	//MyShadeInfo* self = const_cast<MyShadeInfo*>(this);
	//self->IncrementBounce();
	//self->SetHit(raySencondary, hInfo);
	dist = hInfo.front ? hInfo.z : 0.0f;

	const Material* material = (hInfo.node ? hInfo.node->GetMaterial() : nullptr);
	if (material)
	{
		MyShadeInfo child(*this);
		child.IncrementBounce();
		child.SetHit(raySencondary, hInfo);
		return material->Shade(child);
	}

	/*if (hInfo.node)
		if (const Light* light = dynamic_cast<const Light*>(hInfo.node))
			return light->Radiance(*self);*/
	
	return env.EvalEnvironment(dir);
}



// Renderer class
class MyRenderer : public Renderer
{
public:
	void BeginRender() override;
	void StopRender() override;
	bool TraceRay(Ray const& ray, HitInfo& hInfo, int hitSide) const override;
	bool TraceShadowRay(Ray const& ray, float t_max, int hitSide) const override;

	friend class MyShadeInfo;

private:
	// threading state
	std::vector<std::thread> workers;		// working threads
	std::atomic<bool> cancel{ false };		// cancel mark
	std::atomic<int> nextTile{ 0 };			// next tile index 
	std::atomic<int> liveWorkers{ 0 };		// living threads number
};



bool MyRenderer::TraceRay(Ray const& ray, HitInfo& hInfo, int hitSide) const
{
	bool hit = IntersectNodeRecursive(scene.rootNode, ray, hInfo, hitSide);		// Call recursive function to get intersection

	for (Light* light : scene.lights)
	{
		auto* pointlight = dynamic_cast<PointLight*>(light);
		if (!pointlight || !pointlight->IsRenderable())
			continue;

		HitInfo hLight;
		hLight.Init();

		if (pointlight->IntersectRay(ray, hLight, hitSide))
		{
			if (!hit || hLight.z < hInfo.z)
			{
				hInfo = hLight;
				hInfo.node = nullptr;
				hit = true;
			}
		}
	}

	return hit;
}


bool MyRenderer::TraceShadowRay(Ray const& ray, float t_max, int hitSide) const
{
	// Offset avoid self-shadowing
	constexpr float kEps = 1e-4f;
	Ray rayShadow = ray;
	Vec3f dir = ray.dir;
	dir.Normalize();
	rayShadow.p += dir * kEps;

	// Recursive to test occlusion
	return CastShadowRecursive(scene.rootNode, rayShadow, t_max, hitSide, kEps);
}


void MyRenderer::BeginRender()
{
	if (isRendering)
		return;
	isRendering = true;

	// Prepare render target
	renderImage.Init(camera.imgWidth, camera.imgHeight);	// Initialize z-buffer to BIGFLOAT & reset rendered pixels
	const int width = renderImage.GetWidth();
	const int height = renderImage.GetHeight();
	const int pixelNum = width * height;
	Color24* const pixels = renderImage.GetPixels();
	float* const zBuffer = renderImage.GetZBuffer();
	int* const sampleCount = renderImage.GetSampleCount();

	// clear screen
	for (int i = 0; i < pixelNum; ++i)
	{
		zBuffer[i] = BIGFLOAT;
		pixels[i] = Color24(0, 0, 0);
		if (sampleCount)
			sampleCount[i] = 0;
	}

	// Camera basis vectors
	Vec3f camRight, camUp, camDir;
	ConstructCameraBasis(camera, camRight, camUp, camDir);

	// Set parameters of view volume
	const float aspect = float(width) / float(height);
	const float fov = Deg2Rad(camera.fov);
	const float tanHalf = std::tan(fov * 0.5f);

	// Dynamic Tile Scheduling
	constexpr int TILE = 16;		// Tile length
	const int tileX = (width + TILE - 1) / TILE;
	const int tileY = (height + TILE - 1) / TILE;
	const int tileNum = tileX * tileY;

	// Start backend thread
	cancel.store(false, std::memory_order_relaxed);
	nextTile.store(0, std::memory_order_relaxed);

	const unsigned T = std::max(1u, std::thread::hardware_concurrency());
	liveWorkers.store((int)T, std::memory_order_relaxed);
	workers.clear();
	workers.reserve(T);

	constexpr int maxSpecularBounce = 3;		// Set max bounce number

	constexpr int sppMin = 4;		// sample per pixel
	constexpr int sppMax = 64;
	constexpr float deltaTolerance = 0.007f;
	// Halton for pixel sample
	HaltonSeq<sppMax> haltonX(2);
	HaltonSeq<sppMax> haltonY(3);
	// Halton for circle len smaple
	HaltonSeq<sppMax> haltonLenU(5);
	HaltonSeq<sppMax> haltonLenV(7);


	// using T-test table
	auto Ttest_approx = [](int n) -> float
		{
			int nMinusOne = std::max(1, n - 1);
			if (nMinusOne == 1) return 63.657f;
			if (nMinusOne == 2) return 9.925f;
			if (nMinusOne == 3) return 5.841f;
			if (nMinusOne == 4) return 4.604f;
			if (nMinusOne == 5) return 4.032f;
			if (nMinusOne == 6) return 3.707f;
			if (nMinusOne == 7) return 3.499f;
			if (nMinusOne == 8) return 3.355f;
			if (nMinusOne == 9) return 3.250f;
			if (nMinusOne == 10) return 3.169f;
			if (nMinusOne == 11) return 3.106f;
			if (nMinusOne == 12) return 3.055f;
			if (nMinusOne == 13) return 3.012f;
			if (nMinusOne == 14) return 2.977f;
			if (nMinusOne == 15) return 2.947f;
			if (nMinusOne == 16) return 2.921f;
			if (nMinusOne == 17) return 2.898f;
			if (nMinusOne == 18) return 2.878f;
			if (nMinusOne == 19) return 2.861f;
			if (nMinusOne == 20) return 2.845f;
			if (nMinusOne == 21) return 2.831f;
			if (nMinusOne == 22) return 2.819f;
			if (nMinusOne == 23) return 2.807f;
			if (nMinusOne == 24) return 2.797f;
			if (nMinusOne == 25) return 2.787f;
			if (nMinusOne == 26) return 2.779f;
			if (nMinusOne == 27) return 2.771f;
			if (nMinusOne == 28) return 2.763f;
			if (nMinusOne == 29) return 2.756f;
			if (nMinusOne == 30) return 2.750f;
			if (nMinusOne == 31) return 2.744f;
			if (nMinusOne == 32) return 2.738f;
			if (nMinusOne == 33) return 2.733f;
			if (nMinusOne == 34) return 2.728f;
			if (nMinusOne == 35) return 2.724f;
			if (nMinusOne == 36) return 2.719f;
			if (nMinusOne == 37) return 2.715f;
			if (nMinusOne == 38) return 2.712f;
			if (nMinusOne == 39) return 2.708f;
			if (nMinusOne == 40) return 2.704f;
			if (nMinusOne == 41) return 2.701f;
			if (nMinusOne == 42) return 2.698f;
			if (nMinusOne == 43) return 2.695f;
			if (nMinusOne == 44) return 2.692f;
			if (nMinusOne == 45) return 2.690f;
			if (nMinusOne == 46) return 2.687f;
			if (nMinusOne == 47) return 2.685f;
			if (nMinusOne == 48) return 2.682f;
			if (nMinusOne == 49) return 2.680f;
			if (nMinusOne == 50) return 2.678f;
			if (nMinusOne <= 60) return 2.660f;
			if (nMinusOne <= 70) return 2.648f;
			if (nMinusOne <= 80) return 2.639f;
			if (nMinusOne <= 90) return 2.632f;
			if (nMinusOne <= 100) return 2.626f;
			if (nMinusOne <= 120) return 2.617f;
			if (nMinusOne <= 140) return 2.611f;
			if (nMinusOne <= 180) return 2.603f;
			if (nMinusOne <= 200) return 2.601f;
			if (nMinusOne <= 500) return 2.586f;
			if (nMinusOne <= 1000) return 2.581f;
			return 2.576f;
		};

	for (unsigned t = 0; t < T; ++t)
	{
		workers.emplace_back([=]()
			{
				RNG rng;
				int claimed = 0;		// Pixels this thread already finished
				for (;;)
				{
					if (cancel.load(std::memory_order_relaxed))
						break;

					int tileID = nextTile.fetch_add(1, std::memory_order_relaxed);
					if (tileID >= tileNum)
						break;

					const int ty = tileID / tileX;
					const int tx = tileID % tileX;
					const int x0 = tx * TILE, x1 = std::min(x0 + TILE, width);
					const int y0 = ty * TILE, y1 = std::min(y0 + TILE, height);

					for (int y = y0; y < y1 && !cancel.load(std::memory_order_relaxed); ++y)
					{
						for (int x = x0; x < x1 && !cancel.load(std::memory_order_relaxed); ++x)
						{
							const int index = y * width + x;

							const float shiftX = rng.RandomFloat();
							const float shiftY = rng.RandomFloat();
							const float shiftLenU = rng.RandomFloat();
							const float shiftLenV = rng.RandomFloat();

							Color sampleTotal(0.0f, 0.0f, 0.0f);		// S1
							Color sampleTotalSquare(0.0f, 0.0f, 0.0f);		// S2
							int n = 0;
							float zBest = BIGFLOAT;

							auto take_one_sample = [&](int s)
								{
									// makes shiftx and shifty in [0.0f, 1.0f)
									// for pixel (using halton base (2, 3))
									float sx = std::fmod(haltonX[s] + shiftX, 1.0f);
									float sy = std::fmod(haltonY[s] + shiftY, 1.0f);
									// for lens (using halton base (5, 7))
									float lu = std::fmod(haltonLenU[s] + shiftLenU, 1.0f);
									float lv = std::fmod(haltonLenV[s] + shiftLenV, 1.0f);

									// makes range [0.0f, 1.0f) to [-1.0f, 1.0f)
									float px = ((x + sx) / (float)width) * 2.0f - 1.0f;
									float py = 1.0f - ((y + sy) / (float)height) * 2.0f;

									Vec3f dirPin = (px * aspect * tanHalf) * camRight + (py * tanHalf) * camUp + camDir;
									dirPin.Normalize();
									Ray ray;

									// Lens with depth of field
									if (camera.dof > 0.0f && camera.focaldist > 0.0f)
									{
										float cosTheta = dirPin % camDir;	// dot
										float tFocus = (cosTheta != 0.0f) ? (camera.focaldist / cosTheta) : camera.focaldist;
										Vec3f PFocus = camera.pos + tFocus * dirPin;

										// dick sample (lu ,lv) to (dx, dy)
										float dx, dy;
										ConcentricSampleDisk(lu, lv, dx, dy);

										float lenRadius = 0.5f * camera.dof;
										Vec3f PLen = camera.pos + (dx * lenRadius) * camRight + (dy * lenRadius) * camUp;
										Vec3f dir = (PFocus - PLen).GetNormalized();
										ray = Ray(PLen, dir);
									}
									else
									{
										// pin hole
										ray = Ray(camera.pos, dirPin);
									}

									HitInfo hInfo;
									hInfo.Init();

									Color sampleColor(0.0f, 0.0f, 0.0f);
									bool hit = this->TraceRay(ray, hInfo, HIT_FRONT_AND_BACK);

									if (hit)
									{
										if (hInfo.z < zBest)
											zBest = hInfo.z;
										const Material* material = (hInfo.node ? hInfo.node->GetMaterial() : nullptr);
										if (material)
										{
											MyShadeInfo sInfo(this, scene.lights, scene.environment, rng, maxSpecularBounce);
											sInfo.SetPixel(x, y);
											sInfo.SetPixelSample(s);
											sInfo.SetHit(ray, hInfo);
											sampleColor = material->Shade(sInfo);
										}
										else
										{
											sampleColor = Color(1.0f, 1.0f, 1.0f);
										}
									}
									else
									{
										float u = (x + sx) / (float)width;
										float v = (y + sy) / (float)height;
										sampleColor = scene.background.Eval(Vec3f(u, v, 0.0f));
									}
									sampleTotal += sampleColor;
									sampleTotalSquare += Color(sampleColor.r * sampleColor.r, sampleColor.g * sampleColor.g, sampleColor.b * sampleColor.b);
									++n;
								};

							auto need_more_sample = [&]() -> bool
								{
									if (n < sppMin)
										return true;
									Color mean = sampleTotal / (float)n;

									auto variance_per_channel = [&](float sampleSum, float sampleSumSquare) -> float
										{
											float variance = (sampleSumSquare - (sampleSum * sampleSum) / (float)n);
											return (n > 1) ? std::max(0.0f, variance / (float)(n - 1)) : 0.0f;
										};

									float varianceR = variance_per_channel(sampleTotal.r, sampleTotalSquare.r);
									float varianceG = variance_per_channel(sampleTotal.g, sampleTotalSquare.g);
									float varianceB = variance_per_channel(sampleTotal.b, sampleTotalSquare.b);

									float t = Ttest_approx(n);
									auto delta_pass = [&](float variance) -> bool
										{
											float sigma = std::sqrt(variance);		// standard deviation
											float delta = t * (sigma / std::sqrt((float)n));
											return delta <= deltaTolerance;
										};

									return !(delta_pass(varianceR) && delta_pass(varianceG) && delta_pass(varianceB));
								};

							for (int s = 0; s < sppMin; ++s)
								take_one_sample(s);

							int s = sppMin;
							while (s < sppMax && need_more_sample())
							{
								take_one_sample(s);
								++s;
							}

							Color color = sampleTotal / (float)n;
							//color.Clamp();
							bool applyGamma = camera.sRGB;
							pixels[index] = Color24(ToByte(color.r, applyGamma), ToByte(color.g, applyGamma), ToByte(color.b, applyGamma));
							zBuffer[index] = zBest;
							if (sampleCount)
								sampleCount[index] = n;

							++claimed;
						}
					}
				}
				// Merge progress per thread
				this->renderImage.IncrementNumRenderPixel(claimed);
				if (liveWorkers.fetch_sub(1, std::memory_order_acq_rel) == 1)
				{
					if (!cancel.load(std::memory_order_acquire))
						this->renderImage.ComputeZBufferImage();
				}
			});
	}
}


void MyRenderer::StopRender()
{
	// Cancel signal
	cancel.store(true, std::memory_order_relaxed);

	// Recycle working threads
	for (auto& thread : workers)
	{
		if (thread.joinable())
			thread.join();
	}
	workers.clear();

	if (isRendering)
	{
		renderImage.ComputeZBufferImage();
		isRendering = false;
	}
}


Renderer* CreateRenderer()
{
	return new MyRenderer();
}