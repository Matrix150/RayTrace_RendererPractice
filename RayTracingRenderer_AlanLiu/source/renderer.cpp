#include <thread>
#include <algorithm>
#include "renderer.h"
#include "lights.h"


// Transform from Color(0, 1) to Color24 (0, 255)
inline unsigned char ToByte(float x)
{
	x = x * 255.0f;
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



// ShadeInfo class
class MyShadeInfo : public ShadeInfo
{
public:
	MyShadeInfo(const Renderer* renderer, const std::vector<Light*>& lights) : ShadeInfo(lights), rendererPtr(renderer) {}

	float TraceShadowRay(const Ray& ray, float t_max = BIGFLOAT) const override
	{
		return rendererPtr->TraceShadowRay(ray, t_max, HIT_FRONT_AND_BACK) ? 0.0f : 1.0f;		// hit = 0.0f, not hit = 1.0f
	}

private:
	const Renderer* rendererPtr;
};


// Renderer class
class MyRenderer : public Renderer
{
public:
	void BeginRender() override;
	void StopRender() override;
	bool TraceRay(Ray const& ray, HitInfo& hInfo, int hitSide) const override;
	bool TraceShadowRay(Ray const& ray, float t_max, int hitSide) const override;

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

	// clear screen
	for (int i = 0; i < pixelNum; ++i)
	{
		zBuffer[i] = BIGFLOAT;
		pixels[i] = Color24(0, 0, 0);
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

	for (unsigned t = 0; t < T; ++t)
	{
		workers.emplace_back([=]()
			{
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
						const float pixelY = 1.0f - ((y + 0.5f) / float(height)) * 2.0f;
						for (int x = x0; x < x1 && !cancel.load(std::memory_order_relaxed); ++x)
						{
							const float pixelX = ((x + 0.5f) / float(width)) * 2.0f - 1.0f;

							Vec3f dir = (pixelX * aspect * tanHalf) * camRight + (pixelY * tanHalf) * camUp + camDir;
							dir.Normalize();

							Ray ray(camera.pos, dir);
							HitInfo hInfo;
							hInfo.Init();
							bool hit = this->TraceRay(ray, hInfo, HIT_FRONT_AND_BACK);

							const int index = y * width + x;

							// Fill rendering color
							if (hit)
							{
								zBuffer[index] = hInfo.z;
								//pixels[index] = Color24(255, 255, 255);		// White for hit
								// Get material
								const Material* material = (hInfo.node ? hInfo.node->GetMaterial() : nullptr);

								MyShadeInfo shadeInfo(this, scene.lights);
								shadeInfo.SetPixel(x, y);

								// shade hitInfo
								shadeInfo.SetHit(ray, hInfo);
								zBuffer[index] = hInfo.z;
								// Get shade function in materials.cpp
								Color color(1.0f, 1.0f, 1.0f);

								if (material)
								{
									color = material->Shade(shadeInfo);
									pixels[index] = Color24(ToByte(color.r), ToByte(color.g), ToByte(color.b));
								}
							}
							else
							{
								zBuffer[index] = BIGFLOAT;
								pixels[index] = Color24(0, 0, 0);		// Black for not hit
							}
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