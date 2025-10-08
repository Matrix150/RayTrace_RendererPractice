//-------------------------------------------------------------------------------
///
/// \file       renderer.h 
/// \author     Cem Yuksel (www.cemyuksel.com)
/// \version    5.0
/// \date       September 24, 2025
///
/// \brief Project source for CS 6620 - University of Utah.
///
/// Copyright (c) 2025 Cem Yuksel. All Rights Reserved.
///
/// This code is provided for educational use only. Redistribution, sharing, or 
/// sublicensing of this code or its derivatives is strictly prohibited.
///
//-------------------------------------------------------------------------------

#ifndef _RENDERER_H_INCLUDED_
#define _RENDERER_H_INCLUDED_

//-------------------------------------------------------------------------------

#include "scene.h"

#include "lodepng.h"

//-------------------------------------------------------------------------------

class RenderImage
{
private:
	std::vector<Color24> img;
	std::vector<float>   zbuffer;
	std::vector<uint8_t> zbufferImg;
	int                  width=0, height=0;
	std::atomic<int>     numRenderedPixels=0;
public:
	void Init(int w, int h)
	{
		width = w;
		height = h;
		int size = width * height;
		img.resize(size);
		zbuffer.resize(size);
		for ( int i=0; i<size; ++i ) zbuffer[i] = BIGFLOAT;
		zbufferImg.resize(size);
		ResetNumRenderedPixels();
	}

	int      GetWidth  () const { return width; }
	int      GetHeight () const { return height; }
	Color24* GetPixels ()       { return img.data(); }
	float*   GetZBuffer()       { return zbuffer.data(); }
	uint8_t* GetZBufferImage()  { return zbufferImg.data(); }

	void ResetNumRenderedPixels ()        { numRenderedPixels=0; }
	int  GetNumRenderedPixels   () const  { return numRenderedPixels; }
	bool IsRenderDone           () const  { return numRenderedPixels >= width*height; }
	void IncrementNumRenderPixel( int n ) { numRenderedPixels+=n; }

	void ComputeZBufferImage()
	{
		int size = width * height;
		float zmin=BIGFLOAT, zmax=0;
		for ( int i=0; i<size; i++ ) {
			if ( zbuffer[i] == BIGFLOAT ) continue;
			if ( zmin > zbuffer[i] ) zmin = zbuffer[i];
			if ( zmax < zbuffer[i] ) zmax = zbuffer[i];
		}
		for ( int i=0; i<size; i++ ) {
			if ( zbuffer[i] == BIGFLOAT ) zbufferImg[i] = 0;
			else {
				float f = (zmax-zbuffer[i])/(zmax-zmin);
				int c = int(f * 255);
				zbufferImg[i] = c < 0 ? 0 : ( c > 255 ? 255 : c );
			}
		}
	}

	bool SaveImage ( char const *filename ) const { return lodepng::encode(filename,&img[0].r,     width,height,LCT_RGB, 8) == 0; }
	bool SaveZImage( char const *filename ) const { return lodepng::encode(filename,&zbufferImg[0],width,height,LCT_GREY,8) == 0; }
};

//-------------------------------------------------------------------------------

class ShadeInfo
{
public:
	ShadeInfo( std::vector<Light*> const &lightList ) : lights(lightList) {}

	virtual Vec3f P () const { return hInfo.p; }	// returns the shading position
	virtual Vec3f V () const { return -ray.dir; }	// returns the view vector
	virtual Vec3f N () const { return hInfo.N; }	// returns the shading normal
	virtual Vec3f GN() const { return hInfo.GN; }	// returns the geometry normal

	virtual float Depth  () const { return hInfo.z; }		// returns the distance between the shaded hit point and the ray origin
	virtual bool  IsFront() const { return hInfo.front; }	// returns if the shading front part of the surface

	virtual Node const * GetNode() const { return hInfo.node; }	// returns the node that contains the shaded point

	virtual int X() const { return pixelX; }	// returns the current pixel's x coordinate
	virtual int Y() const { return pixelY; }	// returns the current pixel's y coordinate

	virtual int          NumLights()       const { return (int)lights.size(); }	// returns the number of lights to be used during shading
	virtual Light const* GetLight( int i ) const { return lights[i]; }			// returns the i^th light

	virtual bool CanBounce() const { return false; }	// returns if an additional bounce is permitted
	virtual int  CurrentSpecularBounce() const { return bounceS; }	// returns the current specular bounce (zero for primary rays)

	// Traces a shadow ray and returns the visibility
	virtual float TraceShadowRay( Ray   const &ray, float t_max=BIGFLOAT ) const { return 1.0f; }
	virtual float TraceShadowRay( Vec3f const &dir, float t_max=BIGFLOAT ) const { return TraceShadowRay(Ray(P(),dir),t_max); }

	// Traces a ray and returns the shaded color at the hit point.
	// It also sets t to the distance to the hit point, if a front is found.
	// if a back hit is found, dist should be set to zero.
	virtual Color TraceSecondaryRay( Ray   const &ray, float &dist ) const { dist=BIGFLOAT; return Color(0,0,0); }
	virtual Color TraceSecondaryRay( Vec3f const &dir, float &dist ) const { return TraceSecondaryRay(Ray(P(),dir),dist); }

	void SetPixel( int x, int y ) { pixelX = x; pixelY = y; }

	void SetHit( Ray const &r, HitInfo const &h )
	{
		hInfo = h;
		hInfo.z *= r.dir.Length();
		hInfo.N.Normalize();
		hInfo.GN.Normalize();
		ray = r;
		ray.dir.Normalize();
	}

	void IncrementBounce() { bounceS++; }

protected:
	Ray     ray;			// the ray that found this hit point
	HitInfo hInfo;			// ht information
	int     pixelX  = 0;	// current pixel's x coordinate
	int     pixelY  = 0;	// current pixel's y coordinate
	int     bounceS = 0;	// current specular bounce

	std::vector<Light*> const &lights;	// lights
};

//-------------------------------------------------------------------------------

class Renderer
{
protected:
	Scene       scene;
	Camera      camera;
	RenderImage renderImage;
	std::string sceneFile;
	bool isRendering = false;

public:
	Scene &             GetScene      ()       { return scene; }
	Scene const &       GetScene      () const { return scene; }
	Camera &            GetCamera     ()       { return camera; }
	Camera const &      GetCamera     () const { return camera; }
	RenderImage &       GetRenderImage()       { return renderImage; }
	RenderImage const & GetRenderImage() const { return renderImage; }

	virtual bool LoadScene( char const *sceneFilename );
	std::string const & SceneFileName() const { return sceneFile; }

	virtual void BeginRender() {}	// Generates one or more rendering threads and begins rendering. Returns immediately.
	virtual void StopRender () {}	// Stops the current rendering process. It should wait till rendering threads stop.
	bool IsRendering() const { return isRendering; }

	virtual bool TraceRay      ( Ray const &ray, HitInfo &hInfo, int hitSide=HIT_FRONT_AND_BACK ) const { return false; }
	virtual bool TraceShadowRay( Ray const &ray, float t_max,    int hitSide=HIT_FRONT_AND_BACK ) const { return false; }
};

//-------------------------------------------------------------------------------

void ShowViewport( Renderer *renderer, bool beginRendering=false );

//-------------------------------------------------------------------------------

#endif