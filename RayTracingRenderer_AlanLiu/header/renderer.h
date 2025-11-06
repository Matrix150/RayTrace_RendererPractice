//-------------------------------------------------------------------------------
///
/// \file       renderer.h 
/// \author     Cem Yuksel (www.cemyuksel.com)
/// \version    10.0
/// \date       October 25, 2025
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
#include "rng.h"

#include "lodepng.h"

//-------------------------------------------------------------------------------

class RenderImage
{
private:
	std::vector<Color24> img;
	std::vector<float>   zbuffer;
	std::vector<uint8_t> zbufferImg;
	std::vector<int>     sampleCount;
	std::vector<uint8_t> sampleCountImg;
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
		sampleCount.resize(size);
		memset( sampleCount.data(), 0, size*sizeof(uint8_t) );
		sampleCountImg.resize(size);
		ResetNumRenderedPixels();
	}

	int      GetWidth  () const    { return width; }
	int      GetHeight () const    { return height; }
	Color24* GetPixels ()          { return img.data(); }
	float*   GetZBuffer()          { return zbuffer.data(); }
	uint8_t* GetZBufferImage()     { return zbufferImg.data(); }
	int*     GetSampleCount()      { return sampleCount.data(); }
	uint8_t* GetSampleCountImage() { return sampleCountImg.data(); }

	void ResetNumRenderedPixels ()        { numRenderedPixels=0; }
	int  GetNumRenderedPixels   () const  { return numRenderedPixels; }
	bool IsRenderDone           () const  { return numRenderedPixels >= width*height; }
	void IncrementNumRenderPixel( int n ) { numRenderedPixels+=n; }

	void ComputeZBufferImage() { ComputeImage<float,true>( zbufferImg, zbuffer, BIGFLOAT ); }
	int  ComputeSampleCountImage() { return ComputeImage<int,false>( sampleCountImg, sampleCount, 0 );}

	bool SaveImage           ( char const *filename ) const { return lodepng::encode(filename,&img[0].r,         width,height,LCT_RGB, 8) == 0; }
	bool SaveZImage          ( char const *filename ) const { return lodepng::encode(filename,&zbufferImg[0],    width,height,LCT_GREY,8) == 0; }
	bool SaveSampleCountImage( char const *filename ) const { return lodepng::encode(filename,&sampleCountImg[0],width,height,LCT_GREY,8) == 0; }

private:
	template <typename T, bool invert>
	T ComputeImage( std::vector<uint8_t> &img, std::vector<T> &data, T skipv )
	{
		int size = width * height;
		T vmin=std::numeric_limits<T>::max(), vmax=T(0);
		for ( int i=0; i<size; i++ ) {
			if ( data[i] == skipv ) continue;
			if ( vmin > data[i] ) vmin = data[i];
			if ( vmax < data[i] ) vmax = data[i];
		}
		for ( int i=0; i<size; i++ ) {
			if ( data[i] == skipv ) img[i] = 0;
			else {
				float f = float(data[i]-vmin)/float(vmax-vmin);
				if constexpr ( invert ) f = 1 - f;
				int c = int(f * 255);
				img[i] = c < 0 ? 0 : ( c > 255 ? 255 : c );
			}
		}
		return vmax;
	}
};

//-------------------------------------------------------------------------------

class ShadeInfo
{
public:
	ShadeInfo( std::vector<Light*> const &lightList, TexturedColor const &environment, RNG &r ) : lights(lightList), env(environment), rng(r) {}

	virtual Vec3f P () const { return hInfo.p; }	// returns the shading position
	virtual Vec3f V () const { return -ray.dir; }	// returns the view vector
	virtual Vec3f N () const { return hInfo.N; }	// returns the shading normal
	virtual Vec3f GN() const { return hInfo.GN; }	// returns the geometry normal

	virtual float Depth  () const { return hInfo.z; }		// returns the distance between the shaded hit point and the ray origin
	virtual bool  IsFront() const { return hInfo.front; }	// returns if the shading front part of the surface

	virtual Node const * GetNode() const { return hInfo.node; }	// returns the node that contains the shaded point

	virtual int X() const { return pixelX; }	// returns the current pixel's x coordinate
	virtual int Y() const { return pixelY; }	// returns the current pixel's y coordinate

	virtual int  CurrentBounce     () const { return bounce; }	// returns the current bounce (zero for primary rays)
	virtual int  CurrentPixelSample() const { return pSample; }	// returns the current pixel sample ID

	virtual float IOR() const { return 1.0f; }	// outside refraction index

	virtual int   MaterialID() const { return hInfo.mtlID; }	// returns the material ID
	virtual Vec3f UVW       () const { return hInfo.uvw; }		// returns the texture coordinates
	virtual Vec3f dUVW_dX   () const { return hInfo.duvw[0]; }	// returns the texture coordinate derivative in screen-space X direction
	virtual Vec3f dUVW_dY   () const { return hInfo.duvw[1]; }	// returns the texture coordinate derivative in screen-space Y direction

	virtual Color Eval( TexturedColor const &c ) const { return c.Eval(hInfo.uvw,hInfo.duvw); }	// evaluates the given texture at the shaded texture coordinates
	virtual float Eval( TexturedFloat const &f ) const { return f.Eval(hInfo.uvw,hInfo.duvw); }	// evaluates the given texture at the shaded texture coordinates

	virtual Color EvalEnvironment( Vec3f const &dir ) const { return env.EvalEnvironment(dir); };	// returns the environment color

	virtual int          NumLights()       const { return (int)lights.size(); }	// returns the number of lights to be used during shading
	virtual Light const* GetLight( int i ) const { return lights[i]; }			// returns the i^th light

	virtual bool CanBounce() const { return false; }	// returns if an additional bounce is permitted

	// Traces a shadow ray and returns the visibility
	virtual float TraceShadowRay( Ray   const &ray, float t_max=BIGFLOAT ) const { return 1.0f; }
	virtual float TraceShadowRay( Vec3f const &dir, float t_max=BIGFLOAT ) const { return TraceShadowRay(Ray(P(),dir),t_max); }

	// Traces a ray and returns the shaded color at the hit point.
	// It also sets t to the distance to the hit point, if a front is found.
	// if a back hit is found, dist should be set to zero.
	virtual Color TraceSecondaryRay( Ray   const &ray, float &dist, bool reflection=true ) const { dist=BIGFLOAT; return Color(0,0,0); }
	virtual Color TraceSecondaryRay( Vec3f const &dir, float &dist, bool reflection=true ) const { return TraceSecondaryRay(Ray(P(),dir),dist,reflection); }

	virtual float RandomFloat() const { return rng.RandomFloat(); }

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

	void SetPixelSample( int i ) { pSample = i; }

protected:
	Ray     ray;			// the ray that found this hit point
	HitInfo hInfo;			// ht information
	int     pixelX  = 0;	// current pixel's x coordinate
	int     pixelY  = 0;	// current pixel's y coordinate
	int     bounce  = 0;	// current bounce
	int     pSample = 0;	// current pixel sample

	RNG &rng;	// random number generator

	std::vector<Light*> const &lights;	// lights
	TexturedColor       const &env;		// environment
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