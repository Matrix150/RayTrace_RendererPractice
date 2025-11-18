//-------------------------------------------------------------------------------
///
/// \file       lights.h 
/// \author     Cem Yuksel (www.cemyuksel.com)
/// \version    11.0
/// \date       September 24, 2025
///
/// \brief Example source for CS 6620 - University of Utah.
///
//-------------------------------------------------------------------------------

#ifndef _LIGHTS_H_INCLUDED_
#define _LIGHTS_H_INCLUDED_

#include "renderer.h"

//-------------------------------------------------------------------------------

class GenLight : public Light
{
protected:
	void SetViewportParam( int lightID, ColorA const &ambient, ColorA const &intensity, Vec4f const &pos ) const;
};

//-------------------------------------------------------------------------------

class AmbientLight : public GenLight
{
public:
	Color Illuminate( ShadeInfo const &sInfo, Vec3f &dir ) const override { return intensity; }
	bool  IsAmbient() const override { return true; }
	void  SetViewportLight( int lightID ) const override { SetViewportParam(lightID,ColorA(intensity),ColorA(0.0f),Vec4f(0,0,0,1)); }
	void  Load( Loader const &loader ) override;
protected:
	Color intensity = Color(0,0,0);
};

//-------------------------------------------------------------------------------

class DirectLight : public GenLight
{
public:
	Color Illuminate( ShadeInfo const &sInfo, Vec3f &dir ) const override { dir=-direction; return intensity * sInfo.TraceShadowRay(-direction); }
	void  SetViewportLight( int lightID ) const override { SetViewportParam(lightID,ColorA(0.0f),ColorA(intensity),Vec4f(-direction,0.0f)); }
	void  Load( Loader const &loader ) override;
protected:
	Color intensity = Color(0,0,0);
	Vec3f direction = Vec3f(0,0,0);
};

//-------------------------------------------------------------------------------

class PointLight : public GenLight
{
public:
	Color Illuminate( ShadeInfo const &sInfo, Vec3f &dir ) const override;
	Color Radiance  ( ShadeInfo const &sInfo ) const override { return intensity / (Pi<float>()*size*size); }
	bool  IsRenderable() const override { return size > 0.0f; }
	void  SetViewportLight( int lightID ) const override;
	void  Load( Loader const &loader ) override;

	bool IntersectRay( Ray const &ray, HitInfo &hInfo, int hitSide=HIT_FRONT ) const override;
	Box  GetBoundBox() const override { return Box( position-size, position+size ); }
	void ViewportDisplay( Material const *mtl ) const override;	// used for OpenGL display

protected:
	Color intensity   = Color(0,0,0);
	Vec3f position    = Vec3f(0,0,0);
	float size        = 0.0f;
	float attenuation = 0.0f;	// Zero means no attenuation. If non-zero, light distance is scaled by attenuation.
};

//-------------------------------------------------------------------------------

#endif