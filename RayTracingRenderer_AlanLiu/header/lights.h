//-------------------------------------------------------------------------------
///
/// \file       lights.h 
/// \author     Cem Yuksel (www.cemyuksel.com)
/// \version    2.0
/// \date       September 19, 2025
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
	bool IsAmbient() const override { return true; }
	Color Illuminate( ShadeInfo const &sInfo, Vec3f &dir ) const override { return intensity; }
	void SetViewportLight( int lightID ) const override { SetViewportParam(lightID,ColorA(intensity),ColorA(0.0f),Vec4f(0,0,0,1)); }
	void Load( Loader const &loader ) override;
protected:
	Color intensity = Color(0,0,0);
};

//-------------------------------------------------------------------------------

class DirectLight : public GenLight
{
public:
	Color Illuminate( ShadeInfo const &sInfo, Vec3f &dir ) const override { dir=-direction; return intensity; }
	void SetViewportLight( int lightID ) const override { SetViewportParam(lightID,ColorA(0.0f),ColorA(intensity),Vec4f(-direction,0.0f)); }
	void Load( Loader const &loader ) override;
protected:
	Color intensity = Color(0,0,0);
	Vec3f direction = Vec3f(0,0,0);
};

//-------------------------------------------------------------------------------

class PointLight : public GenLight
{
public:
	Color Illuminate( ShadeInfo const &sInfo, Vec3f &dir ) const override { Vec3f d=position-sInfo.P(); dir=d.GetNormalized(); return intensity; }
	void SetViewportLight( int lightID ) const override { SetViewportParam(lightID,ColorA(0.0f),ColorA(intensity),Vec4f(position,1.0f)); }
	void Load( Loader const &loader ) override;
protected:
	Color intensity = Color(0,0,0);
	Vec3f position  = Vec3f(0,0,0);
};

//-------------------------------------------------------------------------------

#endif