//-------------------------------------------------------------------------------
///
/// \file       materials.h 
/// \author     Cem Yuksel (www.cemyuksel.com)
/// \version    2.0
/// \date       September 19, 2025
///
/// \brief Example source for CS 6620 - University of Utah.
///
//-------------------------------------------------------------------------------

#ifndef _MATERIALS_H_INCLUDED_
#define _MATERIALS_H_INCLUDED_

#include "renderer.h"

//-------------------------------------------------------------------------------

class MtlBasePhongBlinn : public Material
{
public:
	void Load( Loader const &loader ) override;

	void SetDiffuse   ( Color const &d ) { diffuse    = d; }
	void SetSpecular  ( Color const &s ) { specular   = s; }
	void SetGlossiness( float        g ) { glossiness = g; }

	const Color& Diffuse   () const { return diffuse;    }
	const Color& Specular  () const { return specular;   }
	const float  Glossiness() const { return glossiness; }

protected:
	Color diffuse    = Color(0.5f);
	Color specular   = Color(0.7f);
	float glossiness = 20.0f;
};

//-------------------------------------------------------------------------------

class MtlPhong : public MtlBasePhongBlinn
{
public:
	Color Shade( ShadeInfo const &shadeInfo ) const override;
	void SetViewportMaterial( int mtlID=0 ) const override;	// used for OpenGL display
};

//-------------------------------------------------------------------------------

class MtlBlinn : public MtlBasePhongBlinn
{
public:
	Color Shade( ShadeInfo const &shadeInfo ) const override;
	void SetViewportMaterial ( int mtlID=0 ) const override;	// used for OpenGL display
};

//-------------------------------------------------------------------------------

class MtlMicrofacet : public Material
{
public:
	void Load( Loader const &loader ) override;

	void SetBaseColor( Color const &c ) { baseColor = c; }
	void SetRoughness( float        r ) { roughness = r; }
	void SetMetallic ( float        m ) { metallic  = m; }
	void SetIOR      ( float        i ) { ior       = i; }

	Color Shade( ShadeInfo const &shadeInfo ) const override;
	void SetViewportMaterial( int mtlID=0 ) const override;	// used for OpenGL display

private:
	Color baseColor = Color(0.5f);	// albedo for dielectrics, F0 for metals
	float roughness = 1.0f;
	float metallic  = 0.0f;
	float ior       = 1.5f;	// index of refraction
};

//-------------------------------------------------------------------------------

#endif