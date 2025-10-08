//-------------------------------------------------------------------------------
///
/// \file       materials.h 
/// \author     Cem Yuksel (www.cemyuksel.com)
/// \version    4.0
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
	void SetReflection( Color const &r ) { reflection = r; }
	void SetRefraction( Color const &r ) { refraction = r; }
	void SetAbsorption( Color const &a ) { absorption = a; }
	void SetIOR       ( float        i ) { ior        = i; }

	const Color& Diffuse   () const { return diffuse;    }
	const Color& Specular  () const { return specular;   }
	const float  Glossiness() const { return glossiness; }
	const Color& Reflection() const { return reflection; }
	const Color& Refraction() const { return refraction; }
	const Color& Absorption() const { return absorption; }
	float        IOR       () const { return ior;        }

protected:
	Color diffuse    = Color(0.5f);
	Color specular   = Color(0.7f);
	float glossiness = 20.0f;
	Color reflection = Color(0.0f);
	Color refraction = Color(0.0f);
	Color absorption = Color(0.0f);
	float ior        = 1.5f;	// index of refraction
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

	void SetBaseColor    ( Color const &c ) { baseColor     = c; }
	void SetRoughness    ( float        r ) { roughness     = r; }
	void SetMetallic     ( float        m ) { metallic      = m; }
	void SetTransmittance( Color const &t ) { transmittance = t; }
	void SetAbsorption   ( Color const &a ) { absorption    = a; }
	void SetIOR          ( float        i ) { ior           = i; }

	Color Shade( ShadeInfo const &shadeInfo ) const override;
	void SetViewportMaterial( int mtlID=0 ) const override;	// used for OpenGL display

private:
	Color baseColor     = Color(0.5f);	// albedo for dielectrics, F0 for metals
	float roughness     = 1.0f;
	float metallic      = 0.0f;
	Color transmittance = Color(0.0f);
	Color absorption    = Color(0.0f);
	float ior           = 1.5f;	// index of refraction
};

//-------------------------------------------------------------------------------

#endif