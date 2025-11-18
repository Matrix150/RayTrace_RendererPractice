//-------------------------------------------------------------------------------
///
/// \file       materials.h 
/// \author     Cem Yuksel (www.cemyuksel.com)
/// \version    11.0
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
	void Load( Loader const &loader, TextureFileList &tfl ) override;

	void SetDiffuse   ( Color const &d ) { diffuse   .SetValue(d); }
	void SetSpecular  ( Color const &s ) { specular  .SetValue(s); }
	void SetGlossiness( float        g ) { glossiness.SetValue(g); }
	void SetEmission  ( Color const &e ) { emission  .SetValue(e); }
	void SetReflection( Color const &r ) { reflection.SetValue(r); }
	void SetRefraction( Color const &r ) { refraction.SetValue(r); }
	void SetAbsorption( Color const &a ) { absorption = a; }
	void SetIOR       ( float        i ) { ior        = i; }

	void SetDiffuseTexture   ( TextureMap *tex ) { diffuse   .SetTexture(tex); }
	void SetSpecularTexture	 ( TextureMap *tex ) { specular  .SetTexture(tex); }
	void SetGlossinessTexture( TextureMap *tex ) { glossiness.SetTexture(tex); }
	void SetEmissionTexture  ( TextureMap *tex ) { emission  .SetTexture(tex); }
	void SetReflectionTexture( TextureMap *tex ) { reflection.SetTexture(tex); }
	void SetRefractionTexture( TextureMap *tex ) { refraction.SetTexture(tex); }

	const TexturedColor& Diffuse   () const { return diffuse;    }
	const TexturedColor& Specular  () const { return specular;   }
	const TexturedFloat& Glossiness() const { return glossiness; }
	const TexturedColor& Emission  () const { return emission;   }
	const TexturedColor& Reflection() const { return reflection; }
	const TexturedColor& Refraction() const { return refraction; }

	Color Absorption( int mtlID=0 ) const override { return absorption; }
	float IOR       ( int mtlID=0 ) const override { return ior; }

protected:
	TexturedColor diffuse    = Color(0.5f);
	TexturedColor specular   = Color(0.7f);
	TexturedFloat glossiness = 20.0f;
	TexturedColor emission   = Color(0.0f);
	TexturedColor reflection = Color(0.0f);
	TexturedColor refraction = Color(0.0f);
	Color         absorption = Color(0.0f);
	float         ior        = 1.5f;	// index of refraction
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
	void Load( Loader const &loader, TextureFileList &tfl ) override;

	void SetBaseColor    ( Color const &c ) { baseColor    .SetValue(c); }
	void SetRoughness    ( float        r ) { roughness    .SetValue(r); }
	void SetMetallic     ( float        m ) { metallic     .SetValue(m); }
	void SetEmission     ( Color const &e ) { emission     .SetValue(e); }
	void SetTransmittance( Color const &t ) { transmittance.SetValue(t); }
	void SetAbsorption   ( Color const &a ) { absorption = a; }
	void SetIOR          ( float        i ) { ior        = i; }

	void SetBaseColorTexture    ( TextureMap *tex ) { baseColor    .SetTexture(tex); }
	void SetRoughnessTexture    ( TextureMap *tex ) { roughness    .SetTexture(tex); }
	void SetMetallicTexture     ( TextureMap *tex ) { metallic     .SetTexture(tex); }
	void SetEmissionTexture     ( TextureMap *tex ) { emission     .SetTexture(tex); }
	void SetTransmittanceTexture( TextureMap *tex ) { transmittance.SetTexture(tex); }

	Color Shade( ShadeInfo const &shadeInfo ) const override;
	Color Absorption         ( int mtlID=0 ) const override { return absorption; }
	float IOR                ( int mtlID=0 ) const override { return ior;        }
	void  SetViewportMaterial( int mtlID=0 ) const override;	// used for OpenGL display

private:
	TexturedColor baseColor     = Color(0.5f);	// albedo for dielectrics, F0 for metals
	TexturedFloat roughness     = 1.0f;
	TexturedFloat metallic      = 0.0f;
	TexturedColor emission      = Color(0.0f);
	TexturedColor transmittance = Color(0.0f);
	Color         absorption    = Color(0.0f);
	float         ior           = 1.5f;	// index of refraction
};

//-------------------------------------------------------------------------------

class MultiMtl : public Material
{
public:
	virtual ~MultiMtl() { for ( Material *m : mtls ) delete m; }

	Color Shade( ShadeInfo const &sInfo ) const override { int m=sInfo.MaterialID(); return m<(int)mtls.size() ? mtls[m]->Shade(sInfo) : Color(1,1,1); }
	Color Absorption         ( int mtlID=0 ) const override { return mtlID<(int)mtls.size() ? mtls[mtlID]->Absorption     (mtlID) : Material::Absorption     (mtlID); }
	float IOR                ( int mtlID=0 ) const override { return mtlID<(int)mtls.size() ? mtls[mtlID]->IOR            (mtlID) : Material::IOR            (mtlID); }
	void  SetViewportMaterial( int mtlID=0 ) const override { if   ( mtlID<(int)mtls.size() ) mtls[mtlID]->SetViewportMaterial(); }

	void AppendMaterial( Material *m ) { mtls.push_back(m); }

private:
	std::vector<Material*> mtls;
};

//-------------------------------------------------------------------------------

#endif