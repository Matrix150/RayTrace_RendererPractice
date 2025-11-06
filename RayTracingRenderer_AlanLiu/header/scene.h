//-------------------------------------------------------------------------------
///
/// \file       scene.h 
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

#ifndef _SCENE_H_INCLUDED_
#define _SCENE_H_INCLUDED_

//-------------------------------------------------------------------------------

#include <vector>
#include <atomic>
#include <string>

#include "cyVector.h"
#include "cyMatrix.h"
#include "cyColor.h"
using namespace cy;

//-------------------------------------------------------------------------------

#define BIGFLOAT std::numeric_limits<float>::max()

class Object;
class Light;
class Material;
class Texture;
class Node;
class ShadeInfo;
class Loader;

template <class T> class ItemList;

typedef ItemList<Object>   ObjFileList;
typedef ItemList<Light>    LightList;
typedef ItemList<Material> MaterialList;
typedef ItemList<Texture>  TextureFileList;

//-------------------------------------------------------------------------------

struct Ray
{
	Vec3f p, dir;

	Ray() = default;
	Ray( Vec3f const &_p, Vec3f const &_dir ) : p(_p), dir(_dir) {}
};

//-------------------------------------------------------------------------------

#define HIT_NONE           0
#define HIT_FRONT          1
#define HIT_BACK           2
#define HIT_FRONT_AND_BACK (HIT_FRONT|HIT_BACK)

//-------------------------------------------------------------------------------

struct HitInfo
{
	Vec3f       p;		// position of the hit point
	float       z;		// the distance from the ray center to the hit point
	Node const *node;	// the object node that was hit
	Vec3f       N;		// surface normal at the hit point
	Vec3f       GN;		// geometry normal at the hit point
	Vec3f       uvw;	// texture coordinate at the hit point
	Vec3f       duvw[2];// derivatives of the texture coordinate
	int         mtlID;	// sub-material index
	bool        front;	// true if the ray hits the front side, false if the ray hits the back side
	bool        light;	// true if the ray hits a renderable light source

	HitInfo() { Init(); }
	void Init() { z=BIGFLOAT; node=nullptr; uvw.Set(0.5f); duvw[0].Zero(); duvw[1].Zero(); mtlID=0; front=true; light=false; }
};

//-------------------------------------------------------------------------------

class Box
{
public:
	Vec3f pmin, pmax;

	// Constructors
	Box() { Init(); }
	Box( Vec3f const &_pmin, Vec3f const &_pmax ) : pmin(_pmin), pmax(_pmax) {}
	Box( float xmin, float ymin, float zmin, float xmax, float ymax, float zmax ) : pmin(xmin,ymin,zmin), pmax(xmax,ymax,zmax) {}
	Box( float const *dim ) : pmin(dim[0],dim[1],dim[2]), pmax(dim[3],dim[4],dim[5]) {}

	// Initializes the box, such that there exists no point inside the box (i.e. it is empty).
	void Init() { pmin.Set(BIGFLOAT,BIGFLOAT,BIGFLOAT); pmax.Set(-BIGFLOAT,-BIGFLOAT,-BIGFLOAT); }

	// Returns true if the box is empty; otherwise, returns false.
	bool IsEmpty() const { return pmin.x>pmax.x || pmin.y>pmax.y || pmin.z>pmax.z; }

	// Returns one of the 8 corner point of the box in the following order:
	// 0:(x_min,y_min,z_min), 1:(x_max,y_min,z_min)
	// 2:(x_min,y_max,z_min), 3:(x_max,y_max,z_min)
	// 4:(x_min,y_min,z_max), 5:(x_max,y_min,z_max)
	// 6:(x_min,y_max,z_max), 7:(x_max,y_max,z_max)
	Vec3f Corner( int i ) const	// 8 corners of the box
	{
		Vec3f p;
		p.x = (i & 1) ? pmax.x : pmin.x;
		p.y = (i & 2) ? pmax.y : pmin.y;
		p.z = (i & 4) ? pmax.z : pmin.z;
		return p;
	}

	// Enlarges the box such that it includes the given point p.
	void operator += ( Vec3f const &p )
	{
		for ( int i=0; i<3; i++ ) {
			if ( pmin[i] > p[i] ) pmin[i] = p[i];
			if ( pmax[i] < p[i] ) pmax[i] = p[i];
		}
	}

	// Enlarges the box such that it includes the given box b.
	void operator += ( Box const &b )
	{
		for ( int i=0; i<3; i++ ) {
			if ( pmin[i] > b.pmin[i] ) pmin[i] = b.pmin[i];
			if ( pmax[i] < b.pmax[i] ) pmax[i] = b.pmax[i];
		}
	}

	// Returns true if the point is inside the box; otherwise, returns false.
	bool IsInside( Vec3f const &p ) const { for ( int i=0; i<3; i++ ) if ( pmin[i] > p[i] || pmax[i] < p[i] ) return false; return true; }

	// Returns true if the ray intersects with the box for any parameter that is smaller than t_max; otherwise, returns false.
	bool IntersectRay( Ray const &r, float t_max ) const;
};

//-------------------------------------------------------------------------------

class Transformation
{
private:
	Matrix34f tm;	// Transformation matrix from the local space
	Matrix34f itm;	// Inverse of the transformation matrix
public:
	Transformation() { InitTransform(); }

	void InitTransform() { tm.SetIdentity(); itm.SetIdentity(); }

	void Translate( Vec3f const &p )                   { Transform(Matrix34f::Translation(p)); }
	void Rotate   ( Vec3f const &axis, float degrees ) { Transform(Matrix34f::Rotation(axis,Deg2Rad(degrees))); }
	void Scale    ( Vec3f const &s )                   { Transform(Matrix34f::Scale(s)); }
	void Transform( Matrix34f const &m )               { tm=m*tm; itm=tm.GetInverse(); }

	Matrix34f const & GetTransform       () const { return tm; }
	Matrix34f const & GetInverseTransform() const { return itm; }

	Vec3f TransformTo  ( Vec3f const &p ) const { return itm*p; }	// Transform a position vector to the local coordinate system
	Vec3f TransformFrom( Vec3f const &p ) const { return tm *p; }	// Transform a position vector from the local coordinate system

	Vec3f DirectionTransformTo  ( Vec3f const &p ) const { return itm.GetSubMatrix3()*p; }	// Transform a direction vector to the local coordinate system
	Vec3f DirectionTransformFrom( Vec3f const &p ) const { return tm .GetSubMatrix3()*p; }	// Transform a direction vector from the local coordinate system

	// Transforms a normal vector to the local coordinate system (same as multiplication with the inverse transpose of the transformation)
	Vec3f NormalTransformTo( Vec3f const &dir ) const { return tm.GetSubMatrix3().TransposeMult(dir); }

	// Transforms a normal vector from the local coordinate system (same as multiplication with the inverse transpose of the transformation)
	Vec3f NormalTransformFrom( Vec3f const &dir ) const { return itm.GetSubMatrix3().TransposeMult(dir); }

	// Transformations
	Ray ToNodeCoords( Ray const &ray ) const { return Ray( TransformTo(ray.p), DirectionTransformTo(ray.dir) ); }
	void FromNodeCoords( HitInfo &hInfo ) const
	{
		hInfo.p  = TransformFrom      ( hInfo.p  );
		hInfo.N  = NormalTransformFrom( hInfo.N  );
		hInfo.GN = NormalTransformFrom( hInfo.GN );
	}

	void Load( Loader const &loader );
};

//-------------------------------------------------------------------------------

class ItemBase
{
private:
	std::string name;	// The name of the item
public:
	char const * GetName() const { return name.data(); }
	void SetName( char const *newName ) { name = newName ? newName : ""; }
};


//-------------------------------------------------------------------------------

class Object : public ItemBase
{
public:
	virtual bool IntersectRay( Ray const &ray, HitInfo &hInfo, int hitSide=HIT_FRONT ) const=0;
	virtual Box  GetBoundBox() const=0;
	virtual void ViewportDisplay( Material const *mtl ) const {}	// used for OpenGL display
	virtual void Load( Loader const &loader ) {}
};

//-------------------------------------------------------------------------------

class Light : public Object
{
public:
	virtual Color Illuminate( ShadeInfo const &sInfo, Vec3f &dir ) const=0;	// returns the light intensity and direction
	virtual Color Radiance  ( ShadeInfo const &sInfo ) const { return Color(0,0,0); }	// Used for shading a hit point on the light
	virtual bool  IsAmbient   () const { return false; }
	virtual bool  IsRenderable() const { return false; }
	virtual void  SetViewportLight( int lightID ) const {}	// used for OpenGL display
	virtual void  Load( Loader const &loader ) {}

	// From Object
	bool IntersectRay( Ray const &ray, HitInfo &hInfo, int hitSide=HIT_FRONT ) const override { return false; }
	Box  GetBoundBox() const override { return Box(); }	// empty box
};

//-------------------------------------------------------------------------------

class Material : public ItemBase
{
public:
	virtual Color Shade( ShadeInfo const &sInfo ) const=0;	// the main method that handles shading
	virtual Color Absorption         ( int mtlID=0 ) const { return Color(0,0,0); }	// returns the absorption of the material
	virtual float IOR                ( int mtlID=0 ) const { return 1.0f; }	// returns the refraction index of the material
	virtual void  SetViewportMaterial( int mtlID=0 ) const {}	// used for OpenGL display
	virtual void  Load( Loader const &loader, TextureFileList &textureFileList ) {}
};

//-------------------------------------------------------------------------------

class Texture : public ItemBase
{
public:
	// Evaluates the color at the given uvw location.
	virtual Color Eval( Vec3f const &uvw ) const=0;

	// Evaluates the color around the given uvw location using the derivatives duvw
	// by calling the Eval function multiple times.
	virtual Color Eval( Vec3f const &uvw, Vec3f const duvw[2] ) const
	{
		Color c = Eval(uvw);
		if ( duvw[0].LengthSquared() + duvw[1].LengthSquared() == 0 ) return c;
		const int sampleCount = 32;
		for ( int i=1; i<sampleCount; i++ ) {
			float x=0, y=0, fx=0.5f, fy=1.0f/3.0f;
			for ( int ix=i; ix>0; ix/=2 ) { x+=fx*(ix%2); fx/=2; }	// Halton sequence (base 2)
			for ( int iy=i; iy>0; iy/=3 ) { y+=fy*(iy%3); fy/=3; }	// Halton sequence (base 3)
			if ( x > 0.5f ) x-=1;
			if ( y > 0.5f ) y-=1;
			c += Eval( uvw + x*duvw[0] + y*duvw[1] );
		}
		return c / float(sampleCount);
	}

	virtual bool SetViewportTexture() const { return false; }	// used for OpenGL display

	virtual void Load( Loader const &loader, TextureFileList &textureFileList ) {}

protected:

	// Clamps the uvw values for tiling textures, such that all values fall between 0 and 1.
	static Vec3f TileClamp( Vec3f const &uvw )
	{
		Vec3f u;
		u.x = uvw.x - (int) uvw.x;
		u.y = uvw.y - (int) uvw.y;
		u.z = uvw.z - (int) uvw.z;
		if ( u.x < 0 ) u.x += 1;
		if ( u.y < 0 ) u.y += 1;
		if ( u.z < 0 ) u.z += 1;
		return u;
	}
};

//-------------------------------------------------------------------------------

// This class handles textures with texture transformations.
// The uvw values passed to the Eval methods are transformed
// using the texture transformation.
class TextureMap : public Transformation
{
public:
	TextureMap() : texture(nullptr) {}
	TextureMap( Texture const *tex ) : texture(tex) {}
	void SetTexture( Texture const *tex ) { texture = tex; }

	virtual Color Eval( Vec3f const &uvw ) const { return texture ? texture->Eval(TransformTo(uvw)) : Color(0,0,0); }
	virtual Color Eval( Vec3f const &uvw, Vec3f const duvw[2] ) const
	{
		if ( texture == nullptr ) return Color(0,0,0);
		Vec3f d[2] = { DirectionTransformTo(duvw[0]), DirectionTransformTo(duvw[1]) };
		return texture->Eval( TransformTo(uvw), d );
	}

	bool SetViewportTexture() const { if ( texture ) return texture->SetViewportTexture(); return false; }	// used for OpenGL display

private:
	Texture const *texture;
};

//-------------------------------------------------------------------------------

// This class keeps a TextureMap and a value. This is useful for keeping material
// parameters that can also be textures. If no texture is specified, it automatically 
// uses the value. Otherwise, the texture is multiplied by the value.
template <typename T>
class TexturedValue
{
private:
	T value;
	TextureMap *map;
public:
	TexturedValue() : value(T(0.0f)), map(nullptr) {}
	TexturedValue( T const &v ) : value(v), map(nullptr) {}
	virtual ~TexturedValue() { if ( map ) delete map; }

	void SetValue( T const &c ) { value=c; }
	void SetTexture( TextureMap *m ) { if ( map ) delete map; map=m; }

	T GetValue() const { return value; }
	const TextureMap* GetTexture() const { return map; }

	T Eval( Vec3f const &uvw ) const { return ( map ) ? value*EvalMap(uvw) : value; }
	T Eval( Vec3f const &uvw, Vec3f const duvw[2] ) const { return ( map ) ? value*EvalMap(uvw,duvw) : value; }

	// Returns the value value at the given direction for environment mapping.
	T EvalEnvironment( Vec3f const &dir ) const
	{
		float len = dir.Length();
		float z = std::asin(-dir.z/len)/Pi<float>()+0.5f;
		float x = dir.x / (fabs(dir.x)+fabs(dir.y));
		float y = dir.y / (fabs(dir.x)+fabs(dir.y));
		return Eval( Vec3f(0.5f + 0.5f*z*(x-y), 0.5f + 0.5f*z*(x+y), 0.0f) );
	}

private:
	T EvalMap( Vec3f const &uvw ) const;
	T EvalMap( Vec3f const &uvw, Vec3f const duvw[2] ) const;
};

template <> inline Color TexturedValue<Color>::EvalMap( Vec3f const &uvw ) const { return map->Eval(uvw); }
template <> inline float TexturedValue<float>::EvalMap( Vec3f const &uvw ) const { return map->Eval(uvw).r; }
template <> inline Color TexturedValue<Color>::EvalMap( Vec3f const &uvw, Vec3f const duvw[2] ) const { return map->Eval(uvw,duvw); }
template <> inline float TexturedValue<float>::EvalMap( Vec3f const &uvw, Vec3f const duvw[2] ) const { return map->Eval(uvw,duvw).r; }

typedef TexturedValue<Color> TexturedColor;
typedef TexturedValue<float> TexturedFloat;

//-------------------------------------------------------------------------------

class Node : public ItemBase, public Transformation
{
private:
	std::vector<Node*> childNodes;		// Child nodes
	Object            *obj = nullptr;	// Object reference (merely points to the object, but does not own the object, so it doesn't get deleted automatically)
	Material          *mtl = nullptr;	// Material used for shading the object
	Box                childBoundBox;	// Bounding box of the childNodes nodes, which does not include the object of this node, but includes the objects of the childNodes nodes
public:
	virtual ~Node() { DeleteAllChildNodes(); }

	void Init() { DeleteAllChildNodes(); obj=nullptr; mtl=nullptr; childBoundBox.Init(); SetName(nullptr); InitTransform(); } // Initialize the node deleting all childNodes nodes

	// Hierarchy management
	int         GetNumChild() const       { return (int) childNodes.size(); }
	Node const* GetChild( int i ) const   { return childNodes[i]; }
	Node*       GetChild( int i )         { return childNodes[i]; }
	void        AppendChild( Node *node ) { childNodes.push_back(node); }
	void        DeleteAllChildNodes()     { for ( Node *c : childNodes ) { c->DeleteAllChildNodes(); delete c; } childNodes.clear(); }

	// Bounding Box
	Box const& ComputeChildBoundBox()
	{
		childBoundBox.Init();
		for ( Node *c : childNodes ) {
			Box childBox = c->ComputeChildBoundBox();
			if ( c->GetNodeObj() ) childBox += c->GetNodeObj()->GetBoundBox();
			if ( ! childBox.IsEmpty() ) for ( int j=0; j<8; j++ ) childBoundBox += c->TransformFrom( childBox.Corner(j) );	// transform the box from childNodes coordinates
		}
		return childBoundBox;
	}
	Box const& GetChildBoundBox() const { return childBoundBox; }

	// Object management
	Object const* GetNodeObj() const { return obj; }
	Object*       GetNodeObj()       { return obj; }
	void          SetNodeObj( Object *object ) { obj = object; }

	// Material management
	Material const* GetMaterial() const { return mtl; }
	void            SetMaterial( Material *material ) { mtl = material; }
};

//-------------------------------------------------------------------------------

class Camera
{
public:
	Vec3f pos, dir, up;
	float fov, focaldist, dof;
	int imgWidth, imgHeight;

	void Init()
	{
		pos.Set(0,0,0);
		dir.Set(0,0,-1);
		up .Set(0,1,0);
		fov       = 40;
		focaldist = 1;
		dof       = 0;
		imgWidth  = 1920;
		imgHeight = 1080;
	}

	void Load( Loader const &loader );
};

//-------------------------------------------------------------------------------

template <class T>
class ItemList : public std::vector<T*>
{
public:
	virtual ~ItemList() { DeleteAll(); }
	void DeleteAll() { for ( T *i : *this ) if (i) delete i; this->clear(); }
	T* Find( char const *name ) const { for ( T *i : *this ) if ( i && strcmp(name,i->GetName())==0 ) return i; return nullptr; }
};

//-------------------------------------------------------------------------------

struct Scene
{
	Node            rootNode;
	ObjFileList     objList;
	LightList       lights;
	MaterialList    materials;
	TextureFileList texFiles;
	TexturedColor   background;
	TexturedColor   environment;

	void Load( Loader const &loader );
};

//-------------------------------------------------------------------------------

#endif
