//-------------------------------------------------------------------------------
///
/// \file       scene.h 
/// \author     Cem Yuksel (www.cemyuksel.com)
/// \version    2.0
/// \date       September 19, 2025
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
class Node;
class ShadeInfo;
class Loader;

template <class T> class ItemList;

typedef ItemList<Light>    LightList;
typedef ItemList<Material> MaterialList;

//-------------------------------------------------------------------------------

class Ray
{
public:
	Vec3f p, dir;

	Ray() {}
	Ray( Vec3f const &_p, Vec3f const &_dir) : p(_p), dir(_dir) {}
	Ray( Ray const &r) : p(r.p), dir(r.dir) {}
	void Normalize() { dir.Normalize(); }
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
	bool        front;	// true if the ray hits the front side, false if the ray hits the back side

	HitInfo() { Init(); }
	void Init() { z=BIGFLOAT; node=nullptr; front=true; }
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
		hInfo.p = TransformFrom(hInfo.p);
		hInfo.N = NormalTransformFrom(hInfo.N);
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
	virtual void ViewportDisplay( Material const *mtl ) const {}	// used for OpenGL display
	virtual void Load( Loader const &loader ) {}
};

//-------------------------------------------------------------------------------

class Light : public ItemBase
{
public:
	virtual Color Illuminate( ShadeInfo const &sInfo, Vec3f &dir ) const=0;	// returns the light intensity and direction
	virtual bool  IsAmbient () const { return false; }
	virtual void  SetViewportLight( int lightID ) const {}	// used for OpenGL display
	virtual void  Load( Loader const &loader ) {}
};

//-------------------------------------------------------------------------------

class Material : public ItemBase
{
public:
	virtual Color Shade( ShadeInfo const &shadeInfo ) const=0;	// the main method that handles shading
	virtual void SetViewportMaterial( int mtlID=0 ) const {}	// used for OpenGL display
	virtual void Load( Loader const &loader ) {}
};

//-------------------------------------------------------------------------------

class Node : public ItemBase, public Transformation
{
private:
	std::vector<Node*> childNodes;		// Child nodes
	Object            *obj = nullptr;	// Object reference (merely points to the object, but does not own the object, so it doesn't get deleted automatically)
	Material          *mtl = nullptr;	// Material used for shading the object
public:
	virtual ~Node() { DeleteAllChildNodes(); }

	void Init() { DeleteAllChildNodes(); obj=nullptr; mtl=nullptr; SetName(nullptr); InitTransform(); } // Initialize the node deleting all childNodes nodes

	// Hierarchy management
	int         GetNumChild() const       { return (int) childNodes.size(); }
	Node const* GetChild( int i ) const   { return childNodes[i]; }
	Node*       GetChild( int i )         { return childNodes[i]; }
	void        AppendChild( Node *node ) { childNodes.push_back(node); }
	void        DeleteAllChildNodes()     { for ( Node *c : childNodes ) { c->DeleteAllChildNodes(); delete c; } childNodes.clear(); }

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
	float fov;
	int imgWidth, imgHeight;

	void Init()
	{
		pos.Set(0,0,0);
		dir.Set(0,0,-1);
		up .Set(0,1,0);
		fov       = 40;
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
	Node         rootNode;
	LightList    lights;
	MaterialList materials;

	void Load( Loader const &loader );
};

//-------------------------------------------------------------------------------

#endif
