//-------------------------------------------------------------------------------
///
/// \file       objects.h 
/// \author     Cem Yuksel (www.cemyuksel.com)
/// \version    6.0
/// \date       September 19, 2025
///
/// \brief Example source for CS 6620 - University of Utah.
///
/// Copyright (c) 2019 Cem Yuksel. All Rights Reserved.
///
/// This code is provided for educational use only. Redistribution, sharing, or 
/// sublicensing of this code or its derivatives is strictly prohibited.
///
//-------------------------------------------------------------------------------

#ifndef _OBJECTS_H_INCLUDED_
#define _OBJECTS_H_INCLUDED_

#include "scene.h"
#include "cyTriMesh.h"
#include "cyBVH.h"

//-------------------------------------------------------------------------------

class Sphere : public Object
{
public:
	bool IntersectRay( Ray const &ray, HitInfo &hInfo, int hitSide=HIT_FRONT ) const override;
	Box  GetBoundBox() const override { return Box(-1,-1,-1,1,1,1); }
	void ViewportDisplay( Material const *mtl ) const override;
};

//-------------------------------------------------------------------------------

class Plane : public Object
{
public:
	bool IntersectRay( Ray const &ray, HitInfo &hInfo, int hitSide=HIT_FRONT ) const override;
	Box  GetBoundBox() const override { return Box(-1,-1,0,1,1,0); }
	void ViewportDisplay( const Material *mtl ) const override;
};

extern Plane thePlane;

//-------------------------------------------------------------------------------

class TriObj : public Object, public TriMesh
{
public:
	bool IntersectRay( Ray const &ray, HitInfo &hInfo, int hitSide=HIT_FRONT ) const override;
	Box  GetBoundBox() const override { return Box(GetBoundMin(),GetBoundMax()); }
	void ViewportDisplay( const Material *mtl ) const override;

	bool Load( char const *filename )
	{
		if ( ! LoadFromFileObj( filename ) ) return false;
		if ( ! HasNormals() ) ComputeNormals();
		ComputeBoundingBox();
		bvh.SetMesh(this,4);
		return true;
	}

private:
	BVHTriMesh bvh;
	bool IntersectTriangle( Ray const &ray, HitInfo &hInfo, int hitSide, unsigned int faceID ) const;
	bool TraceBVHNode     ( Ray const &ray, HitInfo &hInfo, int hitSide, unsigned int nodeID ) const;
};

//-------------------------------------------------------------------------------

#endif