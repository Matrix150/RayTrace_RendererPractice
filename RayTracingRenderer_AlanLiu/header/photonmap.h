//-------------------------------------------------------------------------------
///
/// \file       photonmap.h 
/// \author     Cem Yuksel (www.cemyuksel.com)
/// \version    12.0
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

#ifndef _PHOTON_MAP_H_INCLUDED_
#define _PHOTON_MAP_H_INCLUDED_

//-------------------------------------------------------------------------------

#define PHOTONMAP_FILTER_CONSTANT  0
#define PHOTONMAP_FILTER_LINEAR    1
#define PHOTONMAP_FILTER_QUADRATIC 2

//-------------------------------------------------------------------------------

#define _USE_MATH_DEFINES
#include <math.h>
#include "cyVector.h"
#include "cyColor.h"
#include <vector>
#include <atomic>

//-------------------------------------------------------------------------------

//! Photon map class
class PhotonMap
{
public:
	//! A compact representation of a single photon data
	struct PhotonData
	{
		Vec3f position;
		float power;
		Vec3f pDir;
		Color24 color;
		unsigned char planeAndDirZ;  // splitting plane for kd-tree and one bit for determining the z direction

		void  SetPower    ( Color const &c );
		void  ScalePower  ( float scale ) { power *= scale; }
		void  SetDirection( Vec3f const &d ) { pDir = d; }
		void  SetPlane    ( unsigned char plane ) { planeAndDirZ = (planeAndDirZ & 0x8) | (plane & 0x3); }

		Color GetPower    () const { return color.ToColor() * power; }
		float GetMaxPower () const { return power; }
		Vec3f GetDirection() const { return pDir; }
		int   GetPlane    () const { return planeAndDirZ & 0x3; }
	};

	//! Removes all photons and deallocates the memory.
	void Clear() { std::vector<PhotonData>().swap(photons); numStoredPhotons=0; }

	//! Resizes the photon map by allocating enough memory for n photons.
	void Resize( int n ) { photons.resize(n+1); numStoredPhotons=0; }

	//! Adds a photon to the map with the given position, direction, and power.
	//! Assumes that the direction is normalized.
	//! Returns false if the photon map is full and that the photon cannot be inserted.
	bool AddPhoton( Vec3f const &pos, Vec3f const &dir, Color const &power );

	//! Returns the number of photons stored in the map
	int NumPhotons() const { return numStoredPhotons; }

	//! Returns the total size of the photon map.
	int Size() const { return int(photons.size()) - 1; }

	//! Returns the remaining space in the photon map.
	int RemainingSpace() const { return int(photons.size()) - numStoredPhotons - 1; }

	//! Scales the photon powers using the given scale factor
	void ScalePhotonPowers( float scale, int start=0, int end=-1 ) { if ( end<0 ) end=numStoredPhotons; for ( int i=start+1; i<=end; i++ ) photons[i].ScalePower(scale); }

	//! Builds a balanced kd-tree.
	//! This method must be called after adding all photons and
	//! before calling the EstimateIrradiance() method for the first time.
	void PrepareForIrradianceEstimation();

	//! Returns the irradiance estimate from the photon map at the given position with the given surface normal.
	//! The resulting irradiance estimation is scaled by the geometry term.
	template <int maxPhotons, int filterType=PHOTONMAP_FILTER_CONSTANT>
	void EstimateIrradiance( Color &irrad, Vec3f &direction, float radius, Vec3f const &pos ) const
		{ IrradianceEstimate<false,maxPhotons,filterType>(irrad,direction,radius,pos,Vec3f(0,0,0),1); }

	//! Returns the irradiance estimate from the photon map at the given position with the given surface normal.
	//! The resulting irradiance estimation is scaled by the geometry term.
	template <int maxPhotons, int filterType=PHOTONMAP_FILTER_CONSTANT>
	void EstimateIrradiance( Color &irrad, Vec3f &direction, float radius, Vec3f const &pos, Vec3f const &normal, float ellipticity=1 ) const
		{ IrradianceEstimate<true,maxPhotons,filterType>(irrad,direction,radius,pos,normal,ellipticity); }

	//! Returns the closest photon to the given position.
	//! If no photon is found within the radius, returns false.
	bool GetNearestPhoton( PhotonData &photon, float radius, Vec3f const &pos )                                         const { return NearestPhoton<false>(photon,radius,pos,Vec3f(0,0,0),1); }
	bool GetNearestPhoton( PhotonData &photon, float radius, Vec3f const &pos, Vec3f const &normal, float ellipticity ) const { return NearestPhoton<true >(photon,radius,pos,normal,ellipticity); }

	//! Returns the photon i.
	PhotonData       & operator [] ( int i )       { return photons[i+1]; }
	PhotonData const & operator [] ( int i ) const { return photons[i+1]; }
	PhotonData       * GetPhotons()       { return &photons[1]; }
	PhotonData const * GetPhotons() const { return &photons[1]; }

protected:
	std::vector<PhotonData> photons;
	std::atomic<int> numStoredPhotons;
	int halfStoredPhotons;

private:
	//! Balances the given kd-tree segment
	void BalanceSegment( std::vector<PhotonData> &balancedMap, Vec3f const &boxMin, Vec3f const &boxMax, int index, int start, int end );

	//! Swaps the two photons
	void SwapPhotons( int i, int j ) { PhotonData p=photons[i]; photons[i]=photons[j]; photons[j]=p; }

	struct NearestPhotons
	{
		Vec3f pos;
		Vec3f normal;
		float normScale;
		int maxPhotons;
		int found;
		float *dist2;
		PhotonData *photon;
	};

	template <bool useNormal>
	void LocatePhotons( NearestPhotons &np, int index ) const;

	template <bool useNormal, int maxPhotons, int filterType=PHOTONMAP_FILTER_CONSTANT>
	void IrradianceEstimate( Color &irrad, Vec3f &direction, float radius, Vec3f const &pos, Vec3f const &normal, float ellipticity ) const;

	template <bool useNormal>
	bool NearestPhoton( PhotonData &photon, float radius, Vec3f const &pos, Vec3f const &normal, float ellipticity ) const;

};

//-------------------------------------------------------------------------------

inline void PhotonMap::PhotonData::SetPower( Color const &c )
{
	power = c.r;
	if ( power < c.g ) power = c.g;
	if ( power < c.b ) power = c.b;
	color = Color24(c / power);
}

//-------------------------------------------------------------------------------

inline bool PhotonMap::AddPhoton( Vec3f const &pos, Vec3f const &dir, Color const &power )
{
	if ( numStoredPhotons >= Size() ) return false;
	int i = ++numStoredPhotons;
	if ( i > Size() ) {
		numStoredPhotons--;
		return false;
	}
	PhotonData p;
	p.position = pos;
	p.SetDirection(dir);
	p.SetPower(power);
	photons[i] = p;
	return true;
}

//-------------------------------------------------------------------------------

inline void PhotonMap::PrepareForIrradianceEstimation()
{
	if ( photons.size() == 0 || numStoredPhotons==0 ) return;

	// compute bounding box
	Vec3f boxMin = photons[0].position;
	Vec3f boxMax = photons[0].position;
	for ( int i=1; i<=numStoredPhotons; i++ ) {
		if ( boxMin.x > photons[i].position.x ) boxMin.x = photons[i].position.x;
		if ( boxMax.x < photons[i].position.x ) boxMax.x = photons[i].position.x;
		if ( boxMin.y > photons[i].position.y ) boxMin.y = photons[i].position.y;
		if ( boxMax.y < photons[i].position.y ) boxMax.y = photons[i].position.y;
		if ( boxMin.z > photons[i].position.z ) boxMin.z = photons[i].position.z;
		if ( boxMax.z < photons[i].position.z ) boxMax.z = photons[i].position.z;
	}

	// balance the map
	std::vector<PhotonData> balancedMap( numStoredPhotons+1 );
	BalanceSegment(balancedMap, boxMin, boxMax, 1, 1, numStoredPhotons );

	balancedMap.swap( photons );
	halfStoredPhotons = numStoredPhotons/2 - 1;
}

//-------------------------------------------------------------------------------

inline void PhotonMap::BalanceSegment( std::vector<PhotonData> &balancedMap, Vec3f const &boxMin, Vec3f const &boxMax, int index, int start, int end )
{
	// find median
	int median=1;
	while ((4*median) <= (end-start+1)) median += median;
	if ((3*median) <= (end-start+1)) {
		median += median;
		median += start-1;
	} else {
		median = end-median+1;
	}

	// find splitting axis
	int axis = 2;
	Vec3f boxDif = boxMax - boxMin;
	if ( boxDif.x > boxDif.y ) {
		if ( boxDif.x > boxDif.z ) axis = 0;
	} else if ( boxDif.y > boxDif.z ) axis = 1;

	// partition photon block around the median
	int left = start;
	int right = end;
	while ( right > left ) {
		float const v = photons[right].position[axis];
		int i = left - 1;
		int j = right;
		while ( photons[++i].position[axis] < v );
		while ( photons[--j].position[axis] > v && j>left );
		while ( i < j ) {
			SwapPhotons(i,j);
			while ( photons[++i].position[axis] < v ) ;
			while ( photons[--j].position[axis] > v && j>left ) ;
		}
		SwapPhotons(i,right);
		if ( i >= median ) right = i-1;
		if ( i <= median ) left = i+1;
	}

	// set the photon at index
	balancedMap[index] = photons[median];
	balancedMap[index].SetPlane(axis);

	// recursively balance the two sides of the median
	if ( median > start ) {
		if ( start < median-1 ) {
			Vec3f tBoxMax = boxMax;
			tBoxMax[axis] = balancedMap[index].position[axis];
			BalanceSegment( balancedMap, boxMin, tBoxMax, 2*index, start, median-1 );
		} else {
			balancedMap[ 2*index ] = photons[ start ];
		}
	}

	if ( median < end ) {
		if ( median+1 < end ) {
			Vec3f tBoxMin = boxMin;
			tBoxMin[axis] = balancedMap[index].position[axis];
			BalanceSegment( balancedMap, tBoxMin, boxMax, 2*index+1, median+1, end );
		} else {
			balancedMap[ 2*index+1 ] = photons[end];
		}
	}
}

//-------------------------------------------------------------------------------

template <bool useNormal, int maxPhotons, int filterType>
inline void PhotonMap::IrradianceEstimate( Color &irrad, Vec3f &direction, float radius, Vec3f const &pos, Vec3f const &normal, float ellipticity ) const
{
	irrad.SetBlack();
	direction.Zero();

	float found_dist2[maxPhotons+1];
	PhotonData found_photon[maxPhotons+1];
	NearestPhotons np;
	np.pos = pos;
	np.normal = normal;
	np.normScale = ellipticity==1 ? 0 : 1/ellipticity - 1;
	np.maxPhotons = maxPhotons;
	np.found = 0;
	np.dist2 = found_dist2;
	np.photon = found_photon;
	np.dist2[0] = radius*radius;

	LocatePhotons<useNormal>( np, 1 );

	// sum irradiance from all photons
	for (int i=1; i<=np.found; i++) {
		Color power = np.photon[i].GetPower();
		float filter = 1;
		if constexpr ( filterType == PHOTONMAP_FILTER_LINEAR    ) filter = 1 - Sqrt(np.dist2[i]/np.dist2[0]);
		if constexpr ( filterType == PHOTONMAP_FILTER_QUADRATIC ) filter = 1 - np.dist2[i]/np.dist2[0];
		irrad += filter * power;
		Vec3f dir = np.photon[i].GetDirection();
		direction += dir * (filter * np.photon[i].GetMaxPower());
	}

	if ( np.found > 0 ) {
		float area = Pi<float>()*np.dist2[0];
		if constexpr ( filterType == PHOTONMAP_FILTER_LINEAR    ) area *= 1.0f/3.0f;
		if constexpr ( filterType == PHOTONMAP_FILTER_QUADRATIC ) area *= 0.5f;
		if ( area > 0 ) {
			float const one_over_area = 1.0f/area;
			irrad *= one_over_area;
		}
		direction.Normalize();
	}
}

//-------------------------------------------------------------------------------

template <bool useNormal>
inline bool PhotonMap::NearestPhoton( PhotonMap::PhotonData &photon, float radius, Vec3f const &pos, Vec3f const &normal, float ellipticity ) const
{
	float found_dist2[2];
	PhotonData found_photon[2];
	NearestPhotons np;
	np.pos = pos;
	np.normal = normal;
	np.normScale = ellipticity==1 ? 0 : 1/ellipticity - 1;
	np.maxPhotons = 1;
	np.found = 0;
	np.dist2 = found_dist2;
	np.photon = found_photon;
	np.dist2[0] = radius*radius;

	LocatePhotons<useNormal>( np, 1 );

	if ( np.found ) {
		photon = np.photon[1];
		return true;
	}
	return false;
}

//-------------------------------------------------------------------------------

template <bool useNormal>
inline void PhotonMap::LocatePhotons( NearestPhotons &np, int index ) const
{
	PhotonData const &p = photons[index];
	int axis = p.GetPlane();

	// if this is an internal node
	if ( index < halfStoredPhotons ) {
		float dist = np.pos[axis] - p.position[axis];
		if ( dist > 0 ) {
			LocatePhotons<useNormal>( np, 2*index+1 );
			if ( dist*dist < np.dist2[0] ) LocatePhotons<useNormal>( np, 2*index );
		} else {
			LocatePhotons<useNormal>( np, 2*index );
			if ( dist*dist < np.dist2[0] ) LocatePhotons<useNormal>( np, 2*index+1 );
		}
	}

	// compute squared distance between current photon and np->pos
	Vec3f dif = p.position - np.pos;
	float dist2 = dif.LengthSquared();

	if ( dist2 < np.dist2[0] ) {

		// Check if the photon direction is acceptable
		if constexpr ( useNormal ) {
			Vec3f dir = p.GetDirection();
			if ( (dir % np.normal) >= 0 ) return;
			if ( np.normScale > 0 ) {
				float perp = dif % np.normal;
				dif += np.normal * (perp * np.normScale);
				dist2 = dif.LengthSquared();
				if ( dist2 >= np.dist2[0] ) return;
			}
		}

		if ( np.found < np.maxPhotons ) {
			np.found++;
			np.dist2[np.found] = dist2;
			np.photon[np.found] = p;
			if ( np.found == np.maxPhotons ) { // build a heap
				int half_found = np.found >> 1;
				for ( int k=half_found; k>=1; k--) {
					int parent = k;
					PhotonData tp = np.photon[k];
					float td2 = np.dist2[k];
					while ( parent <= half_found ) {
						int j = parent + parent;
						if ( j < np.found && np.dist2[j] < np.dist2[j+1] ) j++;
						if ( td2 >= np.dist2[j] ) break;
						np.dist2[parent] = np.dist2[j];
						np.photon[parent] = np.photon[j];
						parent=j;
					}
					np.photon[parent] = tp;
					np.dist2[parent] = td2;
				}
			}
		} else {
			int parent = 1;
			int j = 2;
			while ( j <= np.found ) {
				if ( j < np.found && np.dist2[j] < np.dist2[j+1] ) j++;
				if ( dist2 > np.dist2[j] ) break;
				np.dist2[parent] = np.dist2[j];
				np.photon[parent] = np.photon[j];
				parent = j;
				j <<= 1;
			}
			np.photon[parent] = p;
			np.dist2[parent] = dist2;
			np.dist2[0] = np.dist2[1];
		}

	}
}

//-------------------------------------------------------------------------------

#endif