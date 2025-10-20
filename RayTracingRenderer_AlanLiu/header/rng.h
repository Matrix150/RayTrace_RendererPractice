//-------------------------------------------------------------------------------
///
/// \file       rng.h 
/// \author     Cem Yuksel (www.cemyuksel.com)
/// \version    8.0
/// \date       September 26, 2025
///
/// \brief Project source for CS 6620 - University of Utah.
///
/// Copyright (c) 2025 Cem Yuksel. All Rights Reserved.
///
/// This code is provided for educational use only. Redistribution, sharing, or 
/// sublicensing of this code or its derivatives is strictly prohibited.
///
//-------------------------------------------------------------------------------


#ifndef _RNG_H_INCLUDED_
#define _RNG_H_INCLUDED_

//-------------------------------------------------------------------------------

#include <stdint.h>
#include <thread>

//-------------------------------------------------------------------------------

// Random Number Generator using the PCG algorithm
class RNG
{
public:
	// Constructors
	RNG() { SetSequence( static_cast<uint32_t>( std::hash<std::thread::id>{}( std::this_thread::get_id() ) ) ); }
	RNG( uint64_t sequenceIndex )                { SetSequence(sequenceIndex); }
	RNG( uint64_t sequenceIndex, uint64_t seed ) { SetSequence(sequenceIndex,seed); }

	// Selects a sequence with the given index and seed.
	void SetSequence( uint64_t sequenceIndex ) { SetSequence( sequenceIndex, MixBits(sequenceIndex) ); }
	void SetSequence( uint64_t sequenceIndex, uint64_t seed )
	{
		state = 0u;
		inc = (sequenceIndex << 1u) | 1u;
		RandomInt();
		state += seed;
		RandomInt();
	}

	// Returns a random integer.
	uint32_t RandomInt()
	{
		// minimal PCG32 / (c) 2014 M.E. O'Neill / pcg-random.org
		uint64_t oldstate = state;
		state = oldstate * PCG32_Mult() + inc;
		uint32_t xorshifted = (uint32_t)(((oldstate >> 18u) ^ oldstate) >> 27u);
		uint32_t rot = (uint32_t)(oldstate >> 59u);
		return (xorshifted >> rot) | (xorshifted << ((~rot + 1u) & 31));
	}

	// Returns a random float.
	float RandomFloat()
	{
		const float rmax = 0x1.fffffep-1;
		float r = RandomInt() * 0x1p-32f;
		return r < rmax ? r : rmax;
	}

	// Advances the random sequence by the given offset, which can be positive or negative.
	void Advance( int64_t offset )
	{
		uint64_t curMult = PCG32_Mult(), curPlus = inc, accMult = 1u;
		uint64_t accPlus = 0u, delta = offset;
		while ( delta > 0 ) {
			if (delta & 1) {
				accMult *= curMult;
				accPlus = accPlus * curMult + curPlus;
			}
			curPlus = (curMult + 1) * curPlus;
			curMult *= curMult;
			delta /= 2;
		}
		state = accMult * state + accPlus;
	}


private:
	uint64_t state, inc;

	constexpr inline uint64_t PCG32_Mult() const { return 0x5851f42d4c957f2dULL; }

	inline uint64_t MixBits(uint64_t v)
	{
		v ^= (v >> 31);
		v *= 0x7fb5d329728ea185;
		v ^= (v >> 27);
		v *= 0x81dadef4bc2dd44d;
		v ^= (v >> 33);
		return v;
	}
};

//-------------------------------------------------------------------------------

inline float Halton( int index, int base )
{
	float r = 0;
	float f = 1.0f / (float)base;
	for ( int i=index; i>0; i/=base ) {
		r += f * (i%base);
		f /= (float) base;
	}
	return r;
}

//-------------------------------------------------------------------------------

// A precomputed Halton Sequence
template <int NUM_SAMPLES>
class HaltonSeq
{
public:
	HaltonSeq() = default;
	HaltonSeq( int base ) { Precompute(base); }
	void Precompute( int base ) { for ( int i=0; i<NUM_SAMPLES; ++i ) seq[i] = Halton(i,base); }
	float operator [] ( int i ) const { return seq[ i % NUM_SAMPLES ]; }
protected:
	float seq[NUM_SAMPLES];
};

//-------------------------------------------------------------------------------

#endif
