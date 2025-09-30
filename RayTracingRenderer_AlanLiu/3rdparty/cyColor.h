// cyCodeBase by Cem Yuksel
// [www.cemyuksel.com]
//-------------------------------------------------------------------------------
//! \file   cyColor.h 
//! \author Cem Yuksel
//! 
//! \brief  Color classes.
//!
//-------------------------------------------------------------------------------
//
// Copyright (c) 2016, Cem Yuksel <cem@cemyuksel.com>
// All rights reserved.
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy 
// of this software and associated documentation files (the "Software"), to deal 
// in the Software without restriction, including without limitation the rights 
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell 
// copies of the Software, and to permit persons to whom the Software is 
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all 
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR 
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, 
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER 
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE 
// SOFTWARE.
// 
//-------------------------------------------------------------------------------

#ifndef _CY_COLOR_H_INCLUDED_
#define _CY_COLOR_H_INCLUDED_

//-------------------------------------------------------------------------------

#include "cyCore.h"

//-------------------------------------------------------------------------------
namespace cy {
//-------------------------------------------------------------------------------

class ColorA;
class Color24;
class Color32;

//-------------------------------------------------------------------------------

//! RGB color class with 3 float components

class Color
{
	CY_NODISCARD friend Color operator+( float const& v, Color const &c ) { return   c+v;  }	//!< Addition with a constant
	CY_NODISCARD friend Color operator-( float const& v, Color const &c ) { return -(c-v); }	//!< Subtraction from a constant
	CY_NODISCARD friend Color operator*( float const& v, Color const &c ) { return   c*v;  }	//!< Multiplication with a constant

public:

	//!@name Color components
	float r, g, b;

	//!@name Constructors
	Color() CY_CLASS_FUNCTION_DEFAULT
	Color( Color const &c ) : r(c.r), g(c.g), b(c.b) {}
	explicit Color( float _r, float _g, float _b ) : r(_r), g(_g), b(_b) {}
	explicit Color( float const *c ) : r(c[0]), g(c[1]), b(c[2]) {}
	explicit Color( float rgb ) : r(rgb), g(rgb), b(rgb) {}
	explicit Color( ColorA  const &c );
	explicit Color( Color24 const &c );
	explicit Color( Color32 const &c );

	//!@name Set & Get value methods
	void SetBlack() { r=0.0f; g=0.0f; b=0.0f; }							//!< Sets r, g and b components as zero.
	void SetWhite() { r=1.0f; g=1.0f; b=1.0f; }							//!< Sets r, g and b components as one.
	void Set     ( float _r, float _g, float _b ) { r=_r; g=_g; b=_b; }	//!< Sets r, g and b components as given.
	void Set     ( float const *v ) { r=v[0]; g=v[1]; b=v[2]; }			//!< Sets r, g and b components using the values in the given array.
	void GetValue( float *v ) const { v[0]=r; v[1]=g; v[2]=b; }			//!< Puts r, g and b values into the array.

	//!@name Gray-scale methods
	CY_NODISCARD float Sum  () const { return r + g + b; }
	CY_NODISCARD float Gray () const { return Sum() / 3.0f; }
	CY_NODISCARD float Luma1() const { return 0.299f *r + 0.587f *g + 0.114f *b; }
	CY_NODISCARD float Luma2() const { return 0.2126f*r + 0.7152f*g + 0.0722f*b; }
	CY_NODISCARD float Min  () const { return r<g ? (r<b ? r : b) : (g<b ? g : b); }
	CY_NODISCARD float Max  () const { return r>g ? (r>b ? r : b) : (g>b ? g : b); }

	//!@name General methods
	CY_NODISCARD bool IsNegative() const { return r<0 || g<0 || b<0; }				//!< Returns true if any component is negative.
	CY_NODISCARD bool IsBlack   () const { return r==0.0f && g==0.0f && b==0.0f; }	//!< Returns true if all components are exactly zero.
	CY_NODISCARD bool IsFinite  () const { return cy::IsFinite(r) && cy::IsFinite(g) && cy::IsFinite(b); }	//!< Returns true if all components are finite real numbers.

	CY_NODISCARD Color Linear2sRGB() const { auto f=[](float cl){ return cl<0.0031308f ? cl*12.92f : powf(cl,0.41666f)*1.055f-0.055f; }; return Color(f(r),f(g),f(b)); }	//!< Converts linear RGB to sRGB.
	CY_NODISCARD Color sRGB2Linear() const { auto f=[](float cs){ return cs<=0.04045f  ? cs/12.92f : powf( (cs+0.055f)/1.055f, 2.4f); }; return Color(f(r),f(g),f(b)); }	//!< Converts sRGB to linear RGB.

	//!@name Generic template methods
	template <typename F = float(*)(float)>
	void Apply( F func ) { r=func(r); g=func(g); b=func(b); }	//!< Applies the given function to all color components.
	template <typename F = float(*)(float)>
	CY_NODISCARD Color GetApplied( F func ) const { Color c=*this; c.Apply<F>(func); return c; }	//!< Returns the resulting color after applying the given function to all color components.

	//!@name Limit methods
	void Clamp   ( float limitMin=0.0f, float limitMax=1.0f ) { ClampMin(limitMin); ClampMax(limitMax); }
	void ClampMin( float limitMin=0.0f ) { Apply([&limitMin](float v){ return cy::Max(v,limitMin); }); }
	void ClampMax( float limitMax=1.0f ) { Apply([&limitMax](float v){ return cy::Min(v,limitMax); }); }
	void Abs() { Apply(std::abs); }

	//!@name Unary operators
	CY_NODISCARD Color  operator - () const { return Color(-r,-g,-b); } 

	//!@name Binary operators
	CY_NODISCARD Color operator + ( Color const &c ) const { return Color(r+c.r, g+c.g, b+c.b); }
	CY_NODISCARD Color operator - ( Color const &c ) const { return Color(r-c.r, g-c.g, b-c.b); }
	CY_NODISCARD Color operator * ( Color const &c ) const { return Color(r*c.r, g*c.g, b*c.b); }
	CY_NODISCARD Color operator / ( Color const &c ) const { return Color(r/c.r, g/c.g, b/c.b); }
	CY_NODISCARD Color operator + ( float const &n ) const { return Color(r+n, g+n, b+n); }
	CY_NODISCARD Color operator - ( float const &n ) const { return Color(r-n, g-n, b-n); }
	CY_NODISCARD Color operator * ( float const &n ) const { return Color(r*n, g*n, b*n); }
	CY_NODISCARD Color operator / ( float const &n ) const { return Color(r/n, g/n, b/n); }

	//!@name Assignment operators
	Color& operator += ( Color const &c ) { r+=c.r; g+=c.g; b+=c.b; return *this; }
	Color& operator -= ( Color const &c ) { r-=c.r; g-=c.g; b-=c.b; return *this; }
	Color& operator *= ( Color const &c ) { r*=c.r; g*=c.g; b*=c.b; return *this; }
	Color& operator /= ( Color const &c ) { r/=c.r; g/=c.g; b/=c.b; return *this; }
	Color& operator += ( float const &n ) { r+=n; g+=n; b+=n; return *this; }
	Color& operator -= ( float const &n ) { r-=n; g-=n; b-=n; return *this; }
	Color& operator *= ( float const &n ) { r*=n; g*=n; b*=n; return *this; }
	Color& operator /= ( float const &n ) { r/=n; g/=n; b/=n; return *this; }

	//!@name Test operators
	CY_NODISCARD bool operator == ( Color const &c ) const { return ( (c.r==r) && (c.g==g) && (c.b==b) ); }
	CY_NODISCARD bool operator != ( Color const &c ) const { return ( (c.r!=r) || (c.g!=g) || (c.b!=b) ); }

	//!@name Access operators
	CY_NODISCARD float& operator [] ( int i )       { return (&r)[i]; }
	CY_NODISCARD float  operator [] ( int i ) const { return (&r)[i]; }

	//!@name Static methods
	CY_NODISCARD static Color Black() { return Color(0.0f,0.0f,0.0f); }	//!< Returns a black color.
	CY_NODISCARD static Color White() { return Color(1.0f,1.0f,1.0f); }	//!< Returns a white color.
};

//-------------------------------------------------------------------------------

//! RGBA color class with 4 float components

class ColorA
{
	CY_NODISCARD friend ColorA operator + ( float const &v, ColorA const &c ) { return   c+v;  }	//!< Addition with a constant
	CY_NODISCARD friend ColorA operator - ( float const &v, ColorA const &c ) { return -(c-v); }	//!< Subtraction from a constant
	CY_NODISCARD friend ColorA operator * ( float const &v, ColorA const &c ) { return   c*v;  }	//!< Multiplication with a constant

public:

	//!@name Color components
	float r, g, b, a;

	//!@name Constructors
	ColorA() CY_CLASS_FUNCTION_DEFAULT
	ColorA( ColorA const &c ) : r(c.r), g(c.g), b(c.b), a(c.a) {}
	explicit ColorA( float _r, float _g, float _b, float _a=1 ) : r(_r), g(_g), b(_b), a(_a) {}
	explicit ColorA( float const *c ) : r(c[0]), g(c[1]), b(c[2]), a(c[3]) {}
	explicit ColorA( float rgb, float _a=1 ) : r(rgb), g(rgb), b(rgb), a(_a) {}
	explicit ColorA( Color   const &c, float _a=1 ) : r(c.r), g(c.g), b(c.b), a(_a) {}
	explicit ColorA( Color24 const &c, float _a=1 );
	explicit ColorA( Color32 const &c );

	//!@name Set & Get value methods
	void SetBlack( float alpha=1.0f ) { r=0.0f; g=0.0f; b=0.0f; a=alpha; }					//!< Sets r, g, and b components as zero and a component as given.
	void SetWhite( float alpha=1.0f ) { r=1.0f; g=1.0f; b=1.0f; a=alpha; }					//!< Sets r, g, and b components as one and a component as given.
	void Set     ( float _r, float _g, float _b, float _a=1 ) { r=_r; g=_g; b=_b; a=_a; }	//!< Sets r, g, b and a components as given.
	void Set     ( float const *v ) { r=v[0]; g=v[1]; b=v[2]; a=v[3]; }						//!< Sets r, g, b and a components using the values in the given array.
	void GetValue( float *v ) const { v[0]=r; v[1]=g; v[2]=b; v[3]=a; }						//!< Puts r, g, b and a values into the array.

	//!@name Gray-scale methods
	CY_NODISCARD float Sum  () const { return r + g + b; }
	CY_NODISCARD float Gray () const { return Sum() / 3.0f; }
	CY_NODISCARD float Luma1() const { return 0.299f *r + 0.587f *g + 0.114f *b; }
	CY_NODISCARD float Luma2() const { return 0.2126f*r + 0.7152f*g + 0.0722f*b; }
	CY_NODISCARD float Min  () const { float mrg = r<g ? r : g; float mba = b<a ? b : a; return mrg<mba ? mrg : mba; }
	CY_NODISCARD float Max  () const { float mrg = r>g ? r : g; float mba = b>a ? b : a; return mrg>mba ? mrg : mba; }

	//!@name General methods
	CY_NODISCARD bool IsNegative() const { return r<0 || g<0 || b<0 || a<0; }		//!< Returns true if any component is negative.
	CY_NODISCARD bool IsBlack   () const { return r==0.0f && g==0.0f && b==0.0f; }	//!< Returns true if the r, g, and b components are exactly zero.
	CY_NODISCARD bool IsFinite  () const { return cy::IsFinite(r) && cy::IsFinite(g) && cy::IsFinite(b) && cy::IsFinite(a); }	//!< Returns true if all components are finite real numbers.

	CY_NODISCARD ColorA Linear2sRGB() const { return ColorA(Color(r,g,b).Linear2sRGB(),a); }	//!< Converts linear RGB to sRGB.
	CY_NODISCARD ColorA sRGB2Linear() const { return ColorA(Color(r,g,b).sRGB2Linear(),a); }	//!< Converts sRGB to linear RGB.

	//!@name Generic template methods
	template <typename F = float(*)(float)>
	void Apply( F func ) { r=func(r); g=func(g); b=func(b); }	//!< Applies the given function to all color components.
	template <typename F = float(*)(float)>
	CY_NODISCARD ColorA GetApplied( F func ) const { ColorA c=*this; c.Apply<F>(func); return c; }	//!< Returns the resulting color after applying the given function to all color components.

	//!@name Limit methods
	void Clamp   ( float limitMin=0.0f, float limitMax=1.0f ) { ClampMin(limitMin); ClampMax(limitMax); }
	void ClampMin( float limitMin=0.0f ) { Apply([&limitMin](float v){ return cy::Max(v,limitMin); }); }
	void ClampMax( float limitMax=1.0f ) { Apply([&limitMax](float v){ return cy::Min(v,limitMax); }); }
	void Abs() { Apply(std::abs); }

	//!@name Unary operators
	CY_NODISCARD ColorA  operator - () const { return ColorA(-r,-g,-b,-a); } 

	//!@name Binary operators
	CY_NODISCARD ColorA operator + ( ColorA const &c ) const { return ColorA(r+c.r, g+c.g, b+c.b, a+c.a); }
	CY_NODISCARD ColorA operator - ( ColorA const &c ) const { return ColorA(r-c.r, g-c.g, b-c.b, a-c.a); }
	CY_NODISCARD ColorA operator * ( ColorA const &c ) const { return ColorA(r*c.r, g*c.g, b*c.b, a*c.a); }
	CY_NODISCARD ColorA operator / ( ColorA const &c ) const { return ColorA(r/c.r, g/c.g, b/c.b, a/c.a); }
	CY_NODISCARD ColorA operator + ( float  const &n ) const { return ColorA(r+n, g+n, b+n, a+n); }
	CY_NODISCARD ColorA operator - ( float  const &n ) const { return ColorA(r-n, g-n, b-n, a-n); }
	CY_NODISCARD ColorA operator * ( float  const &n ) const { return ColorA(r*n, g*n, b*n, a*n); }
	CY_NODISCARD ColorA operator / ( float  const &n ) const { return ColorA(r/n, g/n, b/n, a/n); }

	//!@name Assignment operators
	ColorA& operator += ( ColorA const &c ) { r+=c.r; g+=c.g; b+=c.b; a+=c.a; return *this; }
	ColorA& operator -= ( ColorA const &c ) { r-=c.r; g-=c.g; b-=c.b; a-=c.a; return *this; }
	ColorA& operator *= ( ColorA const &c ) { r*=c.r; g*=c.g; b*=c.b; a*=c.a; return *this; }
	ColorA& operator /= ( ColorA const &c ) { r/=c.r; g/=c.g; b/=c.b; a/=c.a; return *this; }
	ColorA& operator += ( float  const &n ) { r+=n; g+=n; b+=n; a+=n; return *this; }
	ColorA& operator -= ( float  const &n ) { r-=n; g-=n; b-=n; a-=n; return *this; }
	ColorA& operator *= ( float  const &n ) { r*=n; g*=n; b*=n; a*=n; return *this; }
	ColorA& operator /= ( float  const &n ) { r/=n; g/=n; b/=n; a/=n; return *this; }

	//!@name Test operators
	CY_NODISCARD bool operator == ( ColorA const &c ) const { return ( (c.r==r) && (c.g==g) && (c.b==b) && (c.a==a) ); }
	CY_NODISCARD bool operator != ( ColorA const &c ) const { return ( (c.r!=r) || (c.g!=g) || (c.b!=b) || (c.a!=a) ); }

	//!@name Access operators
	CY_NODISCARD float& operator [] ( int i )       { return (&r)[i]; }
	CY_NODISCARD float  operator [] ( int i ) const { return (&r)[i]; }

	//!@name Static methods
	CY_NODISCARD static ColorA Black( float alpha=1.0f ) { return ColorA(0.0f,0.0f,0.0f,alpha); }	//!< Returns a black color.
	CY_NODISCARD static ColorA White( float alpha=1.0f ) { return ColorA(1.0f,1.0f,1.0f,alpha); }	//!< Returns a white color.
};

//-------------------------------------------------------------------------------

//! 24-bit RGB color class with 3 unsigned byte components

class Color24
{
public:

	//!@name Color components
	uint8_t r, g, b;

	//!@name Constructors
	Color24() CY_CLASS_FUNCTION_DEFAULT
	Color24( Color24 const &c ) : r(c.r), g(c.g), b(c.b) {}
	explicit Color24( uint8_t _r, uint8_t _g, uint8_t _b ) : r(_r), g(_g), b(_b) {}
	explicit Color24( Color   const &c ) { r=FloatToByte(c.r); g=FloatToByte(c.g); b=FloatToByte(c.b); }
	explicit Color24( ColorA  const &c ) { r=FloatToByte(c.r); g=FloatToByte(c.g); b=FloatToByte(c.b); }
	explicit Color24( Color32 const &c );

	//!@name Conversion methods
	CY_NODISCARD Color  ToColor () const { return Color (r/255.0f,g/255.0f,b/255.0f); }
	CY_NODISCARD ColorA ToColorA() const { return ColorA(r/255.0f,g/255.0f,b/255.0f,1.0f); }

	//!@name Set & Get value methods
	void SetBlack() { r=  0; g=  0; b=  0; }									//!< Sets r, g, and b components as zero.
	void SetWhite() { r=255; g=255; b=255; }									//!< Sets r, g, and b components as 255.
	void Set     ( uint8_t _r, uint8_t _g, uint8_t _b ) { r=_r; g=_g; b=_b; }	//!< Sets r, g, and b components as given.
	void Set     ( uint8_t const *v ) { r=v[0]; g=v[1]; b=v[2]; }				//!< Sets r, g, and b components using the values in the given array.
	void GetValue( uint8_t *v ) const { v[0]=r; v[1]=g; v[2]=b; }				//!< Puts r, g, and b values into the array.

	//!@name Gray-scale methods
	CY_NODISCARD int     Sum () const { return int(r) + int(g) + int(b); }
	CY_NODISCARD uint8_t Gray() const { return uint8_t( (Sum()+1) / 3 ); }
	CY_NODISCARD uint8_t Min () const { return r<g ? (r<b ? r : b) : (g<b ? g : b); }
	CY_NODISCARD uint8_t Max () const { return r>g ? (r>b ? r : b) : (g>b ? g : b); }

	//!@name General methods
	CY_NODISCARD bool IsBlack() const { return r==0 && g==0 && b==0; }	//!< Returns true if all components are exactly zero.

	//!@name Limit methods
	void Clamp   ( uint8_t limitMin=  0, uint8_t limitMax=255 ) { ClampMin(limitMin); ClampMax(limitMax); }
	void ClampMin( uint8_t limitMin=  0 ) { r=cy::Max(r,limitMin); g=cy::Max(g,limitMin); b=cy::Max(b,limitMin); }
	void ClampMax( uint8_t limitMax=255 ) { r=cy::Min(r,limitMax); g=cy::Min(g,limitMax); b=cy::Min(b,limitMax); }

	//!@name Test operators
	CY_NODISCARD bool operator == ( Color24 const &c ) const { return ( (c.r==r) && (c.g==g) && (c.b==b) ); }
	CY_NODISCARD bool operator != ( Color24 const &c ) const { return ( (c.r!=r) || (c.g!=g) || (c.b!=b) ); }

	//!@name Access operators
	CY_NODISCARD uint8_t& operator [] ( int i )       { return (&r)[i]; }
	CY_NODISCARD uint8_t  operator [] ( int i ) const { return (&r)[i]; }

	//!@name Static methods
	CY_NODISCARD static Color24 Black() { return Color24(  0,  0,  0); }	//!< Returns a black color.
	CY_NODISCARD static Color24 White() { return Color24(255,255,255); }	//!< Returns a white color.

protected:
	CY_NODISCARD static uint8_t FloatToByte(float r) { return ClampInt(int(r*255+0.5f)); }
	CY_NODISCARD static uint8_t ClampInt(int v) { return v<0 ? 0 : (v>255 ? 255 : static_cast<uint8_t>(v)); }
};

//-------------------------------------------------------------------------------

//! 32-bit RGBA color class with 4 unsigned byte components

class Color32
{
public:

	//!@name Color components
	uint8_t r, g, b, a;

	//!@name Constructors
	Color32() CY_CLASS_FUNCTION_DEFAULT
	Color32( Color32 const &c ) : r(c.r), g(c.g), b(c.b), a(c.a) {}
	explicit Color32( uint8_t _r, uint8_t _g, uint8_t _b, uint8_t _a=255 ) : r(_r), g(_g), b(_b), a(_a) {}
	explicit Color32( Color   const &c, float _a=1.0f ) { r=FloatToByte(c.r); g=FloatToByte(c.g); b=FloatToByte(c.b); a=FloatToByte( _a); }
	explicit Color32( ColorA  const &c )                { r=FloatToByte(c.r); g=FloatToByte(c.g); b=FloatToByte(c.b); a=FloatToByte(c.a); }
	explicit Color32( Color24 const &c, uint8_t _a=255 ) : r(c.r), g(c.g), b(c.b), a(_a) {}

	//!@name Conversion methods
	CY_NODISCARD Color  ToColor () const { return Color (r/255.0f,g/255.0f,b/255.0f); }
	CY_NODISCARD ColorA ToColorA() const { return ColorA(r/255.0f,g/255.0f,b/255.0f,a/255.0f); }

	//!@name Set & Get value methods
	void SetBlack( uint8_t _a=255 ) { r=  0; g=  0; b=  0; a=_a; }								//!< Sets r, g, and b components as zero and a component as given.
	void SetWhite( uint8_t _a=255 ) { r=255; g=255; b=255; a=_a; }								//!< Sets r, g, and b components as 255 and a component as given.
	void Set     ( uint8_t _r, uint8_t _g, uint8_t _b, uint8_t _a ) { r=_r; g=_g; b=_b; a=_a; }	//!< Sets r, g, b and a components as given.
	void Set     ( uint8_t const *v ) { r=v[0]; g=v[1]; b=v[2]; a=v[3]; }						//!< Sets r, g, b and a components using the values in the given array.
	void GetValue( uint8_t *v ) const { v[0]=r; v[1]=g; v[2]=b; v[3]=a; }						//!< Puts r, g, b and a values into the array.

	//!@name Gray-scale methods
	CY_NODISCARD int     Sum () const { return int(r) + int(g) + int(b); }
	CY_NODISCARD uint8_t Gray() const { return uint8_t( (Sum()+1) / 3 ); }
	CY_NODISCARD uint8_t Min () const { uint8_t mrg = r<g ? r : g; uint8_t mba = b<a ? b : a; return mrg<mba ? mrg : mba; }
	CY_NODISCARD uint8_t Max () const { uint8_t mrg = r>g ? r : g; uint8_t mba = b>a ? b : a; return mrg>mba ? mrg : mba; }

	//!@name General methods
	CY_NODISCARD bool IsBlack() const { return r==0 && g==0 && b==0; }	//!< Returns true if the r, g, and b components are exactly zero.

	//!@name Limit methods
	void Clamp   ( uint8_t limitMin=  0, uint8_t limitMax=255 ) { ClampMin(limitMin); ClampMax(limitMax); }
	void ClampMin( uint8_t limitMin=  0 ) { r=cy::Max(r,limitMin); g=cy::Max(g,limitMin); b=cy::Max(b,limitMin); a=cy::Max(a,limitMin); }
	void ClampMax( uint8_t limitMax=255 ) { r=cy::Min(r,limitMax); g=cy::Min(g,limitMax); b=cy::Min(b,limitMax); a=cy::Min(a,limitMax); }

	//!@name Test operators
	CY_NODISCARD bool operator == ( Color32 const &c ) const { return ( (c.r==r) && (c.g==g) && (c.b==b) && (c.a==a) ); }
	CY_NODISCARD bool operator != ( Color32 const &c ) const { return ( (c.r!=r) || (c.g!=g) || (c.b!=b) || (c.a!=a) ); }

	//!@name Access operators
	CY_NODISCARD uint8_t& operator [] ( int i )       { return (&r)[i]; }
	CY_NODISCARD uint8_t  operator [] ( int i ) const { return (&r)[i]; }

	//!@name Static methods
	CY_NODISCARD static Color32 Black( uint8_t alpha=255 ) { return Color32(  0,  0,  0,alpha); }	//!< Returns a black color.
	CY_NODISCARD static Color32 White( uint8_t alpha=255 ) { return Color32(255,255,255,alpha); }	//!< Returns a white color.

protected:
	CY_NODISCARD static uint8_t FloatToByte(float r) { return ClampInt(int(r*255+0.5f)); }
	CY_NODISCARD static uint8_t ClampInt(int v) { return v<0 ? 0 : (v>255 ? 255 : static_cast<uint8_t>(v)); }
};

//-------------------------------------------------------------------------------

//!@name Common math functions

inline Color  Abs  ( Color  const &c ) { return c.GetApplied(std::abs  ); }	//!< Returns a color with abs applied to all components.
inline Color  Exp  ( Color  const &c ) { return c.GetApplied(std::exp  ); }	//!< Returns a color with exponential applied to all components.
inline Color  Exp2 ( Color  const &c ) { return c.GetApplied(std::exp2 ); }	//!< Returns a color with base-2 exponential applied to all components.
inline Color  Log  ( Color  const &c ) { return c.GetApplied(std::log  ); }	//!< Returns a color with natural logarithm applied to all components.
inline Color  Log2 ( Color  const &c ) { return c.GetApplied(std::log2 ); }	//!< Returns a color with base-2 logarithm applied to all components.
inline Color  Log10( Color  const &c ) { return c.GetApplied(std::log10); }	//!< Returns a color with base-10 logarithm applied to all components.
inline Color  Sqrt ( Color  const &c ) { return c.GetApplied(cy::Sqrt<float>); }	//!< Returns a color with square root applied to all components.
inline Color  Pow  ( Color  const &c, float exponent ) { return c.GetApplied([&exponent](float v){ return std::pow(v,exponent); }); }	//!< Returns a color with square root applied to all components.

inline ColorA Abs  ( ColorA const &c ) { return c.GetApplied(std::abs  ); }	//!< Returns a color with abs applied to all components.
inline ColorA Exp  ( ColorA const &c ) { return c.GetApplied(std::exp  ); }	//!< Returns a color with exponential applied to all components.
inline ColorA Exp2 ( ColorA const &c ) { return c.GetApplied(std::exp2 ); }	//!< Returns a color with base-2 exponential applied to all components.
inline ColorA Log  ( ColorA const &c ) { return c.GetApplied(std::log  ); }	//!< Returns a color with natural logarithm applied to all components.
inline ColorA Log2 ( ColorA const &c ) { return c.GetApplied(std::log2 ); }	//!< Returns a color with base-2 logarithm applied to all components.
inline ColorA Log10( ColorA const &c ) { return c.GetApplied(std::log10); }	//!< Returns a color with base-10 logarithm applied to all components.
inline ColorA Sqrt ( ColorA const &c ) { return c.GetApplied(cy::Sqrt<float>); }	//!< Returns a color with square root applied to all components.
inline ColorA Pow  ( ColorA const &c, float exponent ) { return c.GetApplied([&exponent](float v){ return std::pow(v,exponent); }); }	//!< Returns a color with square root applied to all components.

//-------------------------------------------------------------------------------

inline Color  ::Color  ( ColorA  const &c ) : r(c.r), g(c.g), b(c.b) {}
inline Color  ::Color  ( Color24 const &c ) : Color( c.ToColor() ) {}
inline Color  ::Color  ( Color32 const &c ) : Color( c.ToColor() ) {}
inline ColorA ::ColorA ( Color24 const &c, float alpha ) { Color rgb = c.ToColor(); r = rgb.r; g = rgb.g; b = rgb.b; a = alpha; }
inline ColorA ::ColorA ( Color32 const &c ) : ColorA( c.ToColorA() ) {}
inline Color24::Color24( Color32 const &c ) : r(c.r), g(c.g), b(c.b) {}

//-------------------------------------------------------------------------------
} // namespace cy
//-------------------------------------------------------------------------------

typedef cy::Color   cyColor;	//!< RGB color class with 3 float components
typedef cy::ColorA  cyColorA;	//!< RGBA color class with 4 float components
typedef cy::Color24 cyColor24;	//!< 24-bit RGB color class with 3 unsigned byte components
typedef cy::Color32 cyColor32;	//!< 32-bit RGBA color class with 4 unsigned byte components

//-------------------------------------------------------------------------------

#endif

