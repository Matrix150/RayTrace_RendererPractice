//-------------------------------------------------------------------------------
///
/// \file       xmlload.h 
/// \author     Cem Yuksel (www.cemyuksel.com)
/// \version    7.0
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

#ifndef _XMLLOAD_H_INCLUDED_
#define _XMLLOAD_H_INCLUDED_

//-------------------------------------------------------------------------------

#include "tinyxml2.h"

#include "cyVector.h"
#include "cyMatrix.h"
#include "cyColor.h"
using namespace cy;

//-------------------------------------------------------------------------------

class Loader
{
	tinyxml2::XMLElement *elem;

public:
	Loader( tinyxml2::XMLElement *x ) : elem(x) {}

	bool operator == ( char const *name ) const { return Tag() == name; }

	class String
	{
		char const *s;
	public:
		String( char const *str ) : s(str) {}
		operator const char* () const { return s; }
		bool operator == ( char const *v ) const
		{
			if ( s == v ) return true;
			if ( !s || !v ) return false;
			for ( int i=0; tolower(s[i])==tolower(v[i]); ++i ) if ( s[i] == '\0' ) return true;
			return false;
		}
	};

	String Tag() const { return elem->Value(); }
	String Attribute( char const *name ) const { char const *s = nullptr; elem->QueryStringAttribute( name, &s ); return String(s); }

	bool ReadFloat( float &f, char const *name="value" ) const { return elem && elem->QueryFloatAttribute( name, &f ) == tinyxml2::XML_SUCCESS; }
	bool ReadInt  ( int   &i, char const *name="value" ) const { return elem && elem->QueryIntAttribute  ( name, &i ) == tinyxml2::XML_SUCCESS; }
	void ReadVec3f( Vec3f &v, Vec3f const &def=Vec3f(0,0,0) ) const { if (elem) { v=def; ReadFloat(v.x,"x"); ReadFloat(v.y,"y"); ReadFloat(v.z,"z"); float f=1; if ( ReadFloat(f) ) v *= f; } }
	void ReadColor( Color &c, Color const &def=Color(1,1,1) ) const { if (elem) { c=def; ReadFloat(c.r,"r"); ReadFloat(c.g,"g"); ReadFloat(c.b,"b"); float f=1; if ( ReadFloat(f) ) c *= f; } }
	void ReadTexturedColor( TexturedColor &tc, TextureFileList &tfl ) const { if (elem) { Color c(1,1,1); ReadColor(c); tc.SetValue(c); tc.SetTexture(ReadTextureMap(tfl)); } }
	void ReadTexturedFloat( TexturedFloat &tc, TextureFileList &tfl ) const { if (elem) { float f = 1.0f; ReadFloat(f); tc.SetValue(f); tc.SetTexture(ReadTextureMap(tfl)); } }

	TextureMap* ReadTextureMap( TextureFileList &tfl ) const;

	Loader const Child( char const *name ) const { return Loader( elem ? elem->FirstChildElement(name) : nullptr ); }

	class iterator
	{
		tinyxml2::XMLElement *e;
	public:
		iterator( tinyxml2::XMLElement *elem ) : e(elem) {}
		iterator&    operator ++ ()                       { e = e->NextSiblingElement(); return *this; }
		iterator     operator ++ ( int )                  { iterator retval = *this; ++(*this); return retval; }
		bool         operator == ( iterator other ) const { return e == other.e; }
		bool         operator != ( iterator other ) const { return !(*this == other); }
		Loader const operator *  ()                 const { return Loader(e); }
	};

	iterator begin() const { return iterator(elem->FirstChildElement()); }
	iterator end()   const { return iterator(nullptr); }
};

//-------------------------------------------------------------------------------

#endif