//-------------------------------------------------------------------------------
///
/// \file       texture.h 
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

#ifndef _TEXTURE_H_INCLUDED_
#define _TEXTURE_H_INCLUDED_

#include "scene.h"

//-------------------------------------------------------------------------------

class TextureFile : public Texture
{
public:
	Color Eval( Vec3f const &uvw ) const override;
	bool  SetViewportTexture()     const override;
	bool  LoadFile();
private:
	std::vector<Color24> data;
	int width  = 0;
	int height = 0;
	mutable unsigned int viewportTextureID = 0;
};

//-------------------------------------------------------------------------------

class TextureChecker : public Texture
{
public:
	Color Eval( Vec3f const &uvw ) const override;
	bool  SetViewportTexture()     const override;
	void  Load( Loader const &loader, TextureFileList &textureFileList ) override;
private:
	TexturedColor color[2] = { Color(0,0,0), Color(1,1,1) };
	mutable unsigned int viewportTextureID = 0;
};

//-------------------------------------------------------------------------------

#endif