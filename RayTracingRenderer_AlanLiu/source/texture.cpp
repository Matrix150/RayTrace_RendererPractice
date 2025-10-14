//-------------------------------------------------------------------------------
///
/// \file       texture.cpp 
/// \author     Cem Yuksel (www.cemyuksel.com)
/// \version    7.1
/// \date       October 6, 2025
///
/// \brief Project source for CS 6620 - University of Utah.
///
/// Copyright (c) 2025 Cem Yuksel. All Rights Reserved.
///
/// This code is provided for educational use only. Redistribution, sharing, or 
/// sublicensing of this code or its derivatives is strictly prohibited.
///
//-------------------------------------------------------------------------------

#include "texture.h"
#include "lodepng.h"

//-------------------------------------------------------------------------------

#if defined(_MSC_VER) && (_MSC_VER >= 1310)
# pragma warning( disable : 4996 ) // Disable the warning message of MSVS about fopen and sscanf not being safe.
#endif

//-------------------------------------------------------------------------------
// Transform from Color(0, 1) to Color24 (0, 255)
inline unsigned char ToByte(float x)
{
	x = x * 255.0f;
	if (x < 0.0f)
		x = 0.0f;
	else if (x > 255.0f)
		x = 255.0f;
	return static_cast<unsigned char>(x + 0.5f);
}
//-------------------------------------------------------------------------------

int ReadLine( FILE *fp, int size, char *buffer )
{
	int i;
	for ( i=0; i<size; i++ ) {
		buffer[i] = fgetc(fp);
		if ( feof(fp) || buffer[i] == '\n' || buffer[i] == '\r' ) {
			buffer[i] = '\0';
			return i+1;
		}
	}
	return i;
}

//-------------------------------------------------------------------------------

bool LoadPPM( FILE *fp, int &width, int &height, std::vector<Color24> &data )
{
	const int bufferSize = 1024;
	char buffer[bufferSize];
	ReadLine(fp,bufferSize,buffer);
	if ( buffer[0] != 'P' && buffer[1] != '6' ) return false;
	
	ReadLine(fp,bufferSize,buffer);
	while ( buffer[0] == '#' ) ReadLine(fp,bufferSize,buffer);	// skip comments
	
	sscanf(buffer,"%d %d",&width,&height);
	
	ReadLine(fp,bufferSize,buffer);
	while ( buffer[0] == '#' ) ReadLine(fp,bufferSize,buffer);	// skip comments

	// last read line should be "255\n"

	data.resize(width*height);
	fread( data.data(), sizeof(Color24), width*height, fp );

	return true;
}

//-------------------------------------------------------------------------------

bool TextureFile::LoadFile()
{
	data.clear();
	width = 0;
	height = 0;
	char const *name = GetName();
	if ( name[0] == '\0' ) return false;

	int len = (int) strlen(name);
	if ( len < 3 ) return false;

	bool success = false;

	char ext[3] = { (char)tolower(name[len-3]), (char)tolower(name[len-2]), (char)tolower(name[len-1]) };

	if ( strncmp(ext,"png",3) == 0 ) {
		std::vector<unsigned char> d;
		unsigned int w, h;
		unsigned int error = lodepng::decode(d,w,h,name,LCT_RGB);
		if ( error == 0 ) {
			width = w;
			height = h;
			data.resize(width*height);
			memcpy( data.data(), d.data(), width*height*3 );
		}
		success = (error == 0);
	} else if ( strncmp(ext,"ppm",3) == 0 ) {
		FILE *fp = fopen( name, "rb" );
		if ( !fp ) return false;
		success = LoadPPM(fp,width,height,data);
		fclose(fp);
	}

	if (success)
		BuildMipmaps();		// Build mipmap

	return success;
}

//-------------------------------------------------------------------------------
// Mipmaps & Filtering
void TextureFile::BuildMipmaps()
{
	mipmaps.clear();
	if (width <= 0 || height <= 0 || data.empty())
		return;

	// Level0
	MipLevel L0;
	L0.w = width;
	L0.h = height;
	L0.texture = data;
	mipmaps.push_back(std::move(L0));

	int w = width, h = height;
	while (w > 1 || h > 1)
	{
		int wtemp = std::max(1, w / 2), htemp = std::max(1, h / 2);
		MipLevel L;
		L.w = wtemp;
		L.h = htemp;
		L.texture.resize(wtemp * htemp);

		auto fetch = [&](int ix, int iy)->Color 
			{
				ix = (ix < 0) ? 0 : ((ix > (w - 1)) ? (w - 1) : ix);
				iy = (iy < 0) ? 0 : ((iy > (h - 1)) ? (h - 1) : iy);
				const Color24 color = mipmaps.back().texture[iy * w + ix];
				return Color(color.r / 255.0f, color.g / 255.0f, color.b / 255.0f);
			};

		for (int y = 0; y < htemp; ++y)
			for (int x = 0; x < wtemp; ++x)
			{
				Color color = (fetch(2 * x, 2 * y) + fetch(2 * x + 1, 2 * y) + fetch(2 * x, 2 * y + 1) + fetch(2 * x + 1, 2 * y + 1)) * 0.25f;
				L.texture[y * wtemp + x] = Color24(ToByte(color.r), ToByte(color.g), ToByte(color.b));
			}
		mipmaps.push_back(std::move(L));
		w = wtemp;
		h = htemp;
	}
}


Color TextureFile::SampleBilinear(int level, float u, float v) const
{
	int size = (int)mipmaps.size();
	level = (level < 0) ? 0 : ((level > size - 1) ? (size - 1) : level);
	const MipLevel& L = mipmaps[level];
	if (L.w == 0 || L.h == 0)
		return Color(0.0f, 0.0f, 0.0f);

	u = u - std::floor(u);
	v = v - std::floor(v);

	float x = u * L.w - 0.5f;
	float y = v * L.h - 0.5f;
	int ix = (int)std::floor(x);
	int iy = (int)std::floor(y);
	float fx = x - ix, fy = y - iy;

	auto texel = [&](int x, int y)->Color 
		{
			x = (x % L.w + L.w) % L.w;
			y = (y % L.h + L.h) % L.h;
			const Color24 color = L.texture[y * L.w + x];
			return Color(color.r / 255.0f, color.g / 255.0f, color.b / 255.0f);
		};

	Color c00 = texel(ix, iy), c10 = texel(ix + 1, iy), c01 = texel(ix, iy + 1), c11 = texel(ix + 1, iy + 1);
	return c00 * ((1 - fx) * (1 - fy)) + c10 * (fx * (1 - fy)) + c01 * ((1 - fx) * fy) + c11 * (fx * fy);
}


Color TextureFile::SampleTrilinear(float u, float v, float lod) const
{
	if (mipmaps.empty())
		return Color(0.0f, 0.0f, 0.0f);

	float size = (float)mipmaps.size();
	lod = (lod < 0.0f) ? 0.0f : ((lod > size - 1) ? (size - 1) : lod);
	int l0 = (int)std::floor(lod);
	int l1 = std::min(l0 + 1, (int)size - 1);
	float t = lod - l0;
	Color color0 = SampleBilinear(l0, u, v);
	Color color1 = SampleBilinear(l1, u, v);

	return color0 * (1.0f - t) + color1 * t;
}


static inline void eigen_2x2(float a, float b, float c, float d, float& sMax, float& sMin, Vec2f& vMajor)
{
	float tr = a + d;
	float det = a * d - b * c;
	float disc = std::max(0.0f, tr * tr * 0.25f - det);
	float lam1 = tr * 0.5f + std::sqrt(disc);
	float lam2 = tr * 0.5f - std::sqrt(disc);
	sMax = std::sqrt(std::max(lam1, 0.0f));
	sMin = std::sqrt(std::max(lam2, 0.0f));

	Vec2f v(b, lam1 - a);
	if (std::abs(v.x) + std::abs(v.y) < 1e-8f)
		v = Vec2f(1, 0);
	float l = std::sqrt(v.x * v.x + v.y * v.y);
	vMajor = (1.0f / l) * v;
}


Color TextureFile::SampleAnisotropic(float u, float v, Vec2f dx, Vec2f dy, int maxAniso) const
{
	if (mipmaps.empty())
		return SampleBilinear(0, u, v);

	float a = dx.x * dx.x + dx.y * dx.y;
	float b = dx.x * dy.x + dx.y * dy.y;
	float d = dy.x * dy.x + dy.y * dy.y;
	float sMax, sMin;
	Vec2f vMajor;

	eigen_2x2(a, b, b, d, sMax, sMin, vMajor);

	float size = (float)mipmaps.size();
	float lod = std::log2(std::max(sMin, 1.0f));
	lod = (lod < 0.0f) ? 0.0f : ((lod > size - 1) ? (size - 1) : lod);
	float temp = sMax / std::max(sMin, 1e-6f);
	float aniso = (temp < 1.0f) ? 1.0f : (temp > (float)maxAniso) ? (float)maxAniso : temp;
	int n = std::max(1, (int)std::ceil(aniso));
	Vec2f step = (1.0f / std::max((float)n, 1.0f)) * vMajor;

	Color color(0.0f, 0.0f, 0.0f);
	float wsum = 0;
	for (int i = 0; i < n; ++i)
	{
		float t = ((i + 0.5f) / (float)n - 0.5f);
		float uu = u + t * step.x;
		float vv = v + t * step.y;
		color += SampleTrilinear(uu, vv, lod);
		wsum += 1.0f;
	}
	return (wsum > 0.0f) ? color * (1.0f / wsum) : SampleTrilinear(u, v, lod);
}


Color TextureFile::Eval(Vec3f const& uvw, Vec3f const duvw[2]) const
{
	if (mipmaps.empty() || width == 0 || height == 0)
		return Texture::Eval(uvw, duvw);

	Vec2f dx(duvw[0].x * width, duvw[0].y * height);
	Vec2f dy(duvw[1].x * width, duvw[1].y * height);

	float lenx = std::sqrt(dx.x * dx.x + dx.y * dx.y);
	float leny = std::sqrt(dy.x * dy.x + dy.y * dy.y);
	float rho = std::max(lenx, leny);
	float lodIso = std::log2(std::max(rho, 1.0f));

	float aratio = std::max(lenx / std::max(leny, 1e-6f), leny / std::max(lenx, 1e-6f));
	float u = uvw.x, v = uvw.y;
	if (aratio > 1.3f)
		return SampleAnisotropic(u, v, dx, dy, 8);
	else
		return SampleTrilinear(u, v, lodIso);
}


//-------------------------------------------------------------------------------

Color TextureFile::Eval(Vec3f const &uvw) const
{
	if ( width + height == 0 ) return Color(0,0,0);

	if (!mipmaps.empty())
		return SampleBilinear(0, TileClamp(uvw).x, TileClamp(uvw).y);

	/*Vec3f u = TileClamp(uvw);
	float x = width * u.x;
	float y = height * u.y;
	int ix = (int)x;
	int iy = (int)y;
	float fx = x - ix;
	float fy = y - iy;

	if ( ix < 0 ) ix -= (ix/width - 1)*width;
	if ( ix >= width ) ix -= (ix/width)*width;
	int ixp = ix+1;
	if ( ixp >= width ) ixp -= width;

	if ( iy < 0 ) iy -= (iy/height - 1)*height;
	if ( iy >= height ) iy -= (iy/height)*height;
	int iyp = iy+1;
	if ( iyp >= height ) iyp -= height;

	return	data[iy *width+ix ].ToColor() * ((1-fx)*(1-fy)) +
			data[iy *width+ixp].ToColor() * (   fx *(1-fy)) +
			data[iyp*width+ix ].ToColor() * ((1-fx)*   fy ) +
			data[iyp*width+ixp].ToColor() * (   fx *   fy );*/
}

//-------------------------------------------------------------------------------

Color TextureChecker::Eval( Vec3f const &uvw ) const
{
	Vec3f u = TileClamp(uvw);
	return color[ ( (u.x <= 0.5f) ^ (u.y <= 0.5f) ) ].Eval(uvw);
}

//-------------------------------------------------------------------------------
