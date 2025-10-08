#include "materials.h"
#include "lights.h"



static inline float Saturate(float x)
{
	return x < 0.0f ? 0.0f : (x > 1.0f ? 1.0f : x);
}

static inline float FresnelSchlick(float cosTheta, float nIn, float nOut)
{
	cosTheta = std::fabs(cosTheta);
	const float F0 = std::pow((nIn - nOut) / (nIn + nOut), 2.0f);

	return F0 + (1.0f - F0) * std::pow(1.0f - cosTheta, 5.0f);
}

static inline Vec3f Reflect(const Vec3f& V, const Vec3f& N)
{
	return (2.0f * (N % V)) * N - V;
}

static inline bool Refract(const Vec3f& V, const Vec3f& N_geo, bool isFront, float ior, Vec3f& T_out, float& cosi_out, float& eta_out)
{
	float cosi = isFront ? (V % N_geo) : -(V % N_geo);
	Vec3f N = isFront ? N_geo : -(N_geo);
	const float nIn = isFront ? 1.0f : ior;
	const float nOut = isFront ? ior : 1.0f;

	const float eta = nIn / nOut;
	const float k = 1.0f - eta * eta * (1.0f - cosi * cosi);

	if (k < 0.0f)
		return false;		// Total internal reflection

	const float cost = std::sqrt(k);
	T_out = ((-eta) * V - (cost - eta * cosi) * N).GetNormalized();
	cosi_out = cosi;
	eta_out = eta;
	return true;
}

static inline Color Absorb(Color const& sigma, float dist)
{
	return Color(std::expf(-sigma.r * dist), std::expf(-sigma.g * dist), std::expf(-sigma.b * dist));
}


// Shade model
Color MtlPhong::Shade(ShadeInfo const& shadeInfo) const
{
	const Vec3f N = shadeInfo.N().GetNormalized();		// normal
	const Vec3f V = shadeInfo.V().GetNormalized();		// viewing
	Color color(0.0f, 0.0f, 0.0f);

	const int lightNum = shadeInfo.NumLights();	// light numbers
	// Loop all lights
	for (int i = 0; i < lightNum; ++i)
	{
		const Light* light = shadeInfo.GetLight(i);
		Vec3f L;
		Color lightColor = light->Illuminate(shadeInfo, L);		// Get light intensity and direction(vector L)
		L.Normalize();

		// Add ambient light
		if (light->IsAmbient())
		{
			color += Diffuse() * lightColor;
			continue;
		}

		const float NdotL = std::max(0.0f, N % L);
		if (NdotL > 0.0f)
		{
			// Add diffuse
			color += Diffuse() * lightColor * NdotL;

			Vec3f R = (2.0f * NdotL) * N - L;		// Reflection
			R.Normalize();
			const float RdotV = std::max(0.0f, R % V);
			// Add specular
			color += Specular() * lightColor * std::pow(RdotV, Glossiness());
		}
	}

	// Saturate the color
	color.Clamp();

	return color;
}


Color MtlBlinn::Shade(ShadeInfo const& shadeInfo) const
{
	const Vec3f N = shadeInfo.N().GetNormalized();		// normal
	const Vec3f V = shadeInfo.V().GetNormalized();		// viewing
	Color color(0.0f, 0.0f, 0.0f);

	const int lightNum = shadeInfo.NumLights();	// light numbers
	// Loop all lights
	for (int i = 0; i < lightNum; ++i)
	{
		const Light* light = shadeInfo.GetLight(i);
		Vec3f L;
		Color lightColor = light->Illuminate(shadeInfo, L);		// Get light intensity and direction(vector L)
		L.Normalize();

		// Add ambient light
		if (light->IsAmbient())
		{
			color += Diffuse() * lightColor;
			continue;
		}

		const float NdotL = std::max(0.0f, N % L);
		if (NdotL > 0.0f)
		{
			// Add diffuse
			color += Diffuse() * lightColor * NdotL;
			Vec3f H = (L + V).GetNormalized();		// halfway vector H
			const float NdotH = std::max(0.0f, N % H);
			// Add Specular
			color += Specular() * lightColor * std::pow(NdotH, Glossiness());
		}
	}

	if (shadeInfo.CanBounce())
	{
		constexpr float kEps = 1e-4f;

		const Color kr = Reflection();
		const Color kt = Refraction();
		const Color sigma = Absorption();
		const float ior = IOR();

		const bool hasKt = (kt.r > 0.0f || kt.g > 0.0f || kt.b > 0.0f);
		const bool hasKr = (kr.r > 0.0f || kr.g > 0.0f || kr.b > 0.0f) || hasKt;

		const float nIn = shadeInfo.IsFront() ? 1.0f : ior;
		const float	nOut =shadeInfo.IsFront() ? ior : 1.0f;
		float cosi = std::fabs(N % V);
		const float F = FresnelSchlick(cosi, nIn, nOut);

		Vec3f Tdir;
		float eta = 1.0f;
		bool refractionDone = false;

		if (hasKt)
			refractionDone = Refract(V, N, shadeInfo.IsFront(), ior, Tdir, cosi, eta);


		// Refraction
		if (hasKt && refractionDone)
		{
			const bool entering = (nIn < nOut);		// Air into object
			Vec3f nOffset = entering ? (-N) : N;

			Ray tRay(shadeInfo.P() + nOffset * kEps, Tdir);
			float distT = BIGFLOAT;
			Color Lt = shadeInfo.TraceSecondaryRay(tRay, distT);

			if (entering)
				Lt *= Absorb(sigma, distT);
			color += kt * (1.0f - F) * Lt;
		}

		// Reflection or Total internal Refraction
		if (hasKr)
		{
			Vec3f Rdir = Reflect(V, N).GetNormalized();
			Ray rRay(shadeInfo.P() + N * kEps, Rdir);
			float distR = BIGFLOAT;
			Color Lr = shadeInfo.TraceSecondaryRay(rRay, distR);

			const Color kReflection = kr + kt * F;
			color += kReflection * Lr;
		}
	}

	// Saturate the color
	color.Clamp();

	return color;
}


Color MtlMicrofacet::Shade(ShadeInfo const& shadeInfo) const
{
	return Color(0, 0, 0);
}